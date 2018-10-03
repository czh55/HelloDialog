#include "MainWindow.h"
#include "ui_MainWindow.h"
#include <QFileDialog>
#include <pcl/common/transforms.h>
#include "SetBGColorDialog.h"
#include "SetPointCloudProDialog.h"
#include <QLayout>
#include <pcl/filters/filter.h>
#include "PlaneDetect.h"
#include "PlaneDetectSetParamDialog.h"
#include <QSettings>
#include <QMessageBox>
#include "RemovePointCloudDialog.h"
#include <omp.h>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // set up the QVTK window
    // viewer.reset(new pcl::visualization::PCLVisualizer);
    ui->qvtkWidget->SetRenderWindow(viewer_cloud.getRenderWindow());
    //viewer->setupInteractor(ui->qvtkWidget->GetInteractor(), ui->qvtkWidget->GetRenderWindow());
    ui->qvtkWidget->update();

    viewer_cloud.addCoordinateSystem(1.0f);
    viewer_cloud.setBackgroundColor(0, 0, 0);
    ui->qvtkWidget->update();

    viewer_cloud.registerPointPickingCallback(pointPickingEventOccurred, (void*)&source_cloud);
    viewer_cloud.registerKeyboardCallback(keyboardEventOccurred_cloud, (void*)&viewer_cloud);

    viewer_ps.addCoordinateSystem(1.0f);
    viewer_ps.registerKeyboardCallback(keyboardEventOccurred_ps, (void*)&viewer_ps);

    RegulateNormalDialog.setVisible(false);
}

MainWindow::~MainWindow()
{
    delete ui;
}

// ���ļ�
void MainWindow::on_openFileAction_triggered()
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("open file"), " ",  tr("pcdFiles(*.pcd)"));
    if(fileName == "") return;
    pcl::io::loadPCDFile(fileName.toStdString(), *source_cloud);
    cout << "loaded " << source_cloud->size() << " points." << endl;
    viewer_cloud.removePointCloud("source");
    viewer_cloud.addPointCloud(source_cloud, "source");
    ui->qvtkWidget->update ();
}
//define by czh
// ����׼����file_1  target_cloud_registration
void MainWindow::on_openFile1RegistrationAction_triggered()
{
	QString fileName = QFileDialog::getOpenFileName(this, tr("open file"), " ", tr("pcdFiles(*.pcd)"));
	if (fileName == "") return;
	pcl::io::loadPCDFile(fileName.toStdString(), *target_cloud_registration);
	cout << "loaded " << target_cloud_registration->size() << " points." << endl;
	viewer_cloud.removePointCloud("source");
	viewer_cloud.addPointCloud(target_cloud_registration, "source");
	ui->qvtkWidget->update();
}

//define by czh
// ����׼����file_2  source_cloud_registration
void MainWindow::on_openFile2RegistrationAction_triggered()
{
	QString fileName = QFileDialog::getOpenFileName(this, tr("open file"), " ", tr("pcdFiles(*.pcd)"));
	if (fileName == "") return;
	pcl::io::loadPCDFile(fileName.toStdString(), *source_cloud_registration);
	cout << "loaded " << source_cloud_registration->size() << " points." << endl;
	viewer_cloud.removePointCloud("source");
	viewer_cloud.addPointCloud(source_cloud_registration, "source");
	ui->qvtkWidget->update();

	//����ת����
	verb_transform(source_cloud_registration, cloud_result);
}

//*******************************************��ʼ���Ƴ�NAN��������˲�ֱ�Ӹ���ԭ���ƶ���*********************************************************************

//define by czh
// �Ƴ�NaN��
//���������source_cloud_registration  target_cloud_registration
//���������source_cloud_registration  target_cloud_registration
void MainWindow::on_removeNanAction_triggered()
{
	/* std::vector<int> indices;
	PointCloudT::Ptr new_cloud (new PointCloudT);
	pcl::removeNaNFromPointCloud<pcl::PointXYZ>(*source_cloud, *new_cloud, indices);
	source_cloud = new_cloud;

	cout << "After nan points been removed, points size =" << source_cloud->size() << endl;*/

	//����ת����
	verb_transform(cloud_result, source_cloud_registration);
	
	//ȥ��NAN�� source_cloud_registration
	std::vector<int> indices_src; //����ȥ���ĵ������
	pcl::removeNaNFromPointCloud(*source_cloud_registration, *source_cloud_registration, indices_src);
	std::cout << "remove *source_cloud_registration nan" << endl;
	//ȥ��NAN�� target_cloud_registration
	std::vector<int> indices_tgt;
	pcl::removeNaNFromPointCloud(*target_cloud_registration, *target_cloud_registration, indices_tgt);
	std::cout << "remove *target_cloud_registration nan" << endl;

	viewer_cloud.removePointCloud("source");
	viewer_cloud.addPointCloud(source_cloud_registration, "source");
	ui->qvtkWidget->update();

	//����ת����
	verb_transform(source_cloud_registration, cloud_result);
}

//define by czh
/*
���������ƽ��кϲ�����˲�
*/
void MainWindow::on_voxelGridFiltMergeCloudAction_triggered() {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr mergeCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	PointCloudT::Ptr cloud_merge(new PointCloudT);

	//����ת����
	//verb_transform(cloud_result, cloud_result);

	//������������е���
	viewer_cloud.removeAllPointClouds();

	//�ϲ���������
	mergePointCloud(target_cloud_registration, cloud_result, mergeCloud);

	//�� PointXYZRGB to PointXYZ
	xyzrgbTransformxyz(mergeCloud, cloud_merge);

	//�²����˲� source_cloud_registration
	pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
	voxel_grid.setLeafSize(0.012, 0.012, 0.012);
	voxel_grid.setInputCloud(cloud_merge);
	int pointCloud_num1 = mergeCloud->size();
	PointCloudT::Ptr cloud_tr(new PointCloudT);
	voxel_grid.filter(*cloud_merge);
	std::cout << "down size *cloud_tr_o from " << pointCloud_num1 << "to" << cloud_merge->size() << endl;

	//��ʾ����
	viewer_cloud.removePointCloud("source");
	viewer_cloud.addPointCloud(cloud_merge, "source");

	//���ֲ�����cloud_merge��ֵ��ȫ��source_cloud������ƽ����ȡ
	*source_cloud = *cloud_merge;
	
}


//define by czh
// ���������˲�
//���������source_cloud_registration  target_cloud_registration
//���������source_cloud_registration  target_cloud_registration
void MainWindow::on_voxelGridFiltAction_triggered()
{
	/* PointCloudT::Ptr cloud_filtered (new PointCloudT);
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(source_cloud);
	float delta = 0.12f;
	sor.setLeafSize(delta, delta, delta);
	sor.filter(*cloud_filtered);
	cout << "after voxel filterd, cloud size = " << cloud_filtered->size() << endl;
	source_cloud = cloud_filtered;
	viewer_cloud.removeAllPointClouds();
	viewer_cloud.addPointCloud(source_cloud, "source");
	viewer_cloud.spinOnce();*/
	
	//����ת����
	verb_transform(cloud_result, source_cloud_registration);

	//������������е���
	viewer_cloud.removeAllPointClouds();

	//�²����˲� source_cloud_registration
	pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
	voxel_grid.setLeafSize(0.012, 0.012, 0.012);
	voxel_grid.setInputCloud(source_cloud_registration);
	int pointCloud_num1 = source_cloud_registration->size();
	PointCloudT::Ptr cloud_tr(new PointCloudT);
	voxel_grid.filter(*source_cloud_registration);
	std::cout << "down size *cloud_tr_o from " << pointCloud_num1 << "to" << source_cloud_registration->size() << endl;
	//�²����˲� cloud_tgt_o
	pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_2;
	voxel_grid_2.setLeafSize(0.012, 0.012, 0.012);
	voxel_grid_2.setInputCloud(target_cloud_registration);
	int pointCloud_num2 = target_cloud_registration->size();
	PointCloudT::Ptr cloud_tgt(new PointCloudT);
	voxel_grid_2.filter(*target_cloud_registration);
	std::cout << "down size *cloud_tgt_o.pcd from " << pointCloud_num2 << "to" << target_cloud_registration->size() << endl;

	//ֱ����ʾ��������
	viewer_cloud.removePointCloud("source");
	viewer_cloud.addPointCloud(source_cloud_registration, "source");
	ui->qvtkWidget->update();

	//����ת����
	verb_transform(source_cloud_registration, cloud_result);
}

//*******************************************�������Ƴ�NAN��������˲�ֱ�Ӹ���ԭ���ƶ���*********************************************************************

//define by czh
//��תԭ����
//�������ݣ�source_cloud_registration
//������ݣ�cloud_tr
void MainWindow::on_rotatePointCloudAction_triggered() {
	//����ת����
	verb_transform(cloud_result, source_cloud_registration);

	//������������е���
	viewer_cloud.removeAllPointClouds();

	//tools->transformation(*source_cloud_registration, *cloud_icp, transformation_matrix);
	transformation(*source_cloud_registration, *cloud_tr);

	//ֱ����ʾ��������
	viewer_cloud.removePointCloud("source");
	viewer_cloud.addPointCloud(cloud_tr, "source");
	ui->qvtkWidget->update();

	//����ת����
	verb_transform(cloud_tr, cloud_result);
}
//define by czh 
//SAC��׼
//�������ݣ�cloud_tr target_cloud_registration 
//������ݣ�cloud_icp
void MainWindow::on_registrationSACAction_triggered() {
	//����ת����
	verb_transform(cloud_result, cloud_sac);

	//������������е���
	viewer_cloud.removeAllPointClouds();

	//������淨��1
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_src;
	ne_src.setInputCloud(cloud_sac);
	pcl::search::KdTree< pcl::PointXYZ>::Ptr tree_src(new pcl::search::KdTree< pcl::PointXYZ>());
	ne_src.setSearchMethod(tree_src);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_src_normals(new pcl::PointCloud< pcl::Normal>);
	ne_src.setRadiusSearch(0.02);
	ne_src.compute(*cloud_src_normals);
	//������淨��2
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_tgt;
	ne_tgt.setInputCloud(target_cloud_registration);
	pcl::search::KdTree< pcl::PointXYZ>::Ptr tree_tgt(new pcl::search::KdTree< pcl::PointXYZ>());
	ne_tgt.setSearchMethod(tree_tgt);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_tgt_normals(new pcl::PointCloud< pcl::Normal>);
	//ne_tgt.setKSearch(20);
	ne_tgt.setRadiusSearch(0.02);
	ne_tgt.compute(*cloud_tgt_normals);

	//����FPFH 1
	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_src;
	fpfh_src.setInputCloud(cloud_sac);
	fpfh_src.setInputNormals(cloud_src_normals);
	pcl::search::KdTree<PointT>::Ptr tree_src_fpfh(new pcl::search::KdTree<PointT>);
	fpfh_src.setSearchMethod(tree_src_fpfh);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_src(new pcl::PointCloud<pcl::FPFHSignature33>());
	fpfh_src.setRadiusSearch(0.05);
	fpfh_src.compute(*fpfhs_src);
	std::cout << "compute *cloud_tr fpfh" << endl;
	//����FPFH 2
	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_tgt;
	fpfh_tgt.setInputCloud(target_cloud_registration);
	fpfh_tgt.setInputNormals(cloud_tgt_normals);
	pcl::search::KdTree<PointT>::Ptr tree_tgt_fpfh(new pcl::search::KdTree<PointT>);
	fpfh_tgt.setSearchMethod(tree_tgt_fpfh);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_tgt(new pcl::PointCloud<pcl::FPFHSignature33>());
	fpfh_tgt.setRadiusSearch(0.05);
	fpfh_tgt.compute(*fpfhs_tgt);
	std::cout << "compute *cloud_tgt fpfh" << endl;

	//SAC��׼
	pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> scia;
	scia.setInputSource(cloud_sac);
	scia.setInputTarget(target_cloud_registration);
	scia.setSourceFeatures(fpfhs_src);
	scia.setTargetFeatures(fpfhs_tgt);
	//scia.setMinSampleDistance(1);
	//scia.setNumberOfSamples(2);
	//scia.setCorrespondenceRandomness(20);
	scia.align(*cloud_sac);
	//���SAC��ת����
	std::cout << "sac has converged:" << scia.hasConverged() << "  score: " << scia.getFitnessScore() << endl;
	Eigen::Matrix4f sac_trans;
	sac_trans = scia.getFinalTransformation();
	std::cout << sac_trans << endl;
	clock_t sac_time = clock();
	std::cout << "SAC done" << endl;
	//�½�ҳ����ʾSACЧ��
	visualization(target_cloud_registration, cloud_tr, cloud_sac);

	//ֱ����ʾSACЧ��-������
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr mergeCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	mergePointCloud(target_cloud_registration, cloud_sac, mergeCloud);
	viewer_cloud.removePointCloud("source");
	viewer_cloud.addPointCloud(mergeCloud, "source");
	ui->qvtkWidget->update();

	//����ת����
	verb_transform(cloud_sac, cloud_result);

}

//define by czh
//��׼
//�������ݣ�target_cloud_registration cloud_icp
//������ݣ�cloud_icp
void MainWindow::on_registrationICPAction_triggered()
{
	//����ת����
	verb_transform(cloud_result, cloud_icp);

	//������������е���
	viewer_cloud.removeAllPointClouds();

	pcl::console::TicToc time;
	time.tic();

	pcl::IterativeClosestPoint<PointT, PointT> icp;
	icp.setMaximumIterations(iterations);
	icp.setInputSource(cloud_icp);
	icp.setInputTarget(target_cloud_registration);
	icp.align(*cloud_icp);
	icp.setMaximumIterations(1);  // We set this variable to 1 for the next time we will call .align () function
	std::cout << "Applied " << iterations << " ICP iteration(s) in " << time.toc() << " ms" << std::endl;
	
	if (icp.hasConverged())
	{
		std::cout << "\nICP has converged, score is " << icp.getFitnessScore() << std::endl;
		std::cout << "\nICP transformation " << iterations << " : cloud_icp -> cloud_in" << std::endl;
		//transformation_matrix = icp.getFinalTransformation().cast<double>();
		//print4x4Matrix(transformation_matrix);

		//update file
		//tools->savePointCloudFile(cloud_in_1, cloud_icp, iterations);
	}
	else
	{
		//���ICP�㷨�Ƿ���������Ȼ���˳�����
		PCL_ERROR("\nICP has not converged.\n");
		system("pause");
		return;
	}
	
	//˫ͨ����ʾ����
	visualization(target_cloud_registration, cloud_tr, cloud_icp);
	//�������
	savePointCloudFile(target_cloud_registration, cloud_icp, "icp-run"+iterations);
	
	//ֱ����ʾICPЧ��-������
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr mergeCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	mergePointCloud(target_cloud_registration, cloud_icp, mergeCloud);
	viewer_cloud.removePointCloud("source");
	viewer_cloud.addPointCloud(mergeCloud, "source");
	ui->qvtkWidget->update();

	//����ת����
	verb_transform(cloud_icp, cloud_result);
}

//define by czh
/*����ƽ�����׼
ƽ����׼ͬ��ʹ���˱���ת������ֻ������д�˵�һ�������һ���������ڲ���
*/
void MainWindow::on_registrationPlaneAction_triggered() {
	//������������е���
	viewer_cloud.removeAllPointClouds();

	/*��ִ��ƽ����׼ʱ��Ҫ������õ�ǰ������������Ƶ�ƽ������*.pcd *.txt
	*��ʵ���������������Ƕ�PlaneDetect.h�к��������ʹ�ö��ѡ�
	*/

	/*ԭ�����ȫ�ֱ���source_cloud��ƽ����ȡ*/
	//��ʱ����source_cloud��temp_cloud��ʹ����֮���ٻָ�
	PointCloudT::Ptr temp_cloud(new PointCloudT);
	*temp_cloud = *source_cloud;
	//��source_cloud��ֵΪsource_cloud_registration
	*source_cloud = *source_cloud_registration;
	//Ϊƽ����ȡ���ز���
	on_load_param_action_triggered();
	//���Ʒ�����
	on_normalEstimateAction_triggered();
	//�Զ�ִ��
	on_autoPerformAction_triggered();
	//������������
	on_savePolyDataAction_triggered();

	//��source_cloud��ֵΪsource_cloud_registration
	*source_cloud = *target_cloud_registration;
	//Ϊƽ����ȡ���ز���
	on_load_param_action_triggered();
	//���Ʒ�����
	on_normalEstimateAction_triggered();
	//�Զ�ִ��
	on_autoPerformAction_triggered();
	//������������
	on_savePolyDataAction_triggered();

	//�ָ�source_cloud
	*source_cloud = *temp_cloud;

	start = std::clock();
	cout << "auto run......." << endl;
	loadpolygon();
	while (source_index < source_polygon.size())
	{
		FindSimilarPoly();
		cout << "match continue..." << endl;
		source_index++;
		IsMatch = false;
	}
	registration();
	registration_cloud();
	//icp();
	finish = std::clock();
	cout << "finally cost " << finish - start << " ms." << endl;
	memset(state, 0, CMD_SIZE);
}
//define by czh
//�׶��޸�
//��������
void MainWindow::on_repairHolesAction_triggered() {

}
// �ı䱳����ɫ
void MainWindow::on_bgColorMenu_triggered()
{
    SetBGColorDialog dlg;

    if(dlg.exec() != QDialog::Accepted) return;

    int r, g, b;
    dlg.getRGB(r, g, b);

    viewer_cloud.setBackgroundColor(r,g,b);
    viewer_cloud.spinOnce();
    ui->qvtkWidget->update ();
}
// �ı������ɫ
void MainWindow::on_pointCloudColorMenu_triggered()
{
    SetPointCloudProDialog *dlg = new SetPointCloudProDialog();
    if(dlg->exec() != QDialog::Accepted) return;

    int r,g,b,size;
    QString q_cloud_id;
    dlg->getData(r,g,b,size, q_cloud_id);
    delete dlg;

    viewer_cloud.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, q_cloud_id.toStdString());
    viewer_cloud.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, q_cloud_id.toStdString());
}
// �ѵ����ƶ�������
void MainWindow::on_translateToCentroidAction_triggered()
{
    if(source_cloud->size()==0) return;
    pcl::PointXYZ p;
    p.x = p.y = p.z = 0;
    for(int i = 0; i < source_cloud->size(); ++i)
    {
        p.x += source_cloud->points[i].x;
        p.y += source_cloud->points[i].y;
        p.z += source_cloud->points[i].z;
    }
    p.x /= float(source_cloud->size());
    p.y /= float(source_cloud->size());
    p.z /= float(source_cloud->size());

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << -1.0*p.x,
                               -1.0*p.y,
                               -1.0*p.z;
    pcl::transformPointCloud(*source_cloud, *source_cloud, transform);
    viewer_cloud.updatePointCloud(source_cloud, "source");
    viewer_cloud.resetCamera();
    ui->qvtkWidget->update();
    cout << "done." << endl;
}

// ����ƽ����ȡ����
void MainWindow::on_plane_detect_set_param_Action_triggered()
{
    PlaneDetectSetParamDialog dlg;
    dlg.exec();
}
// �Ƴ������
void MainWindow::on_removeRedundantPointsAction_triggered()
{
    std::vector<bool> is_redundant(source_cloud->size(), false);
    kdtree_source.setInputCloud(source_cloud);
    std::vector<int> indices;
    std::vector<float> dists;
    for(int i = 0; i < source_cloud->size(); ++i)
    {
        if(is_redundant[i]) continue;
        kdtree_source.radiusSearch(source_cloud->points[i], min_dist_between_points, indices, dists);

        for(int j = 1; j < indices.size(); ++j)
            is_redundant[indices[j]] = true;
    }
    PointCloudT::Ptr new_cloud (new PointCloudT);
    new_cloud->reserve(source_cloud->size());
    for(int i = 0; i < source_cloud->size(); ++i)
    {
        if(!is_redundant[i])
            new_cloud->push_back(source_cloud->points[i]);
    }
    source_cloud = new_cloud;
    cout << "after remove redundant points, cloud size = " << source_cloud->size() << endl;
}

// ��������
// Ĭ��y�ᳯ�ϣ�x��ˮƽ��z��ָ�����巽��
// �򵥴���swap y��z��ֵ��zֵ�ٳ���-1
void MainWindow::on_regulateCoorAction_triggered()
{
    float tmp;
    for(int i = 0; i < source_cloud->size(); ++i)
    {
        tmp = source_cloud->points[i].y;
        source_cloud->points[i].y = source_cloud->points[i].z;
        source_cloud->points[i].z = tmp;
        source_cloud->points[i].z *= -1.0f;
    }
    viewer_cloud.updatePointCloud(source_cloud, "source");
}

// Ϊƽ����ȡ���ز���
void MainWindow::on_load_param_action_triggered()
{
    QSettings configIni("config.txt", QSettings::IniFormat);

    isConductTranslate = configIni.value("PlaneDetect/isConductTranslate").toBool();
    min_dist_between_points = configIni.value("PlaneDetect/min_dist_between_points").toFloat();
    r_for_estimate_normal = configIni.value("PlaneDetect/r_for_estimate_normal").toFloat();
    interval_level = configIni.value("PlaneDetect/interval_level").toInt();
    normal_length_for_display = configIni.value("PlaneDetect/normal_length_for_display").toFloat();
    normal_length_for_selected = configIni.value("PlaneDetect/normal_length_for_selected").toFloat();
    is_norm_direction_valid = configIni.value("PlaneDetect/is_norm_direction_valid").toBool();
    r_for_regulate_normal = configIni.value("PlaneDetect/r_for_regulate_normal").toFloat();
    num_res = configIni.value("PlaneDetect/num_res").toFloat();
    color_gain = configIni.value("PlaneDetect/color_gain").toFloat();
    std_dev_gaussian = configIni.value("PlaneDetect/std_dev_gaussian").toFloat();
    search_radius_create_gradient = configIni.value("PlaneDetect/search_radius_create_gradient").toFloat();
    T_vote_num = configIni.value("PlaneDetect/T_vote_num").toInt();
    radius_base = configIni.value("PlaneDetect/radius_base").toFloat();
    delta_radius = configIni.value("PlaneDetect/delta_radius").toFloat();
    S_threshold = configIni.value("PlaneDetect/S_threshold").toFloat();
    dev_threshold = configIni.value("PlaneDetect/dev_threshold").toFloat();
    search_radius_for_plane_seg = configIni.value("PlaneDetect/search_radius_for_plane_seg").toFloat();
    T_num_of_single_plane = configIni.value("PlaneDetect/T_num_of_single_plane").toInt();
    radius_border = configIni.value("PlaneDetect/radius_border").toFloat();
    radius_local = configIni.value("PlaneDetect/radius_local").toFloat();
    T_angle_bias_integrate = configIni.value("PlaneDetect/T_angle_bias_integrate").toFloat();
    r_local = configIni.value("PlaneDetect/r_local").toFloat();
    T_curvature_bias = configIni.value("PlaneDetect/T_curvature_bias").toFloat();
    T_point_normal_bias = configIni.value("PlaneDetect/T_point_normal_bias").toFloat();
    T_ratio_merge_planes = configIni.value("PlaneDetect/T_ratio_merge_planes").toFloat();
    alpha_poly = configIni.value("PlaneDetect/alpha_poly").toFloat();
    T_dist_point_plane = configIni.value("PlaneDetect/T_dist_point_plane").toFloat();
    T_cluster_num = configIni.value("PlaneDetect/T_cluster_num").toInt();
    cout << "�����������" << endl;

    kdtree_source.setInputCloud(source_cloud);
}
// ���Ʒ�����
void MainWindow::on_normalEstimateAction_triggered()
{
    estimateNormal();
    memset(state, 0, CMD_SIZE);
    strcpy(state, "estimate normal");
}
// У�����Ʒ�����
void MainWindow::on_regulateNormalAction_triggered()
{
    memset(state, 0, CMD_SIZE);
    strcpy(state, "regulate normal");
    RegulateNormalDialog.show();
}
// ִ�з�����У������
void MainWindow::on_performRegulateAction_triggered()
{
    is_norm_direction_valid = RegulateNormalDialog.is_norm_direction_valid;
    regulateNormal();
}
// �Ƴ�����
void MainWindow::on_removePointCloudAction_triggered()
{
   RemovePointCloudDialog dlg;
   if(dlg.exec() == QDialog::Accepted)
   {
       viewer_cloud.removePointCloud(dlg.cloudID.toStdString());
   }
}
// ���������ռ�
void MainWindow::on_createPSAction_triggered()
{
    start = std::clock();
    createPS();
    voteForPS();
    finish = std::clock();
    cout << "parameter space has been constructed, ps has " << ps_cloud->size() << " elements." << endl;
    cout << "time used: " << finish - start << "ms." << endl;

    memset(state, 0, CMD_SIZE);
    strcpy(state, "create ps");
    processStateMsg();
}
// ���淨���������ļ�
void MainWindow::on_savePointNormalFileAction_triggered()
{
    QString fileName = QFileDialog::getSaveFileName(this,
            tr("Open PCD Files"),
            "",
            tr("PCD Files (*.pcd)"));

    if (!fileName.isNull())
    {
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointNormal>);
        pcl::PointNormal p;
        for(int i = 0; i < source_cloud->size(); ++i)
        {
            p.x = source_cloud->points[i].x;
            p.y = source_cloud->points[i].y;
            p.z = source_cloud->points[i].z;
            p.normal_x = source_normal->points[i].normal_x;
            p.normal_y = source_normal->points[i].normal_y;
            p.normal_z = source_normal->points[i].normal_z;
            cloud->push_back(p);
        }
        pcl::io::savePCDFile(fileName.toStdString(), *cloud);
        cout << "file:" << fileName.toStdString() << " has been saved." << endl;
    }
}
// �򿪴��з������ĵ����ļ�
void MainWindow::on_openPointCloudNormalFileAction_triggered()
{
    on_load_param_action_triggered();
    QString fileName = QFileDialog::getOpenFileName(this, tr("open file"), " ",  tr("pcdFiles(*.pcd)"));
    if(fileName == "") return;
    strcpy(pcdFileName, fileName.toStdString().data());
    loadPointNormal();
    cout << "loaded " << source_cloud->size() << " points." << endl;
}
// ƽ�������ռ�
void MainWindow::on_filtPSAction_triggered()
{
    start = std::clock();
    filtPS();
    finish = std::clock();
    cout << "filt used: " << finish-start << "ms." << endl;
    memset(state, 0, CMD_SIZE);
    strcpy(state, "filt ps");
    processStateMsg();
}
// �ָ�����ռ�
void MainWindow::on_segPSAction_triggered()
{
    start = std::clock();
    createGradientField();
    rectifySinkFields();
    finish = std::clock();
    cout << "segment ps used " << finish-start << "ms." << endl;
    memset(state, 0, CMD_SIZE);
    strcpy(state, "segment ps");
    processStateMsg();
}
// �ָ�ƽ��
void MainWindow::on_segPlaneAction_triggered()
{
    start = std::clock();
    segmentPlanes();
    finish = std::clock();
    cout << "plane segmentation used " << finish-start << "ms." << endl;
    cout << "plane number = " << plane_clouds.size() << endl;

    // ����������Ԫ����
    for(int i = 0; i < plane_clouds.size(); ++i)
    {
        Plane plane;
        PointCloudT::Ptr cloud (new PointCloudT);
        *cloud = *plane_clouds[i].points_set;
        plane.points_set = cloud;
        growth_unit_set.push_back(plane);
    }

    memset(state, 0, CMD_SIZE);
    strcpy(state, "segment planes");
    processStateMsg();
}
// ��������
void MainWindow::on_regionGrowingAction_triggered()
{
    start = std::clock();
    growPlaneArea();
    finish = std::clock();
    cout << "plane growing used " << finish-start << "ms." << endl;
    memset(state, 0, CMD_SIZE);
    strcpy(state, "grow planes");
    processStateMsg();
}
// ƽ��ϲ�
void MainWindow::on_mergePlanesAction_triggered()
{
    start = std::clock();
    mergePlanes();
    finish = std::clock();
    cout << "plane merging used " << finish-start << "ms." << endl;
    memset(state, 0, CMD_SIZE);
    strcpy(state, "merge planes");
    processStateMsg();
}
// ƽ�����λ�
void MainWindow::on_polyPlanesAction_triggered()
{
    start = std::clock();
    polyPlanes();
    finish = std::clock();
    cout << "plane polygonization used " << finish-start << "ms." << endl;
    memset(state, 0, CMD_SIZE);
    strcpy(state, "poly planes");
    processStateMsg();
}
// ���ƺ���
void MainWindow::on_postProcessAction_triggered()
{
    start = std::clock();
    postProcessPlanes();
    finish = std::clock();
    cout << "plane post process used " << finish-start << "ms." << endl;
    memset(state, 0, CMD_SIZE);
    strcpy(state, "postProcess planes");
    processStateMsg();
}
// ��������
void MainWindow::on_runAgainAction_triggered()
{
    // ������
    on_load_param_action_triggered();

    // ���¹��Ʒ�����
    estimateNormal();
    // У��������
    regulateNormal();

    // ����任
    // ���������ռ�
    start = std::clock();
    createPS();
    voteForPS();
    finish = std::clock();
    cout << "parameter space has been constructed, ps has " << ps_cloud->size() << " elements." << endl;
    cout << "time used: " << finish - start << "ms." << endl;
    // ���˲����ռ�
    start = std::clock();
    filtPS();
    finish = std::clock();
    cout << "filt used: " << finish-start << "ms." << endl;
    // �ָ�����ռ�
    start = std::clock();
    createGradientField();
    rectifySinkFields();
    finish = std::clock();
    cout << "segment ps used " << finish-start << "ms." << endl;
    // �ָ�������Ԫ
    start = std::clock();
    segmentPlanes();
    finish = std::clock();
    cout << "plane segmentation used " << finish-start << "ms." << endl;
    cout << "plane number = " << plane_clouds.size() << endl;
    // ��������
    start = std::clock();
    growPlaneArea();
    finish = std::clock();
    cout << "plane growing used " << finish-start << "ms." << endl;
    // ƽ��ϲ�
    start = std::clock();
    mergePlanes();
    finish = std::clock();
    cout << "plane merging used " << finish-start << "ms." << endl;
    // ����λ�
    start = std::clock();
    polyPlanes();
    finish = std::clock();
    cout << "plane polygonization used " << finish-start << "ms." << endl;
    // ����
    start = std::clock();
    postProcessPlanes();
    finish = std::clock();
    cout << "plane post process used " << finish-start << "ms." << endl;
    memset(state, 0, CMD_SIZE);
    strcpy(state, "postProcess planes");
    processStateMsg();
}
// �Զ�ִ��
void MainWindow::on_autoPerformAction_triggered()
{
    // ����任
    // ���������ռ�
    start = std::clock();
    createPS();
    voteForPS();
    finish = std::clock();
    cout << "parameter space has been constructed, ps has " << ps_cloud->size() << " elements." << endl;
    cout << "time used: " << finish - start << "ms." << endl;
    // ���˲����ռ�
    start = std::clock();
    filtPS();
    finish = std::clock();
    cout << "filt used: " << finish-start << "ms." << endl;
    // �ָ�����ռ�
    start = std::clock();
    createGradientField();
    rectifySinkFields();
    finish = std::clock();
    cout << "segment ps used " << finish-start << "ms." << endl;
    // �ָ�������Ԫ
    start = std::clock();
    segmentPlanes();
    finish = std::clock();
    cout << "plane segmentation used " << finish-start << "ms." << endl;
    cout << "plane number = " << plane_clouds.size() << endl;
    // ��������
    start = std::clock();
    growPlaneArea();
    finish = std::clock();
    cout << "plane growing used " << finish-start << "ms." << endl;
    // ƽ��ϲ�
    start = std::clock();
    mergePlanes();
    finish = std::clock();
    cout << "plane merging used " << finish-start << "ms." << endl;
    // ����λ�
    start = std::clock();
    polyPlanes();
    finish = std::clock();
    cout << "plane polygonization used " << finish-start << "ms." << endl;
    // ����
    start = std::clock();
    postProcessPlanes();
    finish = std::clock();
    cout << "plane post process used " << finish-start << "ms." << endl;
    memset(state, 0, CMD_SIZE);
    strcpy(state, "postProcess planes");
    processStateMsg();
}
// ɾ��ָ���Ķ����
// �������plane_clouds_final, g_selected_point, poly_centroids_cloud g_is_poly_del kdtree_poly_centroids_cloud
void MainWindow::on_editPolyAction_triggered()
{
    // step0: ��ʾ�û�
    cout << "��������ɾ��ģʽ" << endl;
    // step1: ��ʾ��ѡȡ�ص�����
    memset(state, 0, CMD_SIZE);
    strcpy(state, "del poly");
    // step2: ��ʼ������
    poly_centroids_cloud->clear();
    g_is_poly_del.clear();
    poly_centroids_cloud->reserve(plane_clouds_final.size());
    g_is_poly_del.reserve(plane_clouds_final.size());
    // step3: ����ȫ������
    pcl::PointXYZ p;
    int size;
    for(int i = 0; i < plane_clouds_final.size(); ++i)
    {
        g_is_poly_del.push_back(false);
        p.x = p.y = p.z = 0;
        size = plane_clouds_final[i].border->size();
        for(int j = 0; j < size; ++j)
        {
            p.x += plane_clouds_final[i].border->points[j].x;
            p.y += plane_clouds_final[i].border->points[j].y;
            p.z += plane_clouds_final[i].border->points[j].z;
        }
        p.x /= float(size);
        p.y /= float(size);
        p.z /= float(size);
        poly_centroids_cloud->push_back(p);
    }
    kdtree_poly_centroids_cloud.setInputCloud(poly_centroids_cloud);
    // step4: ��ʾ����
    viewer_cloud.removeAllPointClouds();
    viewer_cloud.removeAllShapes();

    // ������ʾΪ��ɫ
    viewer_cloud.addPointCloud(poly_centroids_cloud, "centroids");
    viewer_cloud.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 255, 0, "centroids");
    viewer_cloud.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "centroids");
    for(int i = 0; i < plane_clouds_final.size(); ++i)
    {
        QString cloud_id;
        cloud_id = QString::number(i, 10);
        viewer_cloud.addPointCloud(plane_clouds_final[i].border, cloud_id.toStdString());
    }

    g_selected_poly_id = -1;
}
// ȷ��ɾ�������
// g_selected_poly_id
void MainWindow::on_delPolyAction_triggered()
{
    g_is_poly_del[g_selected_poly_id] = true;
    cout << "��ɾ���Ķ����id = " << g_selected_poly_id << endl;
    for(int i = 0; i < g_is_poly_del.size(); ++i)
    {
        if(g_is_poly_del[i])
        {
            QString cloud_id = QString::number(i, 10);
            viewer_cloud.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 255, 0, 0, cloud_id.toStdString());
            viewer_cloud.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, cloud_id.toStdString());
            ui->qvtkWidget->update();
            // cout << cloud_id.toStdString() << endl;
        }
    }
}
// ִ��ɾ������
void MainWindow::on_performDelAction_triggered()
{
    std::vector<int> offset(plane_clouds_final.size(), 0);
    int count = 0;
    for(int i = 0; i < plane_clouds_final.size(); ++i)
    {
        if(g_is_poly_del[i])
        {
            ++count;
            offset[i] = -1;
            continue;
        }
        offset[i] = count;
    }

    for(int i = 0; i < offset.size(); ++i)
    {
        if(offset[i] == -1) continue;
        if(offset[i] > 0)
        {
            plane_clouds_final[i - offset[i]] = plane_clouds_final[i];
        }
    }

    for(int i = 0; i < count; ++i)
    {
        plane_clouds_final.pop_back();
    }

    // ������ͼ
    on_editPolyAction_triggered();
}
// ������������
void MainWindow::on_savePolyDataAction_triggered()
{
    QString fileName = QFileDialog::getSaveFileName(this,
            tr("Open PCD Files"),
            "",
            tr("PCD Files (*.pcd)"));

    if (!fileName.isNull())
    {
        // ���涥������
        PointCloudT::Ptr vertices (new PointCloudT);
        for(int ii = 0; ii < plane_clouds_final.size(); ++ii)
        {
            for(int i = 0; i < plane_clouds_final[ii].border->size(); ++i)
            {
                vertices->push_back(plane_clouds_final[ii].border->points[i]);
            }
        }
        pcl::io::savePCDFileASCII(fileName.toStdString(), *vertices);
        // ����ÿ������ε�size
        fileName += ".txt";
        std::ofstream file(fileName.toStdString(), std::ios::out);

        for(int ii = 0; ii < plane_clouds_final.size(); ++ii)
        {
            file << plane_clouds_final[ii].border->size() << endl;
        }
        // ����ÿ������εķ�����
        pcl::PointCloud<pcl::Normal>::Ptr plane_normals (new pcl::PointCloud<pcl::Normal>);
        pcl::Normal pn;
        for(int i = 0; i < plane_clouds_final.size(); ++i)
        {
            Plane plane = plane_clouds_final[i];
            pn.normal_x = plane.coeff.values[0];
            pn.normal_y = plane.coeff.values[1];
            pn.normal_z = plane.coeff.values[2];
            plane_normals->push_back(pn);
        }
        fileName += ".pcd";
        pcl::io::savePCDFileASCII(fileName.toStdString(), *plane_normals);
        cout << "all " << plane_clouds_final.size() << " polys' data have been saved." << endl;
    }
}
// ���������޼�ģʽ
void MainWindow::on_enterPruneModeAction_triggered()
{
    // step0: ��ʾ�û�
    cout << "���������޼�ģʽ" << endl;
    // step1: ��ʾ��ѡȡ�ص�����
    memset(state, 0, CMD_SIZE);
    strcpy(state, "prune poly");
    // step2: ��ʼ������
    poly_centroids_cloud->clear();
    g_is_poly_del.clear();
    poly_centroids_cloud->reserve(plane_clouds_final.size());
    g_is_poly_del.reserve(plane_clouds_final.size());
    // step3: ����ȫ������
    pcl::PointXYZ p;
    int size;
    for(int i = 0; i < plane_clouds_final.size(); ++i)
    {
        g_is_poly_del.push_back(false);
        p.x = p.y = p.z = 0;
        size = plane_clouds_final[i].border->size();
        for(int j = 0; j < size; ++j)
        {
            p.x += plane_clouds_final[i].border->points[j].x;
            p.y += plane_clouds_final[i].border->points[j].y;
            p.z += plane_clouds_final[i].border->points[j].z;
        }
        p.x /= float(size);
        p.y /= float(size);
        p.z /= float(size);
        poly_centroids_cloud->push_back(p);
    }
    kdtree_poly_centroids_cloud.setInputCloud(poly_centroids_cloud);
    // step4: ��ʾ����
    viewer_cloud.removeAllPointClouds();
    viewer_cloud.removeAllShapes();

    // ������ʾΪ��ɫ
    viewer_cloud.addPointCloud(poly_centroids_cloud, "centroids");
    viewer_cloud.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 255, 0, "centroids");
    viewer_cloud.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "centroids");
    for(int i = 0; i < plane_clouds_final.size(); ++i)
    {
        QString cloud_id;
        cloud_id = QString::number(i, 10);
        viewer_cloud.addPointCloud(plane_clouds_final[i].border, cloud_id.toStdString());
    }

    g_selected_poly_id = -1;
    // ��ʾ�û�ѡ��ĳ�������
    cout << "ͨ��ѡ������ѡ���ض������" << endl;
}
// ѡ�е�ǰ�����
// ����ʾ��ǰ�����
void MainWindow::on_selCurPolyAction_triggered()
{
    viewer_cloud.removeAllPointClouds();
    viewer_cloud.removeAllShapes();

    viewer_cloud.addPointCloud(plane_clouds_final[g_selected_poly_id].border, "border");
    // ��ʾ�û�ѡ�񶥵�
    cout << "���Կ�ʼѡ���һ������" << endl;
    memset(state, 0, CMD_SIZE);
    strcpy(state, "prune poly select points");
}
// ����ѡ�е�Ϊ��һ������
// ѡ�еĵ�Ϊ��ɫ
void MainWindow::on_setFirstPointAction_triggered()
{
    first_point = g_selected_point;
    PointCloudT::Ptr cloud (new PointCloudT);
    cloud->push_back(first_point);
    viewer_cloud.removePointCloud("point");
    viewer_cloud.removePointCloud("first point");
    viewer_cloud.addPointCloud(cloud, "first point");
    viewer_cloud.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 255, 0, "first point");
    viewer_cloud.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "first point");
    cout << "��һ������ѡ��" << endl;
}
// ����ѡ�е�Ϊ�ڶ�������
void MainWindow::on_setSecondPointAction_triggered()
{
    second_point = g_selected_point;
    PointCloudT::Ptr cloud (new PointCloudT);
    cloud->push_back(second_point);
    viewer_cloud.removePointCloud("point");
    viewer_cloud.removePointCloud("second point");
    viewer_cloud.addPointCloud(cloud, "second point");
    viewer_cloud.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 255, 0, "second point");
    viewer_cloud.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "second point");
    cout << "�ڶ�������ѡ��" << endl;
}
// ִ�ж�����зֲ���
// ������һ�������ͨ��ѡ������㹹�ɵ�ֱ�߶ΰ�ԭ������зֳ����������
void MainWindow::on_performPolyCutAction_triggered()
{
    // step0: �ֱ��ҵ���һ����͵ڶ�������ԭ��������е�����
    int first_index = -1;
    int second_index = -1;
    for(int i = 0; i < plane_clouds_final[g_selected_poly_id].border->size(); ++i)
    {
        if( plane_clouds_final[g_selected_poly_id].border->points[i].x == first_point.x &&
            plane_clouds_final[g_selected_poly_id].border->points[i].y == first_point.y &&
            plane_clouds_final[g_selected_poly_id].border->points[i].z == first_point.z ){
            first_index = i;
        }
        if( plane_clouds_final[g_selected_poly_id].border->points[i].x == second_point.x &&
            plane_clouds_final[g_selected_poly_id].border->points[i].y == second_point.y &&
            plane_clouds_final[g_selected_poly_id].border->points[i].z == second_point.z ){
            second_index = i;
        }
        if(first_index != -1 && second_index != -1)
            break;
    }
    cout << "first_index = " << first_index << "; second_index = " << second_index << endl;
    // step1: ���ñ���
    PointCloudT::Ptr cloud_backup (new PointCloudT);
    *cloud_backup = *plane_clouds_final[g_selected_poly_id].border;
    plane_clouds_final[g_selected_poly_id].border->clear();

    // step2: ԭ�ж���Σ�first_index->second_index
    int cur_index = first_index;
    while(true)
    {
        plane_clouds_final[g_selected_poly_id].border->push_back(cloud_backup->points[cur_index]);

        if(cur_index == second_index) break;

        cur_index = cur_index == cloud_backup->size()-1 ? 0 : cur_index + 1;
    }

    cout << "ok1" << endl;

    // step3: ����һ�������:second_index -> first_index
    Plane plane;
    plane.border.reset(new PointCloudT);
    plane.border->reserve(cloud_backup->size() - plane_clouds_final[g_selected_poly_id].border->size());
    cur_index = second_index;
    while(true)
    {
        plane.border->push_back(cloud_backup->points[cur_index]);
        if(cur_index == first_index) break;
        cur_index = cur_index == cloud_backup->size()-1 ? 0 : cur_index + 1;
    }
    plane.coeff = plane_clouds_final[g_selected_poly_id].coeff;
    plane_clouds_final.push_back(plane);

    cout << "ok2" << endl;

    // step4: ������ͼ
    viewer_cloud.removeAllPointClouds();
    viewer_cloud.removeAllShapes();
    on_enterPruneModeAction_triggered();
}
// ��ʾҪɾ�����߶�
void MainWindow::on_displayLineSegAction_triggered()
{
    // step0: �ֱ��ҵ���һ����͵ڶ�������ԭ��������е�����
    int first_index = -1;
    int second_index = -1;
    for(int i = 0; i < plane_clouds_final[g_selected_poly_id].border->size(); ++i)
    {
        if( plane_clouds_final[g_selected_poly_id].border->points[i].x == first_point.x &&
            plane_clouds_final[g_selected_poly_id].border->points[i].y == first_point.y &&
            plane_clouds_final[g_selected_poly_id].border->points[i].z == first_point.z ){
            first_index = i;
        }
        if( plane_clouds_final[g_selected_poly_id].border->points[i].x == second_point.x &&
            plane_clouds_final[g_selected_poly_id].border->points[i].y == second_point.y &&
            plane_clouds_final[g_selected_poly_id].border->points[i].z == second_point.z ){
            second_index = i;
        }
        if(first_index != -1 && second_index != -1)
            break;
    }
    cout << "first_index = " << first_index << "; second_index = " << second_index << endl;

    // step1: �ҵ���first_point �� second_point֮��ĵ�
    PointCloudT::Ptr line_points (new PointCloudT);
    int cur_index = first_index == plane_clouds_final[g_selected_poly_id].border->size() - 1 ?
                    0 : first_index + 1;
    while(cur_index != second_index)
    {
        line_points->push_back(plane_clouds_final[g_selected_poly_id].border->points[cur_index]);
        cur_index = cur_index == plane_clouds_final[g_selected_poly_id].border->size() - 1 ?
                    0 : cur_index + 1;
    }
    viewer_cloud.removePointCloud("line seg");
    viewer_cloud.addPointCloud(line_points, "line seg");
    viewer_cloud.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 255, 0, 0, "line seg");
    viewer_cloud.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "line seg");
}
// �л��߶�
void MainWindow::on_switchLineSegAction_triggered()
{
    pcl::PointXYZ tmp_point = first_point;
    first_point = second_point;
    second_point = tmp_point;
    on_displayLineSegAction_triggered();
}
// ִ��ɾ��
void MainWindow::on_performLineSegDelAction_triggered()
{
    // step0: �ֱ��ҵ���һ����͵ڶ�������ԭ��������е�����
    int first_index = -1;
    int second_index = -1;
    for(int i = 0; i < plane_clouds_final[g_selected_poly_id].border->size(); ++i)
    {
        if( plane_clouds_final[g_selected_poly_id].border->points[i].x == first_point.x &&
            plane_clouds_final[g_selected_poly_id].border->points[i].y == first_point.y &&
            plane_clouds_final[g_selected_poly_id].border->points[i].z == first_point.z ){
            first_index = i;
        }
        if( plane_clouds_final[g_selected_poly_id].border->points[i].x == second_point.x &&
            plane_clouds_final[g_selected_poly_id].border->points[i].y == second_point.y &&
            plane_clouds_final[g_selected_poly_id].border->points[i].z == second_point.z ){
            second_index = i;
        }
        if(first_index != -1 && second_index != -1)
            break;
    }

    // step1: ��[second_index first_index]������ĵ㱣������
    std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> line_points;
    int cur_index = second_index;
    while(true)
    {
        line_points.push_back(plane_clouds_final[g_selected_poly_id].border->points[cur_index]);
        if(cur_index == first_index) break;
        cur_index = cur_index == plane_clouds_final[g_selected_poly_id].border->size() - 1 ?
                    0 : cur_index + 1;
    }
    plane_clouds_final[g_selected_poly_id].border->clear();
    for(int i = 0; i < line_points.size(); ++i)
    {
        plane_clouds_final[g_selected_poly_id].border->push_back(line_points[i]);
    }

    on_selCurPolyAction_triggered();
}
// ���ض��������
void MainWindow::on_loadPolyDataAction_triggered()
{

}
