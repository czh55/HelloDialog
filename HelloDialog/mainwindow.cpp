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

// 打开文件
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
// 打开配准点云file_1  target_cloud_registration
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
// 打开配准点云file_2  source_cloud_registration
void MainWindow::on_openFile2RegistrationAction_triggered()
{
	QString fileName = QFileDialog::getOpenFileName(this, tr("open file"), " ", tr("pcdFiles(*.pcd)"));
	if (fileName == "") return;
	pcl::io::loadPCDFile(fileName.toStdString(), *source_cloud_registration);
	cout << "loaded " << source_cloud_registration->size() << " points." << endl;
	viewer_cloud.removePointCloud("source");
	viewer_cloud.addPointCloud(source_cloud_registration, "source");
	ui->qvtkWidget->update();

	//变量转换器
	verb_transform(source_cloud_registration, cloud_result);
}

//*******************************************开始：移除NAN点和体素滤波直接覆盖原点云对象*********************************************************************

//define by czh
// 移除NaN点
//输入变量：source_cloud_registration  target_cloud_registration
//输出变量：source_cloud_registration  target_cloud_registration
void MainWindow::on_removeNanAction_triggered()
{
	/* std::vector<int> indices;
	PointCloudT::Ptr new_cloud (new PointCloudT);
	pcl::removeNaNFromPointCloud<pcl::PointXYZ>(*source_cloud, *new_cloud, indices);
	source_cloud = new_cloud;

	cout << "After nan points been removed, points size =" << source_cloud->size() << endl;*/

	//变量转换器
	verb_transform(cloud_result, source_cloud_registration);
	
	//去除NAN点 source_cloud_registration
	std::vector<int> indices_src; //保存去除的点的索引
	pcl::removeNaNFromPointCloud(*source_cloud_registration, *source_cloud_registration, indices_src);
	std::cout << "remove *source_cloud_registration nan" << endl;
	//去除NAN点 target_cloud_registration
	std::vector<int> indices_tgt;
	pcl::removeNaNFromPointCloud(*target_cloud_registration, *target_cloud_registration, indices_tgt);
	std::cout << "remove *target_cloud_registration nan" << endl;

	viewer_cloud.removePointCloud("source");
	viewer_cloud.addPointCloud(source_cloud_registration, "source");
	ui->qvtkWidget->update();

	//变量转换器
	verb_transform(source_cloud_registration, cloud_result);
}

//define by czh
/*
将两个点云进行合并后的滤波
*/
void MainWindow::on_voxelGridFiltMergeCloudAction_triggered() {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr mergeCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	PointCloudT::Ptr cloud_merge(new PointCloudT);

	//变量转换器
	//verb_transform(cloud_result, cloud_result);

	//清除界面上所有点云
	viewer_cloud.removeAllPointClouds();

	//合并两个点云
	mergePointCloud(target_cloud_registration, cloud_result, mergeCloud);

	//将 PointXYZRGB to PointXYZ
	xyzrgbTransformxyz(mergeCloud, cloud_merge);

	//下采样滤波 source_cloud_registration
	pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
	voxel_grid.setLeafSize(0.012, 0.012, 0.012);
	voxel_grid.setInputCloud(cloud_merge);
	int pointCloud_num1 = mergeCloud->size();
	PointCloudT::Ptr cloud_tr(new PointCloudT);
	voxel_grid.filter(*cloud_merge);
	std::cout << "down size *cloud_tr_o from " << pointCloud_num1 << "to" << cloud_merge->size() << endl;

	//显示点云
	viewer_cloud.removePointCloud("source");
	viewer_cloud.addPointCloud(cloud_merge, "source");

	//将局部变量cloud_merge赋值给全局source_cloud，进行平面提取
	*source_cloud = *cloud_merge;
	
}


//define by czh
// 进行体素滤波
//输入变量：source_cloud_registration  target_cloud_registration
//输出变量：source_cloud_registration  target_cloud_registration
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
	
	//变量转换器
	verb_transform(cloud_result, source_cloud_registration);

	//清除界面上所有点云
	viewer_cloud.removeAllPointClouds();

	//下采样滤波 source_cloud_registration
	pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
	voxel_grid.setLeafSize(0.012, 0.012, 0.012);
	voxel_grid.setInputCloud(source_cloud_registration);
	int pointCloud_num1 = source_cloud_registration->size();
	PointCloudT::Ptr cloud_tr(new PointCloudT);
	voxel_grid.filter(*source_cloud_registration);
	std::cout << "down size *cloud_tr_o from " << pointCloud_num1 << "to" << source_cloud_registration->size() << endl;
	//下采样滤波 cloud_tgt_o
	pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_2;
	voxel_grid_2.setLeafSize(0.012, 0.012, 0.012);
	voxel_grid_2.setInputCloud(target_cloud_registration);
	int pointCloud_num2 = target_cloud_registration->size();
	PointCloudT::Ptr cloud_tgt(new PointCloudT);
	voxel_grid_2.filter(*target_cloud_registration);
	std::cout << "down size *cloud_tgt_o.pcd from " << pointCloud_num2 << "to" << target_cloud_registration->size() << endl;

	//直接显示在主界面
	viewer_cloud.removePointCloud("source");
	viewer_cloud.addPointCloud(source_cloud_registration, "source");
	ui->qvtkWidget->update();

	//变量转换器
	verb_transform(source_cloud_registration, cloud_result);
}

//*******************************************结束：移除NAN点和体素滤波直接覆盖原点云对象*********************************************************************

//define by czh
//旋转原点云
//输入数据：source_cloud_registration
//输出数据：cloud_tr
void MainWindow::on_rotatePointCloudAction_triggered() {
	//变量转换器
	verb_transform(cloud_result, source_cloud_registration);

	//清除界面上所有点云
	viewer_cloud.removeAllPointClouds();

	//tools->transformation(*source_cloud_registration, *cloud_icp, transformation_matrix);
	transformation(*source_cloud_registration, *cloud_tr);

	//直接显示在主界面
	viewer_cloud.removePointCloud("source");
	viewer_cloud.addPointCloud(cloud_tr, "source");
	ui->qvtkWidget->update();

	//变量转换器
	verb_transform(cloud_tr, cloud_result);
}
//define by czh 
//SAC配准
//输入数据：cloud_tr target_cloud_registration 
//输出数据：cloud_icp
void MainWindow::on_registrationSACAction_triggered() {
	//变量转换器
	verb_transform(cloud_result, cloud_sac);

	//清除界面上所有点云
	viewer_cloud.removeAllPointClouds();

	//计算表面法线1
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_src;
	ne_src.setInputCloud(cloud_sac);
	pcl::search::KdTree< pcl::PointXYZ>::Ptr tree_src(new pcl::search::KdTree< pcl::PointXYZ>());
	ne_src.setSearchMethod(tree_src);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_src_normals(new pcl::PointCloud< pcl::Normal>);
	ne_src.setRadiusSearch(0.02);
	ne_src.compute(*cloud_src_normals);
	//计算表面法线2
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_tgt;
	ne_tgt.setInputCloud(target_cloud_registration);
	pcl::search::KdTree< pcl::PointXYZ>::Ptr tree_tgt(new pcl::search::KdTree< pcl::PointXYZ>());
	ne_tgt.setSearchMethod(tree_tgt);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_tgt_normals(new pcl::PointCloud< pcl::Normal>);
	//ne_tgt.setKSearch(20);
	ne_tgt.setRadiusSearch(0.02);
	ne_tgt.compute(*cloud_tgt_normals);

	//计算FPFH 1
	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_src;
	fpfh_src.setInputCloud(cloud_sac);
	fpfh_src.setInputNormals(cloud_src_normals);
	pcl::search::KdTree<PointT>::Ptr tree_src_fpfh(new pcl::search::KdTree<PointT>);
	fpfh_src.setSearchMethod(tree_src_fpfh);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_src(new pcl::PointCloud<pcl::FPFHSignature33>());
	fpfh_src.setRadiusSearch(0.05);
	fpfh_src.compute(*fpfhs_src);
	std::cout << "compute *cloud_tr fpfh" << endl;
	//计算FPFH 2
	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_tgt;
	fpfh_tgt.setInputCloud(target_cloud_registration);
	fpfh_tgt.setInputNormals(cloud_tgt_normals);
	pcl::search::KdTree<PointT>::Ptr tree_tgt_fpfh(new pcl::search::KdTree<PointT>);
	fpfh_tgt.setSearchMethod(tree_tgt_fpfh);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_tgt(new pcl::PointCloud<pcl::FPFHSignature33>());
	fpfh_tgt.setRadiusSearch(0.05);
	fpfh_tgt.compute(*fpfhs_tgt);
	std::cout << "compute *cloud_tgt fpfh" << endl;

	//SAC配准
	pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> scia;
	scia.setInputSource(cloud_sac);
	scia.setInputTarget(target_cloud_registration);
	scia.setSourceFeatures(fpfhs_src);
	scia.setTargetFeatures(fpfhs_tgt);
	//scia.setMinSampleDistance(1);
	//scia.setNumberOfSamples(2);
	//scia.setCorrespondenceRandomness(20);
	scia.align(*cloud_sac);
	//输出SAC旋转矩阵
	std::cout << "sac has converged:" << scia.hasConverged() << "  score: " << scia.getFitnessScore() << endl;
	Eigen::Matrix4f sac_trans;
	sac_trans = scia.getFinalTransformation();
	std::cout << sac_trans << endl;
	clock_t sac_time = clock();
	std::cout << "SAC done" << endl;
	//新建页面显示SAC效果
	visualization(target_cloud_registration, cloud_tr, cloud_sac);

	//直接显示SAC效果-单点云
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr mergeCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	mergePointCloud(target_cloud_registration, cloud_sac, mergeCloud);
	viewer_cloud.removePointCloud("source");
	viewer_cloud.addPointCloud(mergeCloud, "source");
	ui->qvtkWidget->update();

	//变量转换器
	verb_transform(cloud_sac, cloud_result);

}

//define by czh
//配准
//输入数据：target_cloud_registration cloud_icp
//输出数据：cloud_icp
void MainWindow::on_registrationICPAction_triggered()
{
	//变量转换器
	verb_transform(cloud_result, cloud_icp);

	//清除界面上所有点云
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
		//检测ICP算法是否收敛，不然就退出程序。
		PCL_ERROR("\nICP has not converged.\n");
		system("pause");
		return;
	}
	
	//双通道显示点云
	visualization(target_cloud_registration, cloud_tr, cloud_icp);
	//保存点云
	savePointCloudFile(target_cloud_registration, cloud_icp, "icp-run"+iterations);
	
	//直接显示ICP效果-单点云
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr mergeCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	mergePointCloud(target_cloud_registration, cloud_icp, mergeCloud);
	viewer_cloud.removePointCloud("source");
	viewer_cloud.addPointCloud(mergeCloud, "source");
	ui->qvtkWidget->update();

	//变量转换器
	verb_transform(cloud_icp, cloud_result);
}

//define by czh
/*基于平面的配准
平面配准同样使用了变量转换器，只不过是写了第一个和最后一个函数的内部！
*/
void MainWindow::on_registrationPlaneAction_triggered() {
	//清除界面上所有点云
	viewer_cloud.removeAllPointClouds();

	/*在执行平面配准时，要事先求得当前输入的两个点云的平面数据*.pcd *.txt
	*其实下面三个函数都是对PlaneDetect.h中函数的组合使用而已。
	*/

	/*原程序对全局变量source_cloud的平面提取*/
	//临时保存source_cloud到temp_cloud，使用完之后，再恢复
	PointCloudT::Ptr temp_cloud(new PointCloudT);
	*temp_cloud = *source_cloud;
	//将source_cloud赋值为source_cloud_registration
	*source_cloud = *source_cloud_registration;
	//为平面提取加载参数
	on_load_param_action_triggered();
	//估计法向量
	on_normalEstimateAction_triggered();
	//自动执行
	on_autoPerformAction_triggered();
	//保存多边形数据
	on_savePolyDataAction_triggered();

	//将source_cloud赋值为source_cloud_registration
	*source_cloud = *target_cloud_registration;
	//为平面提取加载参数
	on_load_param_action_triggered();
	//估计法向量
	on_normalEstimateAction_triggered();
	//自动执行
	on_autoPerformAction_triggered();
	//保存多边形数据
	on_savePolyDataAction_triggered();

	//恢复source_cloud
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
//孔洞修复
//输入数据
void MainWindow::on_repairHolesAction_triggered() {

}
// 改变背景颜色
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
// 改变点云颜色
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
// 把点云移动至重心
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

// 设置平面提取参数
void MainWindow::on_plane_detect_set_param_Action_triggered()
{
    PlaneDetectSetParamDialog dlg;
    dlg.exec();
}
// 移除冗余点
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

// 修正坐标
// 默认y轴朝上；x轴水平；z轴指向岩体方向
// 简单处理：swap y与z的值，z值再乘以-1
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

// 为平面提取加载参数
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
    cout << "参数加载完成" << endl;

    kdtree_source.setInputCloud(source_cloud);
}
// 估计法向量
void MainWindow::on_normalEstimateAction_triggered()
{
    estimateNormal();
    memset(state, 0, CMD_SIZE);
    strcpy(state, "estimate normal");
}
// 校正点云法向量
void MainWindow::on_regulateNormalAction_triggered()
{
    memset(state, 0, CMD_SIZE);
    strcpy(state, "regulate normal");
    RegulateNormalDialog.show();
}
// 执行法向量校正操作
void MainWindow::on_performRegulateAction_triggered()
{
    is_norm_direction_valid = RegulateNormalDialog.is_norm_direction_valid;
    regulateNormal();
}
// 移除点云
void MainWindow::on_removePointCloudAction_triggered()
{
   RemovePointCloudDialog dlg;
   if(dlg.exec() == QDialog::Accepted)
   {
       viewer_cloud.removePointCloud(dlg.cloudID.toStdString());
   }
}
// 创建参数空间
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
// 保存法向量点云文件
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
// 打开带有法向量的点云文件
void MainWindow::on_openPointCloudNormalFileAction_triggered()
{
    on_load_param_action_triggered();
    QString fileName = QFileDialog::getOpenFileName(this, tr("open file"), " ",  tr("pcdFiles(*.pcd)"));
    if(fileName == "") return;
    strcpy(pcdFileName, fileName.toStdString().data());
    loadPointNormal();
    cout << "loaded " << source_cloud->size() << " points." << endl;
}
// 平滑参数空间
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
// 分割参数空间
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
// 分割平面
void MainWindow::on_segPlaneAction_triggered()
{
    start = std::clock();
    segmentPlanes();
    finish = std::clock();
    cout << "plane segmentation used " << finish-start << "ms." << endl;
    cout << "plane number = " << plane_clouds.size() << endl;

    // 设置增长单元集合
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
// 区域生长
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
// 平面合并
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
// 平面多边形化
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
// 点云后处理
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
// 继续运行
void MainWindow::on_runAgainAction_triggered()
{
    // 读参数
    on_load_param_action_triggered();

    // 重新估计法向量
    estimateNormal();
    // 校正法向量
    regulateNormal();

    // 霍夫变换
    // 创建参数空间
    start = std::clock();
    createPS();
    voteForPS();
    finish = std::clock();
    cout << "parameter space has been constructed, ps has " << ps_cloud->size() << " elements." << endl;
    cout << "time used: " << finish - start << "ms." << endl;
    // 过滤参数空间
    start = std::clock();
    filtPS();
    finish = std::clock();
    cout << "filt used: " << finish-start << "ms." << endl;
    // 分割参数空间
    start = std::clock();
    createGradientField();
    rectifySinkFields();
    finish = std::clock();
    cout << "segment ps used " << finish-start << "ms." << endl;
    // 分割增长单元
    start = std::clock();
    segmentPlanes();
    finish = std::clock();
    cout << "plane segmentation used " << finish-start << "ms." << endl;
    cout << "plane number = " << plane_clouds.size() << endl;
    // 区域生长
    start = std::clock();
    growPlaneArea();
    finish = std::clock();
    cout << "plane growing used " << finish-start << "ms." << endl;
    // 平面合并
    start = std::clock();
    mergePlanes();
    finish = std::clock();
    cout << "plane merging used " << finish-start << "ms." << endl;
    // 多边形化
    start = std::clock();
    polyPlanes();
    finish = std::clock();
    cout << "plane polygonization used " << finish-start << "ms." << endl;
    // 后处理
    start = std::clock();
    postProcessPlanes();
    finish = std::clock();
    cout << "plane post process used " << finish-start << "ms." << endl;
    memset(state, 0, CMD_SIZE);
    strcpy(state, "postProcess planes");
    processStateMsg();
}
// 自动执行
void MainWindow::on_autoPerformAction_triggered()
{
    // 霍夫变换
    // 创建参数空间
    start = std::clock();
    createPS();
    voteForPS();
    finish = std::clock();
    cout << "parameter space has been constructed, ps has " << ps_cloud->size() << " elements." << endl;
    cout << "time used: " << finish - start << "ms." << endl;
    // 过滤参数空间
    start = std::clock();
    filtPS();
    finish = std::clock();
    cout << "filt used: " << finish-start << "ms." << endl;
    // 分割参数空间
    start = std::clock();
    createGradientField();
    rectifySinkFields();
    finish = std::clock();
    cout << "segment ps used " << finish-start << "ms." << endl;
    // 分割增长单元
    start = std::clock();
    segmentPlanes();
    finish = std::clock();
    cout << "plane segmentation used " << finish-start << "ms." << endl;
    cout << "plane number = " << plane_clouds.size() << endl;
    // 区域生长
    start = std::clock();
    growPlaneArea();
    finish = std::clock();
    cout << "plane growing used " << finish-start << "ms." << endl;
    // 平面合并
    start = std::clock();
    mergePlanes();
    finish = std::clock();
    cout << "plane merging used " << finish-start << "ms." << endl;
    // 多边形化
    start = std::clock();
    polyPlanes();
    finish = std::clock();
    cout << "plane polygonization used " << finish-start << "ms." << endl;
    // 后处理
    start = std::clock();
    postProcessPlanes();
    finish = std::clock();
    cout << "plane post process used " << finish-start << "ms." << endl;
    memset(state, 0, CMD_SIZE);
    strcpy(state, "postProcess planes");
    processStateMsg();
}
// 删除指定的多边形
// 处理对象：plane_clouds_final, g_selected_point, poly_centroids_cloud g_is_poly_del kdtree_poly_centroids_cloud
void MainWindow::on_editPolyAction_triggered()
{
    // step0: 提示用户
    cout << "进入多边形删除模式" << endl;
    // step1: 提示点选取回调函数
    memset(state, 0, CMD_SIZE);
    strcpy(state, "del poly");
    // step2: 初始化数据
    poly_centroids_cloud->clear();
    g_is_poly_del.clear();
    poly_centroids_cloud->reserve(plane_clouds_final.size());
    g_is_poly_del.reserve(plane_clouds_final.size());
    // step3: 设置全局数据
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
    // step4: 显示数据
    viewer_cloud.removeAllPointClouds();
    viewer_cloud.removeAllShapes();

    // 重心显示为绿色
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
// 确定删除多边形
// g_selected_poly_id
void MainWindow::on_delPolyAction_triggered()
{
    g_is_poly_del[g_selected_poly_id] = true;
    cout << "被删除的多边形id = " << g_selected_poly_id << endl;
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
// 执行删除操作
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

    // 更新视图
    on_editPolyAction_triggered();
}
// 保存多边形数据
void MainWindow::on_savePolyDataAction_triggered()
{
    QString fileName = QFileDialog::getSaveFileName(this,
            tr("Open PCD Files"),
            "",
            tr("PCD Files (*.pcd)"));

    if (!fileName.isNull())
    {
        // 保存顶点序列
        PointCloudT::Ptr vertices (new PointCloudT);
        for(int ii = 0; ii < plane_clouds_final.size(); ++ii)
        {
            for(int i = 0; i < plane_clouds_final[ii].border->size(); ++i)
            {
                vertices->push_back(plane_clouds_final[ii].border->points[i]);
            }
        }
        pcl::io::savePCDFileASCII(fileName.toStdString(), *vertices);
        // 保存每个多边形的size
        fileName += ".txt";
        std::ofstream file(fileName.toStdString(), std::ios::out);

        for(int ii = 0; ii < plane_clouds_final.size(); ++ii)
        {
            file << plane_clouds_final[ii].border->size() << endl;
        }
        // 保存每个多边形的法向量
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
// 进入多边形修剪模式
void MainWindow::on_enterPruneModeAction_triggered()
{
    // step0: 提示用户
    cout << "进入多边形修剪模式" << endl;
    // step1: 提示点选取回调函数
    memset(state, 0, CMD_SIZE);
    strcpy(state, "prune poly");
    // step2: 初始化数据
    poly_centroids_cloud->clear();
    g_is_poly_del.clear();
    poly_centroids_cloud->reserve(plane_clouds_final.size());
    g_is_poly_del.reserve(plane_clouds_final.size());
    // step3: 设置全局数据
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
    // step4: 显示数据
    viewer_cloud.removeAllPointClouds();
    viewer_cloud.removeAllShapes();

    // 重心显示为绿色
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
    // 提示用户选择某个多边形
    cout << "通过选中重心选择特定多边形" << endl;
}
// 选中当前多边形
// 仅显示当前多边形
void MainWindow::on_selCurPolyAction_triggered()
{
    viewer_cloud.removeAllPointClouds();
    viewer_cloud.removeAllShapes();

    viewer_cloud.addPointCloud(plane_clouds_final[g_selected_poly_id].border, "border");
    // 提示用户选择顶点
    cout << "可以开始选择第一个顶点" << endl;
    memset(state, 0, CMD_SIZE);
    strcpy(state, "prune poly select points");
}
// 设置选中点为第一个顶点
// 选中的点为绿色
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
    cout << "第一个点已选中" << endl;
}
// 设置选中点为第二个顶点
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
    cout << "第二个点已选中" << endl;
}
// 执行多边形切分操作
// 即，将一个多边形通过选择的两点构成的直线段把原多边形切分成两个多边形
void MainWindow::on_performPolyCutAction_triggered()
{
    // step0: 分别找到第一个点和第二个点在原来多边形中的索引
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
    // step1: 做好备份
    PointCloudT::Ptr cloud_backup (new PointCloudT);
    *cloud_backup = *plane_clouds_final[g_selected_poly_id].border;
    plane_clouds_final[g_selected_poly_id].border->clear();

    // step2: 原有多边形：first_index->second_index
    int cur_index = first_index;
    while(true)
    {
        plane_clouds_final[g_selected_poly_id].border->push_back(cloud_backup->points[cur_index]);

        if(cur_index == second_index) break;

        cur_index = cur_index == cloud_backup->size()-1 ? 0 : cur_index + 1;
    }

    cout << "ok1" << endl;

    // step3: 增加一个多边形:second_index -> first_index
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

    // step4: 更新视图
    viewer_cloud.removeAllPointClouds();
    viewer_cloud.removeAllShapes();
    on_enterPruneModeAction_triggered();
}
// 显示要删除的线段
void MainWindow::on_displayLineSegAction_triggered()
{
    // step0: 分别找到第一个点和第二个点在原来多边形中的索引
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

    // step1: 找到从first_point 到 second_point之间的点
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
// 切换线段
void MainWindow::on_switchLineSegAction_triggered()
{
    pcl::PointXYZ tmp_point = first_point;
    first_point = second_point;
    second_point = tmp_point;
    on_displayLineSegAction_triggered();
}
// 执行删除
void MainWindow::on_performLineSegDelAction_triggered()
{
    // step0: 分别找到第一个点和第二个点在原来多边形中的索引
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

    // step1: 将[second_index first_index]闭区间的点保存起来
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
// 加载多边形数据
void MainWindow::on_loadPolyDataAction_triggered()
{

}
