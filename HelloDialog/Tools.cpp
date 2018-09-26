#include "Tools.h"

//void Tools::transformation(const PointCloudT &souce_cloud, PointCloudT &icp_cloud, Eigen::Matrix4d transformation_matrix) {
void Tools::transformation(Eigen::Matrix4d transformation_matrix) {

	// A rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
	double theta = M_PI / 8;  // The angle of rotation in radians
	transformation_matrix(0, 0) = cos(theta);
	transformation_matrix(0, 1) = -sin(theta);
	transformation_matrix(1, 0) = sin(theta);
	transformation_matrix(1, 1) = cos(theta);

	// A translation on Z axis (0.4 meters)
	transformation_matrix(2, 3) = 0.4;

	// Display in terminal the transformation matrix
	std::cout << "Applying this rigid transformation to: cloud_in -> cloud_icp" << std::endl;
	print4x4Matrix(transformation_matrix);

	// Executing the transformation
	pcl::transformPointCloud(*source_cloud2_registration, *cloud_icp, transformation_matrix);

}

void Tools::print4x4Matrix(const Eigen::Matrix4d & matrix)
{
	printf("Rotation matrix :\n");
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
	printf("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
	printf("Translation vector :\n");
	printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
}

//void Tools::visualization(PointCloudT::Ptr source_cloud1_registration, PointCloudT::Ptr source_cloud2_registration, PointCloudT::Ptr cloud_tr, PointCloudT::Ptr cloud_icp, int iterations)
void Tools::visualization()
{
	// Visualization
	pcl::visualization::PCLVisualizer viewer("ICP demo");
	// Create two vertically separated viewports
	int v1(0);
	int v2(1);
	viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);

	// The color we will be using
	float bckgr_gray_level = 0.0;  // Black
	float txt_gray_lvl = 1.0 - bckgr_gray_level;

	// Original point cloud is white
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h(source_cloud1_registration, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl,
		(int)255 * txt_gray_lvl);
	viewer.addPointCloud(source_cloud1_registration, cloud_in_color_h, "source_cloud1_registration_v1", v1);
	viewer.addPointCloud(source_cloud1_registration, cloud_in_color_h, "source_cloud1_registration_v2", v2);

	// Transformed point cloud is green
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h(cloud_tr, 20, 180, 20);
	viewer.addPointCloud(cloud_tr, cloud_tr_color_h, "cloud_tr_v1", v1);

	// ICP aligned point cloud is red
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h(cloud_icp, 180, 20, 20);
	viewer.addPointCloud(cloud_icp, cloud_icp_color_h, "cloud_icp_v2", v2);

	// Orginal point cloud is blue
	//pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_all_color_h(cloud_in_all, 0, 191, 255);
	//viewer.addPointCloud(cloud_in_all, cloud_all_color_h, "cloud_all_v2", v2);

	// Adding text descriptions in each viewport
	viewer.addText("White: Original point cloud\nGreen: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
	viewer.addText("White: Original point cloud\nRed: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);

	std::stringstream ss;
	ss << iterations;
	std::string iterations_cnt = "ICP iterations = " + ss.str();
	viewer.addText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);

	// Set background color
	viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
	viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);

	// Set camera position and orientation
	viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
	viewer.setSize(1280, 1024);  // Visualiser window size
}


//void Tools::savePointCloudFile(PointCloudT::Ptr source_cloud1_registration, PointCloudT::Ptr cloud_icp, int iterations) {
void Tools::savePointCloudFile() {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr mergeCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	for (int j = 0; j < source_cloud1_registration->points.size(); j += 1)
	{
		pcl::PointXYZRGB p;
		p.x = source_cloud1_registration->points[j].x;
		p.y = source_cloud1_registration->points[j].y;
		p.z = source_cloud1_registration->points[j].z;
		p.r = 255;//��ɫ
		p.g = 0;
		p.b = 0;
		mergeCloud->points.push_back(p);
	}

	for (int j = 0; j < cloud_icp->points.size(); j += 1)
	{
		pcl::PointXYZRGB p;
		p.x = cloud_icp->points[j].x;
		p.y = cloud_icp->points[j].y;
		p.z = cloud_icp->points[j].z;
		p.r = 20;//��ɫ
		p.g = 180;
		p.b = 20;
		mergeCloud->points.push_back(p);
	}
	// ���ò��������
	mergeCloud->height = 1;
	mergeCloud->width = mergeCloud->points.size();
	mergeCloud->is_dense = false;

	std::stringstream ss;
	ss << iterations;
	std::string fileName = "ICPmerge2-in-out-icp-" + ss.str() + ".pcd";

	pcl::io::savePCDFile(fileName, *mergeCloud);

	// ������ݲ��˳�
	mergeCloud->points.clear();
	std::cout << "�ѱ���Ϊ" << fileName << std::endl;

}


