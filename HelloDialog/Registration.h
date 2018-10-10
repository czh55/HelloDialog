#pragma once
//define by czh
#ifndef REGISTRATION_H
#define REGISTRATION_H

#include "HeaderFile.h"

//define by czh 
//��Ϊÿһ�����ܵ���������
PointCloudT::Ptr cloud_result(new PointCloudT);

//define by czh
//����׼�У�ÿ�����ܣ�ִ��ǰ��ִ�к�ı���ת����
void verb_transform(PointCloudT::Ptr give, PointCloudT::Ptr get) {
	*get = *give;
}


//define by czh
/**
����任����
*/
void print4x4Matrix(const Eigen::Matrix4d & matrix)
{
	printf("Rotation matrix :\n");
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
	printf("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
	printf("Translation vector :\n");
	printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
}
/**
��ת���ƣ�Ĭ�ϵ���ת�Ǧ�/8
������
1.cloud_origin ��Ҫ��ת�ĵ���
2.cloud_target ��ת�Ľ��
*/
void transformation(const PointCloudT &cloud_origin, PointCloudT &cloud_target) {
	// Defining a rotation matrix and translation vector
	Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();

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
	pcl::transformPointCloud(cloud_origin, cloud_target, transformation_matrix);

}

/**
˫ͨ����ʾ��
������
1.cloud_double ����ͨ��ȫ����ʾ ��ɫ
2.cloud_left ����ʾ�����ͨ�� ��ɫ
3.cloud_right ����ʾ���ұ�ͨ�� ��ɫ
*/
void visualization(PointCloudT::Ptr cloud_double, PointCloudT::Ptr cloud_left, PointCloudT::Ptr cloud_right)
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
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_double_color_h(cloud_double, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl,
		(int)255 * txt_gray_lvl);
	viewer.addPointCloud(cloud_double, cloud_double_color_h, "target_cloud_registration_v1", v1);
	viewer.addPointCloud(cloud_double, cloud_double_color_h, "target_cloud_registration_v2", v2);

	// Transformed point cloud is green
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_left_color_h(cloud_left, 20, 180, 20);
	viewer.addPointCloud(cloud_left, cloud_left_color_h, "cloud_tr_v1", v1);

	// ICP aligned point cloud is red
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_right_color_h(cloud_right, 180, 20, 20);
	viewer.addPointCloud(cloud_right, cloud_right_color_h, "cloud_icp_v2", v2);

	// Orginal point cloud is blue
	//pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_all_color_h(cloud_in_all, 0, 191, 255);
	//viewer.addPointCloud(cloud_in_all, cloud_all_color_h, "cloud_all_v2", v2);

	// Adding text descriptions in each viewport
	viewer.addText("White: Original point cloud\nGreen: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
	viewer.addText("White: Original point cloud\nRed: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);

	/*std::stringstream ss;
	ss << iterations;
	std::string iterations_cnt = "ICP iterations = " + ss.str();
	viewer.addText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);*/

	// Set background color
	viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
	viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);

	// Set camera position and orientation
	viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
	viewer.setSize(1280, 1024);  // Visualiser window size

								 // Display the visualiser
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}
}

//define by czh
/*
��xyzrgb��ʽת��Ϊxyz��ʽ�ĵ���
*/
void xyzrgbTransformxyz(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb, PointCloudT::Ptr &cloud_xyz) {
	int M = cloud_xyzrgb->points.size();
	cout << "input size is:" << M << endl;

	for (int i = 0; i <M; i++)
	{
		PointT p;
		p.x = cloud_xyzrgb->points[i].x;
		p.y = cloud_xyzrgb->points[i].y;
		p.z = cloud_xyzrgb->points[i].z;
		cloud_xyz->points.push_back(p);
	}
	cloud_xyz->width = 1;
	cloud_xyz->height = M;
}

//define by czh
/**
���������ƽ��кϲ���������
*/
void  mergePointCloud(PointCloudT::Ptr cloud1, PointCloudT::Ptr cloud2, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &mergeCloud) {

	for (int j = 0; j < cloud1->points.size(); j += 1)
	{
		pcl::PointXYZRGB p;
		p.x = cloud1->points[j].x;
		p.y = cloud1->points[j].y;
		p.z = cloud1->points[j].z;
		p.r = 255;//��ɫ
		p.g = 0;
		p.b = 0;
		mergeCloud->points.push_back(p);
	}

	for (int j = 0; j < cloud2->points.size(); j += 1)
	{
		pcl::PointXYZRGB p;
		p.x = cloud2->points[j].x;
		p.y = cloud2->points[j].y;
		p.z = cloud2->points[j].z;
		p.r = 20;//��ɫ
		p.g = 180;
		p.b = 20;
		mergeCloud->points.push_back(p);
	}
	// ���ò��������
	mergeCloud->height = 1;
	mergeCloud->width = mergeCloud->points.size();
	mergeCloud->is_dense = false;
}

/**
�����ļ���
������
1.����1
2.����2
3.Ҫ���ɵ��ļ���
*/
void savePointCloudFile(PointCloudT::Ptr cloud1, PointCloudT::Ptr cloud2, std::string fileName) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr mergeCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	//�ϲ���������
	mergePointCloud(cloud1, cloud2, mergeCloud);

	//���ļ��������׺pcd
	fileName += ".pcd";

	pcl::io::savePCDFile(fileName, *mergeCloud);

	// ������ݲ��˳�
	mergeCloud->points.clear();
	std::cout << "�ѱ���Ϊ" << fileName << std::endl;

}

//define by czh ***********************************************************************************************************
//��׼��ر���
PointCloudT::Ptr target_cloud_registration(new PointCloudT);
PointCloudT::Ptr source_cloud_registration(new PointCloudT);
PointCloudT::Ptr cloud_tr(new PointCloudT);
PointCloudT::Ptr cloud_icp(new PointCloudT);
PointCloudT::Ptr cloud_sac(new PointCloudT);
//PointCloudT::Ptr cloud_merge(new PointCloudT);


//��׼��������
int iterations = 50;


//define by czh
//��������ƽ����׼�����ļ����ļ���
#define PLANE_REGISTRATION_PATH_pcd1 "3DReconstruction/cloud1.pcd"
#define PLANE_REGISTRATION_PATH_pcd2 "3DReconstruction/cloud2.pcd"
#define PLANE_REGISTRATION_PATH_txt1 "3DReconstruction/cloud1.pcd.txt"
#define PLANE_REGISTRATION_PATH_txt2 "3DReconstruction/cloud2.pcd.txt"


//define by czh /***************************************************************************************************************************************/
/*����;*/
struct My_Polygon
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr border;
	//note by czh :���������ƽ����׼����û���ã�������ֱ�ӳ�������
	Eigen::Vector3f normal;
};
// define by czh /***************************************************************************************************************************************/
/*
����ʦ���õĻ���ƽ�����׼�������Ĳ��֣��Խ�ϵ���ǰ����
��source_cloud_border ����Ϊ source_cloud_border_plane_registration
��target_cloud_border ����Ϊ target_cloud_border_plane_registration
��source_cloud ����Ϊ source_cloud_plane_registration
��target_cloud ����Ϊ target_cloud_plane_registration
��source_cloud_backup ����Ϊ source_cloud_backup_plane_registration
*/
/*ȫ�ֺ����ͱ���;*/
pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_border_plane_registration(new pcl::PointCloud<pcl::PointXYZ>);        //Դ����ƽ��߽�;
pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_border_plane_registration(new pcl::PointCloud<pcl::PointXYZ>);        //Ŀ�����ƽ��߽�;
pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_plane_registration(new pcl::PointCloud<pcl::PointXYZ>);				//Դ����;
pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_plane_registration(new pcl::PointCloud<pcl::PointXYZ>);				//Ŀ�����;
pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_backup_plane_registration(new pcl::PointCloud<pcl::PointXYZ>);

Eigen::Matrix4f transformation_matrix_plane_registration; //�任����;

My_Polygon S_polyAfterN;																			//��һ��֮��Ķ����;
My_Polygon T_polyAfterN;
std::vector<My_Polygon> PolyAfterN(4);

pcl::PointCloud<pcl::PointXYZ>::Ptr S_center(new pcl::PointCloud<pcl::PointXYZ>);				    //��Ӧ����ε�����;
pcl::PointCloud<pcl::PointXYZ>::Ptr T_center(new pcl::PointCloud<pcl::PointXYZ>);

My_Polygon P_poly;																					//�������ֶ����;
																									//note by czh: ��*.pcd �� *.txt�л�ȡ���ݲ������Ľ������ʵ����һ�����ƵĶ��������
std::vector<My_Polygon> source_polygon;																	//Դ����μ���;
std::vector<My_Polygon> target_polygon;																	//Ŀ�����μ���;

																										//�ض���
																										//clock_t start;																						//ʱ��;
																										//clock_t finish;

const float sigma = 0.85;																			//ƥ�����;
const int vertex_diff = 20;																			//�������;   

int source_index = 0;																				//��Ȼƥ��ʱ������;
int target_index = -1;
int p_index = 0;

std::vector<int> S_final_index;																		//��Ӧ���������;
std::vector<int> T_final_index;

bool IsMatch = false;																				//�ж�ƥ���Ƿ�ɹ�;

//�ض���
//const int CMD_SIZE = 64;
//char cmd[CMD_SIZE];
//char state[CMD_SIZE];
//DWORD dwThread;

DWORD WINAPI cmdFunc(LPVOID);
HANDLE dispaly_thread;

//define by czh /***************************************************************************************************************************************/
/*ȫ�ֺ���;*/
void processStateMsg();                                                                             //��Ϣ������;
void loadpolygon();                                                                                 //���ض����;
void FindSimilarPoly();																				//Ѱ�����ƶ����;
void registration();																				//��׼����;
void Match();																						//��������ι�������ƥ��;
void normalize();																					//��һ������;
float GetPolyArea(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);										//���������;
bool PublicArea();																					//��ȡ��������ι�������;
bool GetCrossPoint(int s_first, int s_next, int t_first, int t_next, float &x, float &y);			//�������߶εĽ���;
bool IsRectCross(int s_first, int s_next, int t_first, int t_next);									//�ų�ʵ��;
bool IsLineSegmentCross(int s_first, int s_next, int t_first, int t_next);							//�����ж�;
bool IsPointInPolygon(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const pcl::PointXYZ &pt);			//�жϵ��ǲ����ڶ�����ڲ�; 
void ClockwiseSortPoints();																			//�㼯����;
void registration_cloud();																			//��ʼ������׼;
void icp();
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer_void);
//********************************************************************************************************************************************
//define by czh
/*Ŀ�ģ���ƽ����׼�У�����Ҫ�м��*.pcd ��*.txt�ļ���ֱ��ͨ���������洢
�������漰���ı����У�
1.ƽ�桾��ȡ�����������plane_clouds_final
2.ƽ�桾��׼����Ҫ�����������std::vector<My_Polygon> source_polygon;	std::vector<My_Polygon> target_polygon;
polygonת������
ע������һ��ר�ú���������һ��ͨ�ú�����������������ֻ�����plane_clouds_final��������
*/
std::vector<My_Polygon> polygon_transform() {
	std::vector<My_Polygon> polygons;
	My_Polygon my_polygon;
	for (int i = 0; i < plane_clouds_final.size(); i++) {
		my_polygon.border = plane_clouds_final[i].border;
		polygons.push_back(my_polygon);
	}
	return polygons;
}

/*����Ŀ�ꡢԴ�����Լ����ƽ������;*/
void loadpolygon()																					//���ص���;
{
	//����ת����
	verb_transform(source_cloud_registration, source_cloud_plane_registration);
	//����ת����
	verb_transform(target_cloud_registration, target_cloud_plane_registration);

	cout << "please wait, the polygon is loading..." << endl;										//���ض���ο�ʼ�����ȴ�Դ���ƿ�ʼ;

	source_cloud_border_plane_registration->clear();                                                                   //��ʼ��;
	target_cloud_border_plane_registration->clear();

	int num;
	int sum = 0;
	int i = 0;

	pcl::PointXYZ p;
	My_Polygon polygon;

	vector<int> source;																				//������ÿ������ε�ĸ���;
	vector<int> target;
	vector<int>::iterator it;

	//change by czh
	//pcl::io::loadPCDFile<pcl::PointXYZ>("dataForPlane/source_cloud_plane_registration.pcd", *source_cloud_plane_registration);
	//pcl::io::loadPCDFile<pcl::PointXYZ>("dataForPlane/target_cloud_plane_registration.pcd", *target_cloud_plane_registration);

	*source_cloud_backup_plane_registration = *source_cloud_plane_registration;


	if (pcl::io::loadPCDFile<pcl::PointXYZ>(PLANE_REGISTRATION_PATH_pcd1, *source_cloud_border_plane_registration) == -1)              //��Դ�����;
	{
		PCL_ERROR("Couldn't read file source.pcd \n");
		return;
	}
	ifstream file(PLANE_REGISTRATION_PATH_txt1);
	while (1)
	{
		file >> num;
		sum += num;
		source.push_back(sum);
		if (sum >= source_cloud_border_plane_registration->size())
		{
			break;
		}
	}
	it = source.begin();
	while (it != source.end())
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		for (; i < *it; i++)
		{
			p = source_cloud_border_plane_registration->points[i];
			cloud->points.push_back(p);
		}
		cloud->width = cloud->points.size();
		cloud->height = 1;
		cloud->is_dense = false;
		cloud->points.resize(cloud->width * cloud->height);

		polygon.border = cloud;
		source_polygon.push_back(polygon);

		it++;
	}
	cout << "load source polygon success, total has " << source_polygon.size() << "polygon" << endl;

	if (pcl::io::loadPCDFile<pcl::PointXYZ>(PLANE_REGISTRATION_PATH_pcd2, *target_cloud_border_plane_registration) == -1)
	{
		PCL_ERROR("Couldn't read file target.pcd \n");
		return;
	}
	ifstream file1(PLANE_REGISTRATION_PATH_txt2);

	sum = 0;
	i = 0;

	while (1)
	{
		file1 >> num;
		sum += num;
		target.push_back(sum);
		if (sum >= target_cloud_border_plane_registration->size())
		{
			break;
		}
	}
	it = target.begin();
	while (it != target.end())
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		for (; i < *it; i++)
		{
			p = target_cloud_border_plane_registration->points[i];
			cloud->points.push_back(p);
		}
		cloud->width = cloud->points.size();
		cloud->height = 1;
		cloud->is_dense = false;
		cloud->points.resize(cloud->width * cloud->height);

		polygon.border = cloud;
		target_polygon.push_back(polygon);

		it++;
	}
	cout << "load target polygon success, total has " << target_polygon.size() << "polygon" << endl;
}

/***************************************************************************************************************************************/
/*����γ�ƥ��;*/
void FindSimilarPoly()																			    //������Դ���������ѡȡһ������Σ���Ŀ���������ñ߽��������г�ʼƥ��;
{
	int source_num;																					//ȡ��Ӧ����ε���Ŀ; 
	int target_num;

	while (source_index < source_polygon.size())
	{
		source_num = source_polygon[source_index].border->size();

		for (int i = 0; i < target_polygon.size(); ++i)
		{
			target_num = target_polygon[i].border->size();

			if (abs(target_num - source_num) <= vertex_diff)										//�����������εĶ����ֵ��һ����ֵ�ڣ�����Ϊ���Խ���ƥ��;
			{
				target_index = i;
				cout << "there are two polygon start to math " << endl;
				cout << "source_index = " << source_index << ", total has " << source_num << " vertexes." << endl;
				cout << "target_index = " << target_index << ", total has " << target_num << " vertexes." << endl;
				normalize();

				if (IsMatch == true)                                                                //ƥ��ɹ�����forѭ��;
				{
					viewer_cloud.removeAllPointClouds();
					viewer_cloud.addPointCloud<pcl::PointXYZ>(S_polyAfterN.border, "s");
					viewer_cloud.addPointCloud<pcl::PointXYZ>(PolyAfterN[p_index].border, "t");
					viewer_cloud.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 255, 0, 0, "s");
					viewer_cloud.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 255, 0, "t");
					break;
				}
			}
		}
		if (target_index != -1 && IsMatch == true)												   //��ʾƥ��ɹ�������while;
		{
			break;
		}
		else																					   //else��ʾ������Դ�������Ŀ��������û���ҵ�ƥ��;
		{
			source_index++;
		}
	}
	if (source_index >= source_polygon.size())
	{
		cout << "find finish!...." << endl;
	}
}

/***************************************************************************************************************************************/
/*����ι�һ��;*/
void normalize()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr S_cloudAfterN(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr T_cloudAfterN(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr p_cloud0(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr p_cloud1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr p_cloud2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr p_cloud3(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PointXYZ p, p1, p2, p3;

	*S_cloudAfterN = *source_polygon[source_index].border;
	*T_cloudAfterN = *target_polygon[target_index].border;

	S_polyAfterN.border = S_cloudAfterN;
	T_polyAfterN.border = T_cloudAfterN;

	PolyAfterN[0].border = p_cloud0;
	PolyAfterN[1].border = p_cloud1;
	PolyAfterN[2].border = p_cloud2;
	PolyAfterN[3].border = p_cloud3;

	Eigen::Matrix3f covariance_matrix_source, covariance_matrix_target;                           //Э�������;
	Eigen::Vector4f xyz_centroid_source, xyz_centroid_target;									  //���ĵ�;
	Eigen::Vector3f eigen_values_source, eigen_values_target;									  //����ֵ����������;
	Eigen::Matrix3f eigen_vectors_source, eigen_vectors_target;

	pcl::computeMeanAndCovarianceMatrix(*S_cloudAfterN, covariance_matrix_source, xyz_centroid_source);
	pcl::eigen33(covariance_matrix_source, eigen_vectors_source, eigen_values_source);			  //����Э�����������ֵ����������;

	pcl::computeMeanAndCovarianceMatrix(*T_cloudAfterN, covariance_matrix_target, xyz_centroid_target);
	pcl::eigen33(covariance_matrix_target, eigen_vectors_target, eigen_values_target);			  //����Э�����������ֵ����������;

	for (int i = 0; i < source_polygon[source_index].border->points.size(); ++i)
	{
		S_cloudAfterN->points[i].x = S_cloudAfterN->points[i].x - xyz_centroid_source[0];		  //���Ƚ�����ƽ�Ƶ���������;
		S_cloudAfterN->points[i].y = S_cloudAfterN->points[i].y - xyz_centroid_source[1];
		S_cloudAfterN->points[i].z = S_cloudAfterN->points[i].z - xyz_centroid_source[2];

		p.x = S_cloudAfterN->points[i].x * eigen_vectors_source.coeff(6)						  //��������任;
			+ S_cloudAfterN->points[i].y * eigen_vectors_source.coeff(7)
			+ S_cloudAfterN->points[i].z * eigen_vectors_source.coeff(8);

		p.y = S_cloudAfterN->points[i].x * eigen_vectors_source.coeff(3)
			+ S_cloudAfterN->points[i].y * eigen_vectors_source.coeff(4)
			+ S_cloudAfterN->points[i].z * eigen_vectors_source.coeff(5);

		p.z = 0;

		S_cloudAfterN->points[i] = p;
	}

	for (int i = 0; i < target_polygon[target_index].border->points.size(); ++i)
	{
		T_cloudAfterN->points[i].x = T_cloudAfterN->points[i].x - xyz_centroid_target[0];
		T_cloudAfterN->points[i].y = T_cloudAfterN->points[i].y - xyz_centroid_target[1];
		T_cloudAfterN->points[i].z = T_cloudAfterN->points[i].z - xyz_centroid_target[2];

		p.x = T_cloudAfterN->points[i].x * eigen_vectors_target.coeff(6)
			+ T_cloudAfterN->points[i].y * eigen_vectors_target.coeff(7)
			+ T_cloudAfterN->points[i].z * eigen_vectors_target.coeff(8);

		p.y = T_cloudAfterN->points[i].x * eigen_vectors_target.coeff(3)
			+ T_cloudAfterN->points[i].y * eigen_vectors_target.coeff(4)
			+ T_cloudAfterN->points[i].z * eigen_vectors_target.coeff(5);

		p.z = 0;
		T_cloudAfterN->points[i] = p;
	}
	*PolyAfterN[0].border = *T_cloudAfterN;
	for (int i = 0; i < PolyAfterN[0].border->size(); ++i)
	{
		p = PolyAfterN[0].border->points[i];
		p1.x = p.x; p1.y = -1 * p.y; p1.z = p.z;
		p2.x = -1 * p.x; p2.y = p.y; p2.z = p.z;
		p3.x = -1 * p.x; p3.y = -1 * p.y; p3.z = p.z;
		PolyAfterN[1].border->points.push_back(p1);
		PolyAfterN[2].border->points.push_back(p2);
		PolyAfterN[3].border->points.push_back(p3);
	}
	cout << "coordinate transformation success!, start to match.... " << endl;
	Match();																					  //��ת�������������ν��й�������ƥ��;
}

/***************************************************************************************************************************************/
/*��������ƥ��;*/
bool IsRectCross(int s_first, int s_next, int t_first, int t_next)								  //�ų�ʵ��;
{
	pcl::PointXYZ p1, p2, q1, q2;
	p1 = S_polyAfterN.border->points[s_first];
	p2 = S_polyAfterN.border->points[s_next];
	q1 = T_polyAfterN.border->points[t_first];
	q2 = T_polyAfterN.border->points[t_next];

	bool ret = std::min(p1.x, p2.x) <= std::max(q1.x, q2.x) &&
		std::min(q1.x, q2.x) <= std::max(p1.x, p2.x) &&
		std::min(p1.y, p2.y) <= std::max(q1.y, q2.y) &&
		std::min(q1.y, q2.y) <= std::max(p1.y, p2.y);

	return ret;
}

bool IsLineSegmentCross(int s_first, int s_next, int t_first, int t_next)						  //�����ж�;
{
	long line1, line2;
	pcl::PointXYZ pFirst1, pSecond1, pFirst2, pSecond2;

	pFirst1 = S_polyAfterN.border->points[s_first];
	pSecond1 = S_polyAfterN.border->points[s_next];
	pFirst2 = T_polyAfterN.border->points[t_first];
	pSecond2 = T_polyAfterN.border->points[t_next];

	line1 = pFirst1.x * (pSecond1.y - pFirst2.y) +
		pFirst2.x * (pFirst1.y - pSecond1.y) +
		pSecond1.x * (pFirst2.y - pFirst1.y);
	line2 = pFirst1.x * (pSecond2.y - pFirst2.y) +
		pFirst2.x * (pFirst1.y - pSecond2.y) +
		pSecond2.x * (pFirst2.y - pFirst1.y);
	if (((line1 ^ line2) >= 0) && !(line1 == 0 && line2 == 0))
		return false;

	line1 = pSecond1.x * (pFirst1.y - pSecond2.y) +
		pSecond2.x * (pSecond1.y - pFirst1.y) +
		pFirst1.x * (pSecond2.y - pSecond1.y);
	line2 = pSecond1.x * (pFirst2.y - pSecond2.y) +
		pSecond2.x * (pSecond1.y - pFirst2.y) +
		pFirst2.x * (pSecond2.y - pSecond1.y);
	if (((line1 ^ line2) >= 0) && !(line1 == 0 && line2 == 0))
		return false;

	return true;
}
bool GetCrossPoint(int s_first, int s_next, int t_first, int t_next, float &x, float &y)		  //�������߶εĽ���;
{
	pcl::PointXYZ p1, p2, q1, q2;
	p1 = S_polyAfterN.border->points[s_first];
	p2 = S_polyAfterN.border->points[s_next];
	q1 = T_polyAfterN.border->points[t_first];
	q2 = T_polyAfterN.border->points[t_next];

	if (IsRectCross(s_first, s_next, t_first, t_next))
	{
		if (IsLineSegmentCross(s_first, s_next, t_first, t_next))
		{
			float tmpLeft, tmpRight;

			tmpLeft = (q2.x - q1.x) * (p1.y - p2.y) - (p2.x - p1.x) * (q1.y - q2.y);
			tmpRight = (p1.y - q1.y) * (p2.x - p1.x) * (q2.x - q1.x) + q1.x * (q2.y - q1.y) * (p2.x - p1.x) - p1.x * (p2.y - p1.y) * (q2.x - q1.x);

			x = tmpRight / tmpLeft;

			tmpLeft = (p1.x - p2.x) * (q2.y - q1.y) - (p2.y - p1.y) * (q1.x - q2.x);
			tmpRight = p2.y * (p1.x - p2.x) * (q2.y - q1.y) + (q2.x - p2.x) * (q2.y - q1.y) * (p1.y - p2.y) - q2.y * (q1.x - q2.x) * (p2.y - p1.y);
			y = tmpRight / tmpLeft;
			return true;
		}
	}
	return false;
}

bool IsPointInPolygon(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const pcl::PointXYZ &pt)		  //�жϵ��ǲ����ڶ�����ڲ�; 
{
	int i, j;
	bool c = false;

	for (i = 0, j = cloud->size() - 1; i < cloud->size(); j = i++)
	{
		if ((((cloud->points[i].y <= pt.y) && (pt.y < cloud->points[j].y)) || ((cloud->points[j].y <= pt.y) && (pt.y < cloud->points[i].y)))
			&& (pt.x < (cloud->points[j].x - cloud->points[i].x) * (pt.y - cloud->points[i].y) / (cloud->points[j].y - cloud->points[i].y) + cloud->points[i].x))
		{
			c = !c;
		}
	}
	return c;
}

void ClockwiseSortPoints()																		  //�㼯����;
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointXYZ center;
	pcl::PointXYZ p;

	float x = 0, y = 0, size;
	size = P_poly.border->size();
	for (int i = 0; i < size; i++)																  //�������ĵ�;
	{
		x += P_poly.border->points[i].x;
		y += P_poly.border->points[i].y;
	}
	center.x = x / size;
	center.y = y / size;
	multimap<float, int> map_indices1;
	multimap<float, int> map_indices2;
	multimap<float, int>::iterator it;
	multimap<float, int>::reverse_iterator rit;

	for (int i = 0; i < P_poly.border->size(); ++i)
	{
		p = P_poly.border->points[i];
		if (p.y >= center.y)
		{
			map_indices1.insert(pair<float, int>(p.x, i));
		}
		else
		{
			map_indices2.insert(pair<float, int>(p.x, i));
		}
	}
	for (it = map_indices1.begin(); it != map_indices1.end(); ++it)
	{
		cloud->points.push_back(P_poly.border->points[it->second]);
	}
	for (rit = map_indices2.rbegin(); rit != map_indices2.rend(); ++rit)
	{
		cloud->points.push_back(P_poly.border->points[rit->second]);
	}
	cloud->height = 1;
	cloud->width = cloud->points.size();
	cloud->is_dense = false;
	cloud->reserve(cloud->height * cloud->width);
	P_poly.border->clear();
	P_poly.border = cloud;
}

bool PublicArea()																				  //��ȡ��������ι�������;
{
	float x, y;
	pcl::PointXYZ p;
	for (int i = 0; i < S_polyAfterN.border->points.size(); i++)								  //�����αߵĽ���;
	{

		int S_poly_next_idx = (i + 1) % S_polyAfterN.border->points.size();

		for (int j = 0; j < T_polyAfterN.border->points.size(); j++)
		{
			int T_poly_next_idx = (j + 1) % T_polyAfterN.border->points.size();
			if (GetCrossPoint(i, S_poly_next_idx, j, T_poly_next_idx, x, y))
			{
				p.x = x;
				p.y = y;
				p.z = 0;
				P_poly.border->points.push_back(p);
			}
		}
	}

	for (int i = 0; i < S_polyAfterN.border->points.size(); i++)
	{
		p = S_polyAfterN.border->points[i];
		if (IsPointInPolygon(T_polyAfterN.border, p))
		{
			P_poly.border->points.push_back(p);
		}
	}
	for (int i = 0; i < T_polyAfterN.border->points.size(); i++)
	{
		p = T_polyAfterN.border->points[i];
		if (IsPointInPolygon(S_polyAfterN.border, p))
		{
			P_poly.border->points.push_back(p);
		}
	}

	if (P_poly.border->size() <= 0)
		return false;
	ClockwiseSortPoints();																		  //�㼯����;
	return true;
}

float GetPolyArea(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)									  //��ö�������;
{
	int Count;
	float area = 0;

	Count = cloud->points.size();

	for (int i = 0; i < Count; i++)
	{
		area = area + (cloud->points[i].x * cloud->points[(i + 1) % Count].y - cloud->points[(i + 1) % Count].x * cloud->points[i].y);
	}
	return fabs(area / 2);
}

void Match()
{

	float sum = 0;
	float ratio;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	P_poly.border = cloud;
	float S_Area = 0;
	float T_Area = 0;
	float P_Area = 0;
	for (int i = 0; i < 4; i++)
	{
		T_polyAfterN = PolyAfterN[i];
		P_poly.border->clear();
		if (PublicArea())
		{
			S_Area = GetPolyArea(S_polyAfterN.border);
			T_Area = GetPolyArea(T_polyAfterN.border);
			P_Area = GetPolyArea(P_poly.border);

			if (S_Area >= T_Area)
			{
				ratio = P_Area / S_Area;
			}
			else
				ratio = P_Area / T_Area;

			if (sum <= ratio)
			{
				sum = ratio;
				p_index = i;
			}
		}

	}
	if (sum >= sigma)
	{
		cout << "match success! " << endl;
		cout << "public area ratio is " << sum << endl;
		cout << "source_index = " << source_index << endl;
		cout << "target_index = " << target_index << endl;
		cout << "p_index = " << p_index << endl;
		cout << endl;
		S_final_index.push_back(source_index);
		T_final_index.push_back(target_index);
		IsMatch = true;
		return;
	}
	else
	{
		cout << "public area ratio is " << sum << " match fail...." << endl;
		cout << "match continue...\n" << endl;
	}
}

/***************************************************************************************************************************************/
/*����任����;*/
void registration()
{

	pcl::PointXYZ s_center;
	pcl::PointXYZ t_center;

	pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);

	cout << "start to get transformation_matrix " << endl;
	float s_size, t_size;
	for (int k = 0; k < S_final_index.size(); ++k)
	{
		float x1 = 0, y1 = 0;
		float x2 = 0, y2 = 0;
		float z1 = 0, z2 = 0;
		s_size = source_polygon[S_final_index[k]].border->size();
		t_size = target_polygon[T_final_index[k]].border->size();
		for (int i = 0; i < s_size; i++)                                                          //����Դ��������ĵ�;
		{
			x1 += source_polygon[S_final_index[k]].border->points[i].x;
			y1 += source_polygon[S_final_index[k]].border->points[i].y;
			z1 += source_polygon[S_final_index[k]].border->points[i].z;
		}
		s_center.x = x1 / s_size;
		s_center.y = y1 / s_size;
		s_center.z = z1 / s_size;

		S_center->points.push_back(s_center);

		for (int j = 0; j < t_size; j++)                                                          //����Ŀ���������ĵ�;
		{
			x2 += target_polygon[T_final_index[k]].border->points[j].x;
			y2 += target_polygon[T_final_index[k]].border->points[j].y;
			z2 += target_polygon[T_final_index[k]].border->points[j].z;
		}
		t_center.x = x2 / t_size;
		t_center.y = y2 / t_size;
		t_center.z = z2 / t_size;
		T_center->points.push_back(t_center);
	}

	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>SVD;
	SVD.estimateRigidTransformation(*S_center, *T_center, transformation_matrix_plane_registration);

	pcl::transformPointCloud(*source_cloud_border_plane_registration, *output, transformation_matrix_plane_registration);
	cout << "transformation_matrix : " << endl;
	cout << transformation_matrix_plane_registration << endl;

	viewer_cloud.removeAllPointClouds();
	viewer_cloud.addPointCloud(output, "source");
	viewer_cloud.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0, 0, "source");
	viewer_cloud.addPointCloud(target_cloud_border_plane_registration, "target");
	viewer_cloud.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0, "target");
}
/***************************************************************************************************************************************/
/*
change by czh
ͨ���������ת��������ʾƽ����׼������Ч��
*/
void registration_cloud()
{
	//����ת����
	verb_transform(target_cloud_registration, target_cloud_plane_registration);
	verb_transform(cloud_result, source_cloud_plane_registration);

	pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);

	cout << "Now,let's begin to registration! please wait a moment..." << endl;
	pcl::transformPointCloud(*source_cloud_plane_registration, *output, transformation_matrix_plane_registration);
	*source_cloud_plane_registration = *output;

	viewer_cloud.removeAllPointClouds();
	viewer_cloud.addPointCloud(source_cloud_plane_registration, "source");
	viewer_cloud.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0, 0, "source");
	viewer_cloud.addPointCloud(target_cloud_plane_registration, "target");
	viewer_cloud.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1.0, 0, "target");

	//����ת����
	verb_transform(output, cloud_result);
	verb_transform(target_cloud_plane_registration, target_cloud_registration);
}
/***************************************************************************************************************************************/

#endif // !REGISTRATION_H
