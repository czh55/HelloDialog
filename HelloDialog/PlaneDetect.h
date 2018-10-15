#pragma once;

#ifndef PLANEDETECT_H
#define PLANEDETECT_H

#include "HeaderFile.h"
#include <QString>
#include <pcl/features/normal_3d_omp.h>

// ��Щ����ֻ���ڳ�������ʱ������Ч�ģ�Ӧ������һ����һ�����еı�־����
bool isFirstRun = true;
bool isFirstPostProcess = true;

// parameter space
// �����ռ����
pcl::PointCloud<pcl::PointXYZRGB>::Ptr ps_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree_ps;
// �����ռ�������ʾ
PSElem *param_space = NULL;

// for timing use
clock_t start;
clock_t finish;

int selected_point_index = -1;

// parameters from config.txt
// �����ļ���
const int FileNameSize = 256;
char pcdFileName[FileNameSize];
// �Ƿ�Ե���ִ��ƽ�Ʋ���
bool isConductTranslate = false;
// ȥ��ͬһ��λ�õ������ʱʹ�ã���Ҫ��������֮�����С����
float min_dist_between_points = 0;
// ���㷨�����������뾶
float r_for_estimate_normal = 0;
// ���Ʒ���������ʾ���
int interval_level = 0;
// ���Ʒ���������ʾ����
float normal_length_for_display = 0;
// ѡ�е�ķ������ĳ���
float normal_length_for_selected = 0;
// �������Ƿ�ָ�������ⲿ
bool is_norm_direction_valid = 0;
// У������������������뾶
float r_for_regulate_normal = 0;
// �����ռ���ɢ��Ԫ�ı߳�����ʵ������ֵ�ֱ���
float num_res = 0;
// ���ӻ�ͶƱ�������ɫ����
float color_gain = 0;
// ��˹�˺����ı�׼��
float std_dev_gaussian = 0;
// �����ݶȳ�ʱ�����������뾶
float search_radius_create_gradient = 0;
// ��sink�����ͶƱ����С������ֵ
int T_vote_num = 0;
// �ݶȳ�sink��У��ʱ��ʼ�������뾶
float radius_base = 0;
// radius_base������
float delta_radius = 0;
// ��Ч��Ԫռ�ȵ���ֵ
float S_threshold = 0;
// У���ݶȳ�ʱ�ĽǶ�ƫ����ֵ(�Ƕȣ�
float dev_threshold = 0;
// ����ƽ��ָ�ʱ�����뾶
float search_radius_for_plane_seg = 0;
// ����ƽ���������ֵ
int T_num_of_single_plane = 0;
// ʵ��ƽ����������ʱ��Ѱ�ұ߽�İ뾶
float radius_border = 0;
// ʵ��ƽ����������ʱ��Ѱ�Ҿֲ�ƽ��İ뾶
float radius_local = 0;
// ���º�ƽ��ĽǶ�ƫ����ֵ���Ƕȣ�
float T_angle_bias_integrate = 0;
// ����ƽ������ʱ�����ķ�����ʱѡȡ��һ����С�������뾶
float r_local = 0;
// ����ƫ����ֵ
float T_curvature_bias = 0;
// ƽ���Ե��ķ�������ƽ�淨������ƫ����ֵ
float T_point_normal_bias = 0;
// ����ƽ��ϲ�ʱ�������ֵ���С����
float T_ratio_merge_planes = 0;
// ����ƽ�氼��ʱalpha��С
float alpha_poly = 0;
// ����ƽ��ľ�����ֵ
float T_dist_point_plane = 0;
// ÿ���������С������ֵ
int T_cluster_num = 0;
///////////////////////////////////////////////////////////////////////////////
// glabal variables
char g_argu_buffer[ARGU_BUFFER_SIZE];
const float PI = 3.141592653f;
const float e = 2.7182818f;

std::vector<SinkPoint> sinkPointsSet;	// store sink points

// �ָ������ƽ�漯��
std::vector<Plane, Eigen::aligned_allocator<Plane>> plane_clouds;
std::vector<Plane, Eigen::aligned_allocator<Plane>> growth_unit_set;


PointCloudT::Ptr source_cloud (new PointCloudT);
pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_source;
bool* isProcessed = NULL;
pcl::PointCloud<pcl::Normal>::Ptr source_normal (new pcl::PointCloud<pcl::Normal>);

PointCloudT::Ptr source_cloud_backup(new PointCloudT);
pcl::PointCloud<pcl::Normal>::Ptr source_normal_backup(new pcl::PointCloud<pcl::Normal>);
bool isFirstLoop = true;

// ������
const int CMD_SIZE = 64;
char cmd[CMD_SIZE];
char state[CMD_SIZE];

DWORD dwThread;

///////////////////////////////////////////////////////////////////////////////

DWORD WINAPI cmdFunc(LPVOID);
// �������ļ��м��ظ������
void loadParam();
// ���ص��Ʋ���������
void loadPointCloud();
// ���ص��ƣ����з�����
void loadPointNormal();
// ����Ԥ����
void preProcess();
// ���Ƶ��Ʒ�����
void estimateNormal();
// У�����Ʒ�����
bool isFirstUseRegulateNormal = true;
void regulateNormal();
// ���������ռ�
void createPS();
// �ж�С�������Ƿ��������ཻ
bool isCubeIntersectWithUnitSphere( float xmin, float ymin, float zmin, float length);
// ������ռ����ͶƱ
void voteForPS();
// �Բ����ռ���ж�ά�ĸ�˹�˲�
void filtPS();
// ��˹�˺���
float gaussianKernal(float x, float std_dev);
// �Բ����ռ乹���ݶȳ�
void createGradientField();
// �ڲ����ռ���Ѱ�ҵ�ǰ�������voteֵ�ĵ������
void findMaxVoteInPS(PSElem *&param_space, int ps_size, int &index);
// У�������ռ���ݶȳ�
void rectifySinkFields();
// �ָ�ƽ�����
void segmentPlanes();
// ��ȡ�ָ�ƽ������ӵ�
int getSeedIndex(bool* &isProcessed, int size);
// ���÷ָ�õ���������Ԫ����ƽ�����
void growPlaneArea();
float arccos(float val)
{
	if(val > 1.0f) val = 1.0f;
	if(val < -1.0f) val = -1.0f;
	return acos(val);
}
// ƽ��ϲ�
void mergePlanes();
// ��ȡ����ƽ�湫���������
int getCommonPointsNumber(PointCloudT::Ptr &cloud1, PointCloudT::Ptr &cloud2);
// ��Ŀ����ƺϲ���Դ����
void mergeClouds(PointCloudT::Ptr &source, PointCloudT::Ptr &target);
// ����λ�ƽ�����
void polyPlanes();
void polyPointCloud( PointCloudT::Ptr &cloud,  
					 PointCloudT::Ptr &border,
					 Eigen::Vector3f &pn);
// ����ͶӰ��ƽ����
void projPoint2Plane( pcl::PointXYZ &source_point, pcl::PointXYZ &dest_point, Eigen::Vector4f &plane_param );
// ƽ�����
void postProcessPlanes();
// ��Դ���ƽ��о���ָ������Ԫ���������ٵľ���
void clusterFilt();
// ���ǻ�ƽ��
void trianglatePlane(Plane &plane);
// ĳ���Ƿ���ƽ����
bool isPointInPlane(pcl::PointXYZ &p, Plane &plane);
bool isPointInPlane_complement(pcl::PointXYZ &p, Plane &plane);
bool isPointInPoly(pcl::PointXYZ &p, Plane &plane);
bool isBothLineSegsIntersect(pcl::PointXYZ &pa, pcl::PointXYZ &pb, pcl::PointXYZ &pc, pcl::PointXYZ &pd, pcl::PointXYZ &p_inter);
// ��ȡ�㵽ƽ��ľ�����ͶӰ
void getInfoBetPointAndPlane(pcl::PointXYZ &p, Eigen::Vector4f &plane_param, float &dist, pcl::PointXYZ &p_proj);
// �㵽��ľ��뺯��
inline float distP2P(pcl::PointXYZ &p1, pcl::PointXYZ &p2)
{
	float a = (p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z);
	return pow(a, 0.5f);
}
// �㵽ֱ�ߵľ��뺯��
void distP2L(pcl::PointXYZ &p, pcl::PointXYZ &p_base, Eigen::Vector3f &n, pcl::PointXYZ &p_projOnLine, float &dist);

// ��ѡȡ�¼��ص�����
void pointPickingEventOccurred ( const pcl::visualization::PointPickingEvent &event,
								 void* viewer_void);
void areaPickingEventOccurred ( const pcl::visualization::AreaPickingEvent &event,
								 void* viewer_void);
// �����¼�������
void keyboardEventOccurred_ps (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void);
// �����¼�������
void keyboardEventOccurred_cloud (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void);
// ��Ϣ������
void processStateMsg();

HANDLE display_thread;

/*void main()
{
	srand(time(0));
	//HANDLE display_thread;
	//DWORD dwThread;
	display_thread = ::CreateThread(NULL, 0, cmdFunc, (LPVOID)cmdFunc, 0, &dwThread);

	viewer_cloud.addCoordinateSystem(1.0f);
	viewer_ps.addCoordinateSystem(1.0f);

	viewer_cloud.setWindowName("point cloud");
	viewer_ps.setWindowName("param space");

	viewer_cloud.registerPointPickingCallback(pointPickingEventOccurred, (void*)&viewer_cloud);
	viewer_cloud.registerKeyboardCallback(keyboardEventOccurred_cloud, (void*)&viewer_cloud);
	viewer_cloud.registerAreaPickingCallback(areaPickingEventOccurred, (void*)&viewer_cloud);
	viewer_ps.registerKeyboardCallback(keyboardEventOccurred_ps, (void*)&viewer_ps);

	while(!viewer_ps.wasStopped())
	{
		viewer_ps.spinOnce(500);
		viewer_cloud.spinOnce(500);
		processStateMsg();
	}
}*/

// ������
// ע�ⲻҪ���̺߳����д�����ʾ����
DWORD WINAPI cmdFunc(LPVOID)
{
	while(TRUE)
	{
		memset(cmd, 0, sizeof(char)*CMD_SIZE);
		cout << ">";
		
		int i = 0;
		do{
			char c;
			c = getchar();
			if(c == '\n')
				break;
			cmd[i] = c;
			++i;
		}while(true);

		if(strcmp(cmd, "load param") == 0)
		{
			loadParam();
			continue;
		}

		if(strcmp(cmd, "load point cloud") == 0)
		{
			loadPointCloud();
			continue;
		}

		if(strcmp(cmd, "load point normal") == 0)
		{
			loadPointNormal();
			cout << "loaded " << source_cloud->size() << " points." << endl;
			continue;
		}
		
		if(strcmp(cmd, "clear normal") == 0)
		{
			memset(state, 0, CMD_SIZE);
			strcpy(state, cmd);
			continue;
		}

		if(strcmp(cmd, "estimate normal") == 0)
		{
			memset(state, 0, CMD_SIZE);
			strcpy(state, cmd);
			estimateNormal();
			// ::SuspendThread(display_thread);
			continue;
		}

		if(strcmp(cmd, "save normal") == 0)
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
			std::stringstream ss;
			ss << std::clock() << ".pcd";
			pcl::io::savePCDFile(ss.str(), *cloud);
			cout << "file:" << ss.str() << " has been saved." << endl;
			continue;
		}

		if(strcmp(cmd, "regulate normal") == 0)
		{
			memset(state, 0, CMD_SIZE);
			strcpy(state, cmd);
			if(isFirstUseRegulateNormal)
			{
				isFirstUseRegulateNormal = false;
				if(isProcessed == NULL)
				{
					isProcessed = new bool[source_cloud->size()];
					memset(isProcessed, 0, sizeof(bool) * source_cloud->size());
				}else{
					memset(isProcessed, 0, sizeof(bool) * source_cloud->size());
				}
			}
			//regulateNormal();
			memset(state, 0, CMD_SIZE);
			strcpy(state, cmd);
			continue;
		}

		if(strcmp(cmd, "create ps") == 0)
		{
			start = std::clock();
			createPS();
			voteForPS();
			finish = std::clock();
			cout << "parameter space has been constructed, ps has " << ps_cloud->size() << " elements." << endl;
			cout << "time used: " << finish - start << "ms." << endl;

			memset(state, 0, CMD_SIZE);
			strcpy(state, cmd);
			continue;
		}

		if(strcmp(cmd, "filt ps") == 0)
		{
			start = std::clock();
			filtPS();
			finish = std::clock();
			cout << "filt used: " << finish-start << "ms." << endl;
			memset(state, 0, CMD_SIZE);
			strcpy(state, cmd);
			continue;
		}

		if(strcmp(cmd, "segment ps") == 0)
		{
			start = std::clock();
			createGradientField();
			rectifySinkFields();
			finish = std::clock();
			cout << "segment ps used " << finish-start << "ms." << endl;
			memset(state, 0, CMD_SIZE);
			strcpy(state, cmd);
			continue;
		}

		if(strcmp(cmd, "segment planes") == 0)
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
			strcpy(state, cmd);
			continue;
		}

		if(strcmp(cmd, "grow planes") == 0)
		{
			start = std::clock();
			growPlaneArea();
			finish = std::clock();
			cout << "plane growing used " << finish-start << "ms." << endl;
			memset(state, 0, CMD_SIZE);
			strcpy(state, cmd);
			continue;
		}

		if(strcmp(cmd, "merge planes") == 0)
		{
			start = std::clock();
			mergePlanes();
			finish = std::clock();
			cout << "plane merging used " << finish-start << "ms." << endl;
			memset(state, 0, CMD_SIZE);
			strcpy(state, cmd);
			continue;
		}

		if(strcmp(cmd, "poly planes") == 0)
		{
			start = std::clock();			
			polyPlanes();
			finish = std::clock();
			cout << "plane polygonization used " << finish-start << "ms." << endl;
			memset(state, 0, CMD_SIZE);
			strcpy(state, cmd);
			continue;
		}

		// ִ�к����Ŀ����Ϊ�˽�ƽ���ڲ��Ļ�δ��ǵĵ������ƽ��
		if(strcmp(cmd, "postProcess planes") == 0)
		{
			start = std::clock();
			postProcessPlanes();
			finish = std::clock();
			cout << "plane post process used " << finish-start << "ms." << endl;
			memset(state, 0, CMD_SIZE);
			strcpy(state, cmd);
			continue;
		}

		if(strcmp(cmd, "run again") == 0)
		{
			// ������
			loadParam();

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
			continue;
		}

		if(strcmp(cmd, "do") == 0)
		{	
			// Ԥ����
			start = std::clock();
			loadParam();
			loadPointNormal();
			finish = std::clock();
			cout << "preProcess: " << finish-start << "ms." << endl;
			if(isFirstRun)
			{				
				cout << "loaded " << source_cloud->size() << " points." << endl;
			}
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
			continue;
		}

		if(strcmp(cmd, "save polys") == 0)
		{
			/*PointCloudT::Ptr poly_cloud (new PointCloudT);
			int count = 0;
			for(int ii = 0; ii < plane_clouds_final.size(); ++ii)
			{
				for(int i = 0; i < plane_clouds_final[ii].border->size(); ++i)
				{
					poly_cloud->push_back(plane_clouds_final[ii].border->points[i]);
					++count;
				}
				poly_cloud->push_back(plane_clouds_final[ii].border->points[0]);
				++count;
			}
			pcl::io::savePCDFileASCII("poly.pcd", *poly_cloud);			
			cout << "count = " << count << endl;
			cout << "plane number = " << plane_clouds_final.size() << endl;

			pcl::PolygonMesh pm;*/
			/*pcl::PolygonMesh polygons;
			PointCloudT vertices;
			for(int ii = 0; ii < plane_clouds_final.size(); ++ii)
			{
				Plane plane = plane_clouds_final[ii];
				for(int i = 0; i < plane.border->size(); ++i)
				{
					vertices.push_back(plane.border->points[i]);
				}
			}
			pcl::toPCLPointCloud2(vertices, polygons.cloud);
			int base_index = 0;
			for(int ii = 0; ii < plane_clouds_final.size(); ++ii)
			{
				pcl::Vertices polygon;
				int num_of_vertices = plane_clouds_final[ii].border->size();
				for(int i = base_index; i < base_index+num_of_vertices; ++i)
				{
					polygon.vertices.push_back(i);
				}
				polygon.vertices.push_back(base_index);
				polygons.polygons.push_back(polygon);
				base_index += num_of_vertices;
			}*/

			PointCloudT::Ptr vertices (new PointCloudT);
			for(int ii = 0; ii < plane_clouds_final.size(); ++ii)
			{
				for(int i = 0; i < plane_clouds_final[ii].border->size(); ++i)
				{
					vertices->push_back(plane_clouds_final[ii].border->points[i]);
				}
			}
			pcl::io::savePCDFileASCII("poly_vertices.pcd", *vertices);

			std::ofstream file("poly_indices.txt", std::ios::out);

			for(int ii = 0; ii < plane_clouds_final.size(); ++ii)
			{
				file << plane_clouds_final[ii].border->size() << endl;
			}

			continue;
		}

		if(strcmp(cmd, "load polys") == 0)
		{
			PointCloudT::Ptr vertices (new PointCloudT);
			pcl::io::loadPCDFile("poly_vertices.pcd", *vertices);
			std::ifstream file("poly_indices.txt", std::ios::out);
			int num_of_polys = 0;
			std::vector<int> poly_size;
			while(!file.eof())
			{
				++num_of_polys;
				int count;
				file >> count;
				poly_size.push_back(count);
			}
			--num_of_polys;
			cout << "num_of_polys = " << num_of_polys << endl;

			for(int ii = 0; ii < plane_clouds_final.size(); ++ii)
			{
				plane_clouds_final[ii].border->clear();
				plane_clouds_final[ii].coeff.values.clear();
				plane_clouds_final[ii].points_set->clear();
				plane_clouds_final[ii].triangles.clear();
			}
			plane_clouds_final.clear();

			int base_index = 0;
			for(int ii = 0; ii < num_of_polys; ++ii)
			{
				Plane plane;
				PointCloudT::Ptr border (new PointCloudT);
				for(int i = base_index; i < base_index+poly_size[ii]; ++i)
				{
					border->push_back(vertices->points[i]);
				}
				base_index += poly_size[ii];
				plane.border = border;
				plane_clouds_final.push_back(plane);
			}
			cout << "load polys done." << endl;
			/*for(int ii = 0; ii < plane_clouds_final.size(); ++ii)
			{
				plane_clouds_final[ii].border->clear();
				plane_clouds_final[ii].coeff.values.clear();
				plane_clouds_final[ii].points_set->clear();
				plane_clouds_final[ii].triangles.clear();
			}
			plane_clouds_final.clear();

			PointCloudT::Ptr poly_cloud (new PointCloudT);
			pcl::io::loadPCDFile("poly.pcd", *poly_cloud);
			viewer_cloud.removeAllPointClouds();
			viewer_cloud.addPointCloud(poly_cloud,"poly");*/
			//bool isStartNewPoly = true;
			//pcl::PointXYZ p_start, p_cur;

			//// Plane plane;
			//int plane_index = 0;
			//for(int i = 0; i < poly_cloud->size(); ++i)
			//{
			//	if(isStartNewPoly)
			//	{
			//		isStartNewPoly = false;
			//		p_start = poly_cloud->points[i];

			//		Plane plane;
			//		plane_clouds_final.push_back(plane);

			//		PointCloudT::Ptr border (new PointCloudT);
			//		PointCloudT::Ptr points (new PointCloudT);
			//		plane_clouds_final[plane_index].border = border;
			//		plane_clouds_final[plane_index].points_set = points;
			//		plane_clouds_final[plane_index].coeff.values.clear();
			//		plane_clouds_final[plane_index].triangles.clear();
			//		plane_clouds_final[plane_index].border->push_back(p_start);
			//		continue;
			//	}
			//	p_cur = poly_cloud->points[i];
			//	if( p_cur.x == p_start.x && 
			//		p_cur.y == p_start.y && 
			//		p_cur.z == p_start.z)
			//	{
			//		//plane_clouds_final.push_back(plane);
			//		++plane_index;
			//		isStartNewPoly = true;
			//	}else{
			//		plane_clouds_final[plane_index].border->push_back(p_cur);
			//	}
			//}
			//cout << "plane number = " << plane_clouds_final.size() << endl;
			//memset(state, 0, CMD_SIZE);
			//strcpy(state, "load polys");
			continue;
		}

		if(strcmp(cmd, "test search_radius") == 0)
		{
			memset(state, 0, CMD_SIZE);
			strcpy(state, cmd);
			continue;
		}

		if(strcmp(cmd, "save cloud") == 0)
		{
			pcl::io::savePCDFileASCII("point_cloud", *source_cloud);
			cout << "point cloud has been saved..." << endl;
			continue;
		}
		
		cout << cmd << ": invalid cmd" << endl;
	}
	return 0;
}
// �������ļ��м��ظ������
void loadParam()
{
	// �ļ���
	memset(pcdFileName,0,sizeof(char)*FileNameSize);
	getParameterFromConfigFile("pcdFileName");
	std::strcpy(pcdFileName, g_argu_buffer);
	cout << "pcdFileName = " << pcdFileName << endl;

	// �Ƿ�Ե���ִ��ƽ�Ʋ���
	getParameterFromConfigFile("isConductTranslate");
	isConductTranslate = atoi(g_argu_buffer);
	cout << "isConductTranslate = " << isConductTranslate << endl;

	// ȥ��ͬһ��λ�õ������ʱʹ�ã�Ҳ��������֮�����С����
	getParameterFromConfigFile("min_dist_between_points");
	min_dist_between_points = atof(g_argu_buffer);
	cout << "min_dist_between_points = " << min_dist_between_points << endl;

	// ���㷨�����������뾶
	getParameterFromConfigFile("r_for_estimate_normal");
	r_for_estimate_normal = atof(g_argu_buffer);
	cout << "r_for_estimate_normal = " << r_for_estimate_normal << endl;

	// ���Ʒ���������ʾ���
	getParameterFromConfigFile("interval_level");
	interval_level = atoi(g_argu_buffer);
	cout << "interval_level = " << interval_level << endl;

	// ���Ʒ���������ʾ����
	getParameterFromConfigFile("normal_length_for_display");
	normal_length_for_display = atof(g_argu_buffer);
	cout << "normal_length_for_display = " << normal_length_for_display << endl;

	// ѡ�е�ķ������ĳ���
	getParameterFromConfigFile("normal_length_for_selected");
	normal_length_for_selected = atof(g_argu_buffer);
	cout << "normal_length_for_selected = " << normal_length_for_selected << endl;

	// У������������������뾶
	getParameterFromConfigFile("r_for_regulate_normal");
	r_for_regulate_normal = atof(g_argu_buffer);
	cout << "r_for_regulate_normal = " << r_for_regulate_normal << endl;

	// �����ռ���ֵ�ֱ���
	getParameterFromConfigFile("num_res");
	num_res = atof(g_argu_buffer);
	cout << "num_res = " << num_res << endl;

	// ���ӻ�ͶƱ�������ɫ����
	getParameterFromConfigFile("color_gain");
	color_gain = atof(g_argu_buffer);
	cout << "color_gain = " << color_gain << endl;

	// ��˹�˺����ı�׼��
	getParameterFromConfigFile("std_dev_gaussian");
	std_dev_gaussian = atof(g_argu_buffer);
	cout << "std_dev_gaussian = " << std_dev_gaussian << endl;

	// �����ݶȳ�ʱ�����������뾶
	getParameterFromConfigFile("search_radius_create_gradient");
	search_radius_create_gradient = atof(g_argu_buffer);
	cout << "search_radius_create_gradient = " << search_radius_create_gradient << endl;

	// ��sink�����ͶƱ����С������ֵ
	getParameterFromConfigFile("T_vote_num");
	T_vote_num = atoi(g_argu_buffer);
	cout << "T_vote_num = " << T_vote_num << endl;

	// �ݶȳ�sink��У��ʱ��ʼ�������뾶
	getParameterFromConfigFile("radius_base");
	radius_base = atof(g_argu_buffer);
	cout << "radius_base = " << radius_base << endl;

	// radius_base������
	getParameterFromConfigFile("delta_radius");
	delta_radius = atof(g_argu_buffer);
	cout << "delta_radius = " << delta_radius << endl;

	// ��Ч��Ԫռ�ȵ���ֵ
	getParameterFromConfigFile("S_threshold");
	S_threshold = atof(g_argu_buffer);
	cout << "S_threshold = " << S_threshold << endl;

	// У���ݶȳ�ʱ�ĽǶ�ƫ����ֵ(�Ƕȣ�
	getParameterFromConfigFile("dev_threshold");
	dev_threshold = atof(g_argu_buffer);
	cout << "dev_threshold = " << dev_threshold << endl;

	// ����ƽ��ָ�ʱ�����뾶
	getParameterFromConfigFile("search_radius_for_plane_seg");
	search_radius_for_plane_seg = atof(g_argu_buffer);
	cout << "search_radius_for_plane_seg = " << search_radius_for_plane_seg << endl;

	// ����ƽ���������ֵ
	getParameterFromConfigFile("T_num_of_single_plane");
	T_num_of_single_plane = atoi(g_argu_buffer);
	cout << "T_num_of_single_plane = " << T_num_of_single_plane << endl;

	// ʵ��ƽ����������ʱ��Ѱ�ұ߽�İ뾶
	getParameterFromConfigFile("radius_border");
	radius_border = atof(g_argu_buffer);
	cout << "radius_border = " << radius_border << endl;

	// ʵ��ƽ����������ʱ��Ѱ�Ҿֲ�ƽ��İ뾶
	getParameterFromConfigFile("radius_local");
	radius_local = atof(g_argu_buffer);
	cout << "radius_local = " << radius_local << endl;

	// ���º�ƽ��ĽǶ�ƫ����ֵ���Ƕȣ�
	getParameterFromConfigFile("T_angle_bias_integrate");
	T_angle_bias_integrate = atof(g_argu_buffer);
	cout << "T_angle_bias_integrate = " << T_angle_bias_integrate << endl;

	// ����ƽ������ʱ�����ķ�����ʱѡȡ��һ����С�������뾶
	getParameterFromConfigFile("r_local");
	r_local = atof(g_argu_buffer);
	cout << "r_local = " << r_local << endl;

	// ����ƫ����ֵ
	getParameterFromConfigFile("T_curvature_bias");
	T_curvature_bias = atof(g_argu_buffer);
	cout << "T_curvature_bias = " << T_curvature_bias << endl;

	// ƽ���Ե��ķ�������ƽ�淨������ƫ����ֵ
	getParameterFromConfigFile("T_point_normal_bias");
	T_point_normal_bias = atof(g_argu_buffer);
	cout << "T_point_normal_bias = " << T_point_normal_bias << endl;

	// ����ƽ��ϲ�ʱ�������ֵ���С����
	getParameterFromConfigFile("T_ratio_merge_planes");
	T_ratio_merge_planes = atof(g_argu_buffer);
	cout << "T_ratio_merge_planes = " << T_ratio_merge_planes << endl;

	// ����ƽ�氼��ʱalpha��С
	getParameterFromConfigFile("alpha_poly");
	alpha_poly = atof(g_argu_buffer);
	cout << "alpha_poly = " << alpha_poly << endl;

	// ����ƽ��ľ�����ֵ
	getParameterFromConfigFile("T_dist_point_plane");
	T_dist_point_plane = atof(g_argu_buffer);
	cout << "T_dist_point_plane = " << T_dist_point_plane << endl;

	// ÿ���������С������ֵ
	getParameterFromConfigFile("T_cluster_num");
	T_cluster_num = atoi(g_argu_buffer);
	cout << "T_cluster_num = " << T_cluster_num << endl;
}
// ���ص��ƣ�����������
void loadPointCloud()
{
	source_cloud->clear();
	source_normal->clear();

	int r = pcl::io::loadPCDFile<pcl::PointXYZ>(pcdFileName, *source_cloud);
	if(r == -1)
	{
		cout << "pcd file loaded failed... " << endl;
		return;
	}

	preProcess();
	viewer_cloud.removeAllPointClouds();
	viewer_cloud.addPointCloud(source_cloud,"source");
}
// ���ص��ƣ����з�����
void loadPointNormal()
{
	if(isFirstRun)
	{
		source_cloud->clear();
		source_normal->clear();
		pcl::PointCloud<pcl::PointNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointNormal>);

		int r = pcl::io::loadPCDFile<pcl::PointNormal>(pcdFileName, *cloud);
		if(r == -1)
		{
			cout << "pcd file loaded failed... " << endl;
			return;
		}
	
		pcl::PointXYZ p;
		pcl::Normal n;
        source_cloud->reserve(cloud->size());
        source_normal->reserve(cloud->size());
		for(int i = 0; i < cloud->size(); ++i)
		{
			p.x = cloud->points[i].x;
			p.y = cloud->points[i].y;
			p.z = cloud->points[i].z;
			n.normal_x = cloud->points[i].normal_x;
			n.normal_y = cloud->points[i].normal_y;
			n.normal_z = cloud->points[i].normal_z;
			source_cloud->push_back(p);
			source_normal->push_back(n);
		}
		viewer_cloud.removeAllPointClouds();
		viewer_cloud.addPointCloud(source_cloud,"source");
		viewer_cloud.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(
			source_cloud, source_normal, 
			interval_level, normal_length_for_display, "normal");

		kdtree_source.setInputCloud(source_cloud);
	}else{
		if(source_normal->size()>0) source_normal->clear();
		pcl::PointCloud<pcl::PointNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointNormal>);

		int r = pcl::io::loadPCDFile<pcl::PointNormal>(pcdFileName, *cloud);
		if(r == -1)
		{
			cout << "pcd file loaded failed... " << endl;
			return;
		}
		pcl::KdTreeFLANN<pcl::PointNormal> kdtree;
		kdtree.setInputCloud(cloud);
		std::vector<int> indices;
		std::vector<float> dist;
		pcl::PointNormal p;
		pcl::Normal n;
		for(int i = 0; i < source_cloud->size(); ++i)
		{
			p.x = source_cloud->points[i].x;
			p.y = source_cloud->points[i].y;
			p.z = source_cloud->points[i].z;
			kdtree.nearestKSearch(p, 1, indices, dist);
			n.normal_x = cloud->points[indices[0]].normal_x;
			n.normal_y = cloud->points[indices[0]].normal_y;
			n.normal_z = cloud->points[indices[0]].normal_z;
			source_normal->push_back(n);
		}
	}
}
// ����Ԥ����[ȥ����Ч�㡢ƽ�Ƶ��������ģ�ȥ�������]
void preProcess()
{
	// ȥ��nan��Ч��
	PointCloudT::Ptr cloud_out (new PointCloudT);
	std::vector<int> index;
	pcl::removeNaNFromPointCloud(*source_cloud, *cloud_out, index);
	source_cloud = cloud_out;
	cout << "after filt nan points, cloud points number = " << source_cloud->size() << endl;

	// ƽ�Ƶ��Ƶ�����
	if(isConductTranslate)
	{
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

		for(int i = 0; i < source_cloud->size(); ++i)
		{
			source_cloud->points[i].x -= p.x;
			source_cloud->points[i].y -= p.y;
			source_cloud->points[i].z -= p.z;
		}
		cout << "cloud has been translated..." << endl;
		cout << "translation vector = " << p << endl;
	}

	// ȥ�������
	if(isProcessed != NULL) delete []isProcessed;
	isProcessed = new bool[source_cloud->size()];
	memset(isProcessed, 0, sizeof(bool)*source_cloud->size());

	kdtree_source.setInputCloud(source_cloud);
	float radius = min_dist_between_points;
	std::vector<int> indices;
	std::vector<float> dist;
	PointCloudT::Ptr cloud_processed (new PointCloudT);
	for(int i = 0; i < source_cloud->size(); ++i)
	{
		if(isProcessed[i]) continue;

		isProcessed[i] = true;
		cloud_processed->push_back(source_cloud->points[i]);

		kdtree_source.radiusSearch(source_cloud->points[i], radius, indices, dist);

		for(int j = 1; j < indices.size(); ++j)
		{
			isProcessed[indices[j]] = true;
		}
	}
	delete []isProcessed;
	isProcessed = NULL;
	source_cloud = cloud_processed;
	// ��������kdtree
	kdtree_source.setInputCloud(source_cloud);
	cout << "after remove redundancy points, cloud points number = " << source_cloud->size() << endl;
}
// ���Ƶ��Ʒ�����
void estimateNormal()
{
	if(source_cloud->size() == 0)	return;
	start = std::clock();
    // ���У�
    /*pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(source_cloud);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree
		 (new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);
    ne.setRadiusSearch(r_for_estimate_normal); cout << "���Ʒ����������뾶��" << r_for_estimate_normal << endl;
    ne.compute(*source_normal);*/

    // ���У�
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(source_cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree
            (new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(r_for_estimate_normal); cout << "���Ʒ����������뾶��" << r_for_estimate_normal << endl;
    ne.compute(*source_normal);

	finish = std::clock();
	cout << "normal estimation used: " << finish-start << "ms." << endl;
   // viewer_cloud.removePointCloud("source");
   // viewer_cloud.addPointCloud(source_cloud, "source");
    //viewer_cloud.removePointCloud("normal");
   // viewer_cloud.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(
        //source_cloud, source_normal,
       // interval_level, normal_length_for_display, "normal");
}
// У�����Ʒ�����
void regulateNormal()
{
    if(isProcessed != NULL) delete []isProcessed;
    isProcessed = new bool[source_cloud->size()];
    memset(isProcessed, 0, sizeof(bool)*source_cloud->size());

	if(!isFirstPostProcess)
	{
		//cout << "isFirstPostProcess=" << isFirstPostProcess <<endl;
		std::vector<int> indices;
		std::vector<float> dist;
		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
		kdtree.setInputCloud(source_cloud_backup);
		for(int i = 0; i < source_cloud->size(); ++i)
		{
			kdtree.nearestKSearch(source_cloud->points[i], 1, indices, dist);
			Eigen::Vector3f v1, v2;
			v1 << source_normal->points[i].normal_x,
				  source_normal->points[i].normal_y,
				  source_normal->points[i].normal_z;
			v2 << source_normal_backup->points[indices[0]].normal_x,
				  source_normal_backup->points[indices[0]].normal_y,
				  source_normal_backup->points[indices[0]].normal_z;
			if(v1.dot(v2) < 0)
			{
				source_normal->points[i].normal_x *= -1.0f;
				source_normal->points[i].normal_y *= -1.0f;
				source_normal->points[i].normal_z *= -1.0f;
			}
		}

		// ������ʾ�ķ�����
		viewer_cloud.removePointCloud("normal");
		viewer_cloud.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(
		source_cloud, source_normal, 
		interval_level, normal_length_for_display, "normal");
		return;
	}

	// ���ȴ������ļ��ж�ȡ��ǰ��ѡ�еĵ�ķ������Ƿ�ָ�������ⲿ
    // ���µĲ�������������ļ�����ȡ�����ǴӶԻ���������
//	getParameterFromConfigFile("is_norm_direction_valid");
//	is_norm_direction_valid = atoi(g_argu_buffer);
	cout << "is_norm_direction_valid = " << is_norm_direction_valid << endl;

	if(selected_point_index < 0)
	{
		cout << "invalid point index: " << selected_point_index << endl;
		return;
	}

    cout << "selected_point_index = " << selected_point_index << endl;
    cout << "source_normal->size() = " << source_normal->size() << endl;
	if(!is_norm_direction_valid)
	{
		source_normal->points[selected_point_index].normal_x *= -1.0f;
		source_normal->points[selected_point_index].normal_y *= -1.0f;
		source_normal->points[selected_point_index].normal_z *= -1.0f;
	}
	isProcessed[selected_point_index] = true;	// idicate that the point with the index 'selected_point_index' has been processed

	std::queue<int> Q;
	Q.push(selected_point_index);

	pcl::Normal p_cur;
	pcl::Normal p_neig;

	std::vector<int> indices;
    std::vector<float> dist;
    kdtree_source.setInputCloud(source_cloud);
	while(!Q.empty())
	{
		int cur_index = Q.front();
		Q.pop();

		kdtree_source.radiusSearch(source_cloud->points[cur_index], r_for_regulate_normal, indices, dist);
		for(int i = 0; i < indices.size(); ++i)
		{
			if(isProcessed[indices[i]])	continue;
			p_cur = source_normal->points[cur_index];
			p_neig = source_normal->points[indices[i]];
			// �����������������ڻ�
			float dot_product = p_cur.normal_x*p_neig.normal_x + p_cur.normal_y*p_neig.normal_y + p_cur.normal_z*p_neig.normal_z;
			if(dot_product<0)
			{
				source_normal->points[indices[i]].normal_x *= -1.0f;
				source_normal->points[indices[i]].normal_y *= -1.0f;
				source_normal->points[indices[i]].normal_z *= -1.0f;
			}
			isProcessed[indices[i]] = true;
			Q.push(indices[i]);
        }
	}

	PointCloudT::Ptr processed_cloud (new PointCloudT);
	for(int i = 0; i < source_cloud->size(); ++i)
	{
		if(isProcessed[i])
		{
			processed_cloud->push_back(source_cloud->points[i]);
		}
	}
	cout << "number of points that have been processed is: " << processed_cloud->size() << endl;
	viewer_cloud.removePointCloud("processed");
	viewer_cloud.addPointCloud(processed_cloud, "processed");
	viewer_cloud.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 255, 0, "processed");

	// ������ʾ�ķ�����
	viewer_cloud.removePointCloud("normal");
	viewer_cloud.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(
		source_cloud, source_normal, 
		interval_level, normal_length_for_display, "normal");

	// ��֮ǰѡ��ĵ����ʾ״̬ɾ��
	viewer_cloud.removePointCloud("selected normal");

	// ����selected_point_index
	selected_point_index = -1;
}
// ���������ռ�
void createPS()
{	
	if(ps_cloud->size() > 0) 
	{
			ps_cloud->clear();
			ps_cloud->resize(0);
	}
	int x_size = floor(2.0f/num_res+0.5f);
	int y_size = floor(2.0f/num_res+0.5f);
	int z_size = floor(2.0f/num_res+0.5f);

	pcl::PointXYZRGB p;
	float x,y,z;
	for(int i = 0; i < x_size; ++i)
	{
		for(int j = 0; j < y_size; ++j)
		{
			for(int k = 0; k < z_size; ++k)
			{
				x = (float)i*num_res - 1.0f;
				y = (float)j*num_res - 1.0f;
				z = (float)k*num_res - 1.0f;
				if(isCubeIntersectWithUnitSphere(x,y,z,num_res))
				{
					p.x = x;
					p.y = y;
					p.z = z;
					ps_cloud->push_back(p);
				}
			}
		}
	}
	kdtree_ps.setInputCloud(ps_cloud);
	if(param_space == NULL)
	{
		param_space = new PSElem[ps_cloud->size()];
	}else{
		for(int i = 0; i < ps_cloud->size(); ++i)
		{
			if(param_space[i].point_indices.size()>0)
			{
				param_space[i].point_indices.clear();
			}
		}
		delete []param_space;
		param_space = new PSElem[ps_cloud->size()];
	}
	/*memset(param_space, 0, sizeof(PSElem)*ps_cloud->size());*///vector iterator incompatible error
	for(int i = 0; i < ps_cloud->size(); ++i)	// ��ʼ�������ռ�
	{
		param_space[i].sink = param_space[i].sink_init = -1;
		param_space[i].vote = 0;
	}
}
// �ж�С�������Ƿ��������ཻ
bool isCubeIntersectWithUnitSphere( float xmin, float ymin, float zmin, float length)
{
	std::vector<Eigen::Vector3f> vertices;
	Eigen::Vector3f v;

	v << xmin, ymin, zmin;
	vertices.push_back(v);
	v << xmin+length, ymin, zmin;
	vertices.push_back(v);
	v << xmin, ymin+length, zmin;
	vertices.push_back(v);
	v << xmin, ymin, zmin+length;
	vertices.push_back(v);
	v << xmin+length, ymin+length, zmin;
	vertices.push_back(v);
	v << xmin+length, ymin, zmin+length;
	vertices.push_back(v);
	v << xmin, ymin+length, zmin+length;
	vertices.push_back(v);
	v << xmin+length, ymin+length, zmin+length;
	vertices.push_back(v);

	float norm_min = FLT_MAX;
	float norm_max = FLT_MIN;
	for(int i = 0; i < vertices.size(); ++i)
	{
		float norm = vertices[i].norm();
		if(norm < norm_min)
			norm_min = norm;
		if(norm > norm_max)
			norm_max = norm;
	}

	if(norm_max >= 1.0f && norm_min <= 1.0f)
		return true;

	return false;
}
// ������ռ����ͶƱ
void voteForPS()
{
	pcl::PointXYZRGB p;
	std::vector<int> indices;
	std::vector<float> dist;

	for(int i = 0; i < source_normal->size(); ++i)
	{
		// ������ֵ����Ϊxmin, ymin��zmin
		p.x = floor(source_normal->points[i].normal_x / num_res) * num_res;
		p.y = floor(source_normal->points[i].normal_y / num_res) * num_res;
		p.z = floor(source_normal->points[i].normal_z / num_res) * num_res;
		// ��������������
		kdtree_ps.nearestKSearch(p, 1, indices, dist);
		// ִ��ͶƱ
		++param_space[indices[0]].vote;
		param_space[indices[0]].point_indices.push_back(i);
	}
}
// �Բ����ռ���ж�ά�ĸ�˹�˲�;
void filtPS()
{
	float radius = 3.0*std_dev_gaussian;
	pcl::PointXYZRGB p;
	std::vector<int> indices;
	std::vector<float>dist;
	int* vote_ = new int[kdtree_ps.getInputCloud()->size()];
	memset(vote_, 0, sizeof(int) * kdtree_ps.getInputCloud()->size());
	for(int i = 0; i < kdtree_ps.getInputCloud()->size(); ++i)
	{
		kdtree_ps.radiusSearch(kdtree_ps.getInputCloud()->points[i], radius, indices, dist);
		float G = 0.0;
		for(int j = 0; j < indices.size(); ++j)
		{
			float d = pow(dist[j], 0.5f);
			G += gaussianKernal(d, std_dev_gaussian) * (float)(param_space[indices[j]].vote);
		}
		vote_[i] = floor(G+0.5f);
	}
	for(int i = 0; i < kdtree_ps.getInputCloud()->size(); ++i)
		param_space[i].vote = vote_[i];
	delete []vote_;
}
// ��˹�˺���
float gaussianKernal(float x, float std_dev)
{
	double sigma = (double)std_dev * (double)std_dev;
	double A = 2.0*(double)PI*sigma;
	double B = (double)x*(double)x/(-2.0*sigma);
	double C = pow(double(e), B);
	double D = C / A;
	return D;
}
// �Բ����ռ乹���ݶȳ�
void createGradientField()
{	
	// sink������
	int sink_index = -1;
	
	pcl::PointXYZRGB p;

	std::vector<int> indices;
	std::vector<float> dist;

	int ps_size = ps_cloud->size();

	while(true)
	{
		findMaxVoteInPS(param_space, ps_size, sink_index);
		if(sink_index == -1)
			break;

		// ����sinkpoint
		param_space[sink_index].sink_init = sink_index;
		SinkPoint sp;
		sp.index = sink_index;
		sp.vote_num = param_space[sink_index].point_indices.size();	// ����ʵ�ʵ�ͶƱ������voteֵΪ�˲������ֵ���ѱ仯��		
		
		std::queue<int> Q;
		Q.push(sink_index);
		while(!Q.empty())
		{
			int curIndex = Q.front();
			Q.pop();

			p = ps_cloud->points[curIndex];

			kdtree_ps.radiusSearch(p, search_radius_create_gradient, indices, dist);

			for(int i = 1; i < indices.size(); ++i)
			{
				if(param_space[indices[i]].vote == 0) continue;	// ��Ч��
				int curVote = param_space[curIndex].vote;
				int neiVote = param_space[indices[i]].vote;
				if(neiVote > curVote) continue;
				if(param_space[indices[i]].sink_init == -1)		// �ھӵ����û�б������
				{
					param_space[indices[i]].sink_init = sink_index;
					sp.vote_num += param_space[indices[i]].point_indices.size();
					Q.push(indices[i]);
				}
			}
		}
		if(sp.vote_num > T_vote_num)
		{
			sinkPointsSet.push_back(sp);
			cout << "sp[" << sinkPointsSet.size()-1 << "].index = " << sink_index << "; size = " << sinkPointsSet[sinkPointsSet.size()-1].vote_num << endl;
		}
	}
}
// �ڲ����ռ���Ѱ�ҵ�ǰ�������voteֵ�ĵ������
void findMaxVoteInPS(PSElem *&param_space, int ps_size, int &index)
{
	int maxvote = -1;
	for(int i = 0; i < ps_size; ++i)
	{
		if(param_space[i].sink_init != -1) continue;	// �õ��ѱ���ǹ�
		if(param_space[i].vote == 0) continue;			// �ĵ�Ϊ��Ч��
		if(param_space[i].vote > maxvote)
		{
			maxvote = param_space[i].vote;
			index = i;
		}
	}
	if(maxvote == -1)
		index = -1;
}
// У�������ռ���ݶȳ�
void rectifySinkFields()
{
	float dev_threshold_rad = dev_threshold * PI / 180.0f;

	std::vector<int> indices;
	std::vector<float> dist;
	for(int ii = 0; ii < sinkPointsSet.size(); ++ii)
	{
		int sink_index = sinkPointsSet[ii].index;
		pcl::PointXYZRGB p = ps_cloud->points[sink_index];
		float search_radius_max = sin(dev_threshold_rad*0.5f) * 2.0f;
		float radius = radius_base;
		while(true)
		{
			if(radius > search_radius_max) break;

			float S_total = 0;
			float S_sink = 0;

			kdtree_ps.radiusSearch(p, radius, indices, dist);

			for(int i = 0; i < indices.size(); ++i)
			{
				++S_total;
				if(param_space[indices[i]].sink_init == sink_index)
				{
					++ S_sink;
				}
			}
			if(S_sink/S_total < S_threshold) break;

			for(int i = 0; i < indices.size(); ++i)
			{
				if(param_space[indices[i]].sink_init == sink_index && param_space[indices[i]].sink == -1)
				{
					param_space[indices[i]].sink = sink_index;
				}
			}

			radius += delta_radius;
		}
	}
}
// �ָ�ƽ�����
void segmentPlanes()
{
	int ps_size = ps_cloud->size();
	for(int ii = 0; ii < sinkPointsSet.size(); ++ii)
	{
		int sink_index = sinkPointsSet[ii].index;

		PointCloudT::Ptr planes (new PointCloudT);
		pcl::PointXYZ p;
		// ���ȹ������ͬ�������ƽ����Ƽ���
		for(int i = 0; i < ps_size; ++i)
		{
			if(param_space[i].sink == sink_index)
			{
				for(int j = 0; j < param_space[i].point_indices.size(); ++j)
				{
					p.x = source_cloud->points[ param_space[i].point_indices[j] ].x;
					p.y = source_cloud->points[ param_space[i].point_indices[j] ].y;
					p.z = source_cloud->points[ param_space[i].point_indices[j] ].z;
					planes->push_back(p);
				}
			}
		}

		if(planes->size() < T_num_of_single_plane) continue;

		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
		kdtree.setInputCloud(planes);
		std::vector<int> indices;
		std::vector<float> dist;

		// ������Ҫ�ָ�plane_clouds�еĵ㣬���Ҫ�ָ��һ��ƽ�棬ÿ��ƽ���ڲ�����������
		bool* isProcessed = new bool[planes->size()];
		memset(isProcessed, 0, sizeof(bool)*planes->size());
		
		while(true)
		{
			int seedIndex = getSeedIndex(isProcessed, planes->size());
			if(seedIndex == -1) break;

			PointCloudT::Ptr plane (new PointCloudT);

			std::queue<int> Q;
			Q.push(seedIndex);
			while(!Q.empty())
			{
				int curIndex = Q.front();
				Q.pop();

				isProcessed[curIndex] = true;
				plane->push_back(planes->points[curIndex]);

				p = planes->points[curIndex];
				kdtree.radiusSearch(p, search_radius_for_plane_seg, indices, dist);
				for(int i = 0; i < indices.size(); ++i)
				{
					if(isProcessed[indices[i]]) continue;
					isProcessed[indices[i]] = true;
					Q.push(indices[i]);
					plane->push_back(planes->points[indices[i]]);
				}
			}
			if(plane->size() >= T_num_of_single_plane)
			{
				Plane plane_;
				plane_.points_set = plane;
				plane_clouds.push_back(plane_);
			}
		}
		delete []isProcessed;
		isProcessed = NULL;
	}
}
// ��ȡ�ָ�ƽ������ӵ�;
int getSeedIndex(bool* &isProcessed, int size)
{
	for(int i = 0; i < size; ++i)
	{
		if(!isProcessed[i])
			return i;
	}
	return -1;
}
// ���÷ָ�õ���������Ԫ����ƽ�����
void growPlaneArea()
{
	// �Ƕ�ƫ����ֵ,ת��Ϊ����
	T_angle_bias_integrate = T_angle_bias_integrate * PI / 180.0f;

	// �������󣺳��ηָ��ƽ����Ƽ���plane_clouds(vector����);
	PointCloudT::Ptr plane (new PointCloudT);

	// ԭʼ�����еĵ��Ƿ��ѱ������
	bool *isProcessed = new bool[source_cloud->size()];

	pcl::PointXYZ p;
	pcl::Normal pn;
	std::vector<int> indices;
	std::vector<float> dist;

	for(int ii = 0; ii < plane_clouds.size(); ++ii)
	{
		memset(isProcessed, 0, sizeof(bool)*source_cloud->size());
		plane = plane_clouds[ii].points_set;
		// ����Ѱ��ƽ��ı߽��
		std::queue<int> border_points;
		int point_index_for_normal_recify = -1;
		// ��ԭʼ����Ⱦɫ(������ɫ��true��false)
		for(int i = 0; i < plane->size(); ++i)
		{			
			p.x = plane->points[i].x;
			p.y = plane->points[i].y;
			p.z = plane->points[i].z;
			kdtree_source.nearestKSearch(p, 1, indices, dist);
			isProcessed[indices[0]] = true;
			if(point_index_for_normal_recify==-1)
			{
				point_index_for_normal_recify = indices[0];
			}
		}
		// Ѱ�ұ߽��
		for(int i = 0; i < source_cloud->size(); ++i)
		{
			if(isProcessed[i])
			{
				kdtree_source.radiusSearch(source_cloud->points[i], radius_border, indices, dist);
				for(int j = 0; j < indices.size(); ++j)
				{
					if(!isProcessed[indices[j]])	// �ܱ��������û��������ھӵ�, ��ĵ㱻����Ϊ�߽��
					{
						border_points.push(i);
						break;
					}
				}
			}
		}

		// ͨ���߽������ƽ��
		// ����ƽ��ķ�����������
		// ���㷨����ʱ��Ҫע�⽫�䷽�����Ϊָ��ƽ����ⲿ
		Eigen::Vector3f normal_seed_plane;	// ����ƽ��ķ�����
		Eigen::Vector3f normal_update;		// ���º��ƽ��ķ�����
		float curvature_seed_plane;			// ����ƽ�������
		float curvature_update;				// ���º��ƽ�������
		Eigen::Vector4f param_seed_plane;	// ����ƽ��Ĳ���
		Eigen::Vector4f param_update;		// ���º��ƽ��Ĳ���
		std::vector<int> index;
		for(int i = 0; i < plane->size(); ++i)
		{
			index.push_back(i);
		}
		pcl::computePointNormal<pcl::PointXYZ>(*plane, index, param_seed_plane, curvature_seed_plane);
		normal_seed_plane << param_seed_plane[0], param_seed_plane[1], param_seed_plane[2];
		Eigen::Vector3f p_n;
		p_n << source_normal->points[point_index_for_normal_recify].normal_x,
				source_normal->points[point_index_for_normal_recify].normal_y,
				source_normal->points[point_index_for_normal_recify].normal_z;
		if(p_n.dot(normal_seed_plane) < 0)	// У������ƽ�淨�����ķ���ʹ��ָ�������ⲿ
		{
			normal_seed_plane = -1.0f * normal_seed_plane;
		}
		plane_clouds[ii].coeff.values.push_back(normal_seed_plane[0]);
		plane_clouds[ii].coeff.values.push_back(normal_seed_plane[1]);
		plane_clouds[ii].coeff.values.push_back(normal_seed_plane[2]);

		bool isLastPointValid = true;
		while(!border_points.empty())
		{
			int curIndex = border_points.front();
			border_points.pop();

			kdtree_source.radiusSearch(source_cloud->points[curIndex], radius_local, indices, dist);
			for(int i = 0; i < indices.size(); ++i)
			{
				if(isProcessed[indices[i]]) continue;

				isProcessed[indices[i]] = true;

				if(isLastPointValid)
				{
					pcl::PointXYZ point;
					point.x = source_cloud->points[indices[i]].x;
					point.y = source_cloud->points[indices[i]].y;
					point.z = source_cloud->points[indices[i]].z;
					plane->push_back(point);
				}else{
					plane->points[plane->size()-1].x = source_cloud->points[indices[i]].x;
					plane->points[plane->size()-1].y = source_cloud->points[indices[i]].y;
					plane->points[plane->size()-1].z = source_cloud->points[indices[i]].z;
				}

				// ��֤����ӵĵ��Ƿ�Ϸ�
				// ����index
				if(isLastPointValid)
				{
					index.push_back(index.size());
				}
				pcl::computePointNormal<pcl::PointXYZ>(*plane, index, param_update, curvature_update);
				normal_update << param_update[0], param_update[1], param_update[2];
				if(normal_seed_plane.dot(normal_update) < 0)
				{
					normal_update = -1.0f*normal_update;
				}

				float dev_rad = arccos(normal_seed_plane.dot(normal_update));	// ���º�ƽ��������ƽ�淨����ƫ��Ļ���
				float curvature_bias; /*= (curvature_update-curvature_seed_plane)/curvature_seed_plane * 100.000;*/
				if(abs(curvature_seed_plane) <= 0.0001f)
				{
					curvature_bias = curvature_update-curvature_seed_plane;
				}else{
					curvature_bias = (curvature_update-curvature_seed_plane)/curvature_seed_plane * 100.000;
				}
				
				std::vector<int> indices_local;
				std::vector<float> dist_local;
				Eigen::Vector4f param_local;
				float curvature_local;
				//float r_local = 0.1f;
				kdtree_source.radiusSearch(source_cloud->points[indices[i]], r_local, indices_local, dist_local);
				pcl::computePointNormal<pcl::PointXYZ>(*source_cloud, indices_local, param_local, curvature_local);
				Eigen::Vector3f n_local, n_local_prev;
				n_local << param_local[0], param_local[1], param_local[2];
				n_local_prev << source_normal->points[indices[i]].normal_x,	// ���ݸĵ�����ķ�����
					            source_normal->points[indices[i]].normal_y,
								source_normal->points[indices[i]].normal_z;
				if(n_local.dot(n_local_prev) < 0)	// ��֤�ֲ���ķ�����ָ�������ⲿ
				{
					n_local = -1.0f * n_local;
				}
				float point_normal_bias = arccos(n_local.dot(normal_update));

				if( abs(curvature_seed_plane) > 0.0001f &&
					dev_rad <= T_angle_bias_integrate &&
					curvature_bias <= T_curvature_bias &&
					point_normal_bias <= T_point_normal_bias*PI/180.0)
				{
					isLastPointValid = true;
					border_points.push(indices[i]);
				}else if( abs(curvature_seed_plane) <= 0.0001f &&
					dev_rad <= T_angle_bias_integrate && 
					curvature_bias <= 10.0*curvature_seed_plane &&
					point_normal_bias <= T_point_normal_bias*PI/180.0){
					isLastPointValid = true;
					border_points.push(indices[i]);
				}
				else{
					isLastPointValid = false;
				}
			}
		}
	}

	delete []isProcessed;
}
// ƽ��ϲ�
void mergePlanes()
{
	/*
	// �����ϵͼ
	std::vector<std::vector<int>> E;
	for(int i = 0; i < plane_clouds.size(); ++i)
	{
		std::vector<int> relation;
		E.push_back(relation);
	}

	int plane_size = plane_clouds.size();
	for(int i = 0; i < plane_size-1; ++i)
	{
		for(int j = i+1; j < plane_size; ++j)
		{
			int plane_i_size = plane_clouds[i].points_set->size();
			int plane_j_size = plane_clouds[j].points_set->size();

			int common_points_number = getCommonPointsNumber(plane_clouds[i].points_set, plane_clouds[j].points_set);

			int size = plane_i_size <= plane_j_size ? plane_i_size : plane_j_size;

			float ratio_merge_planes = (float)common_points_number / (float)size;

			if(ratio_merge_planes >= T_ratio_merge_planes)
			{
				E[i].push_back(j);
				E[j].push_back(i);
			}
		}
	}

//////////////////////////////////////////////////////////////////////////////////////////////
	bool *isNodeProcessed = new bool[plane_clouds.size()];
	memset(isNodeProcessed, 0, sizeof(bool) * plane_clouds.size());

	for(int i = 0; i < E.size(); ++i)
	{
		if(isNodeProcessed[i])	// �����ǰ�ڵ��Ѿ���������ˣ��������һ��
			continue;
		
		std::vector<int> cluster;
		std::queue<int> Q;
		Q.push(i);
		cluster.push_back(i);
		isNodeProcessed[i] = true;

		for(int j = 0; j < E[i].size(); ++j)
		{
			if(isNodeProcessed[E[i][j]])	// ���E[i][j]�ڵ��ѱ���������������һ��
				continue;
			Q.push(E[i][j]);
			cluster.push_back(E[i][j]);
			isNodeProcessed[E[i][j]] = true;
		}
		while(!Q.empty())
		{
			int index = Q.front();
			Q.pop();

			for(int j = 0; j < E[index].size(); ++j)
			{
				if(isNodeProcessed[E[index][j]])
					continue;

				Q.push(E[index][j]);
				cluster.push_back(E[index][j]);
				isNodeProcessed[E[index][j]] = true;
			}
		}

		for(int j = 1; j < cluster.size(); ++j)
		{
			mergeClouds(plane_clouds[cluster[0]].points_set, plane_clouds[cluster[j]].points_set);
			plane_clouds[cluster[j]].points_set->clear();
			plane_clouds[cluster[j]].points_set->resize(0);
		}
	}

	for(int i = 0; i < plane_clouds.size(); ++i)
	{
		if(plane_clouds[i].points_set->size() > 0)
		{
			plane_clouds_final.push_back(plane_clouds[i]);
		}
	}

	plane_clouds.clear();
	plane_clouds.resize(0);

	cout << "final plane number = " << plane_clouds_final.size() << endl;

	delete []isNodeProcessed;
	*/
	plane_clouds_final = plane_clouds;
}
// ��ȡ����ƽ�湫���������
int getCommonPointsNumber(PointCloudT::Ptr &cloud1, PointCloudT::Ptr &cloud2)
{
	bool *isMarked = new bool[source_cloud->size()];
	memset(isMarked, 0, sizeof(bool)*source_cloud->size());

	pcl::PointXYZ p;
	std::vector<int> indices;
	std::vector<float> dist;

	for(int i = 0; i < cloud1->size(); ++i)
	{
		p.x = cloud1->points[i].x;
		p.y = cloud1->points[i].y;
		p.z = cloud1->points[i].z;

		kdtree_source.nearestKSearch(p, 1, indices, dist);

		isMarked[indices[0]] = true;
	}

	int count = 0;
	for(int i = 0; i < cloud2->size(); ++i)
	{
		p.x = cloud2->points[i].x;
		p.y = cloud2->points[i].y;
		p.z = cloud2->points[i].z;

		kdtree_source.nearestKSearch(p, 1, indices, dist);
		if(isMarked[indices[0]])
			++count;
	}
	delete isMarked;
	return count;
}
// ��Ŀ����ƺϲ���Դ����
void mergeClouds(PointCloudT::Ptr &source, PointCloudT::Ptr &target)
{
	bool *isMarked = new bool[source_cloud->size()];
	memset(isMarked, 0, sizeof(bool)*source_cloud->size());

	pcl::PointXYZ p;
	std::vector<int> indices;
	std::vector<float> dist;

	for(int i = 0; i < source->size(); ++i)
	{
		p.x = source->points[i].x;
		p.y = source->points[i].y;
		p.z = source->points[i].z;

		kdtree_source.nearestKSearch(p, 1, indices, dist);

		isMarked[indices[0]] = true;
	}

	for(int i = 0; i < target->size(); ++i)
	{
		p.x = target->points[i].x;
		p.y = target->points[i].y;
		p.z = target->points[i].z;

		kdtree_source.nearestKSearch(p, 1, indices, dist);

		if(!isMarked[indices[0]])	// ���û�б��˵��Դ������û�д˵�
		{
			source->push_back(target->points[i]);
		}
	}

	delete []isMarked;
}
// ����λ�ƽ�����
void polyPlanes()
{
	for(int i = 0; i < plane_clouds_final.size(); ++i)
	{
		if(plane_clouds_final[i].border != NULL) continue;	// �������ܵ�ʱ���ٴμ���
		PointCloudT::Ptr border (new PointCloudT);
		plane_clouds_final[i].border = border;
		PointCloudT::Ptr cloud (new PointCloudT);
		*cloud = *plane_clouds_final[i].points_set;
		Eigen::Vector3f pn;
		pn << plane_clouds_final[i].coeff.values[0],
			  plane_clouds_final[i].coeff.values[1],
			  plane_clouds_final[i].coeff.values[2];
		polyPointCloud(/*plane_clouds_final[i].points_set*/cloud, plane_clouds_final[i].border, pn);
	}
}
void polyPointCloud( PointCloudT::Ptr &cloud,  
					 PointCloudT::Ptr &border,
					 Eigen::Vector3f &pn)
{
	// ����С���˷�����ƽ�淽��
	std::vector<int> indices;
	for(int i = 0; i < cloud->size(); ++i)
	{
		indices.push_back(i);
	}
	Eigen::Vector4f plane_param;
	float curvature;
	pcl::computePointNormal(*cloud, indices, plane_param, curvature);
	/*cout << "plane_param = " << plane_param << endl;*/

	// �������еĵ�ȫ��ͶӰ����ƽ����
	pcl::PointXYZ p_proj;
	for(int i = 0; i < cloud->size(); ++i)
	{
		projPoint2Plane( cloud->points[i], p_proj, plane_param);
		cloud->points[i].x = p_proj.x;
		cloud->points[i].y = p_proj.y;
		cloud->points[i].z = p_proj.z;
	}
	// ��ƽ��㼯���찼��
	pcl::ConcaveHull<pcl::PointXYZ> caHull;
	caHull.setInputCloud(cloud);
	caHull.setAlpha(alpha_poly);
	PointCloudT::Ptr output(new PointCloudT);
	/*caHull.reconstruct(*output);*/
	std::vector<pcl::Vertices> polygons;
	caHull.reconstruct(*output, polygons);
	/*cout << "output->size() = " << output->size() << endl;
	for(int i = 0; i < polygons.size(); ++i)
	{
		cout << "i = " << i << "; poly size = " << polygons[i].vertices.size() << endl;
	}*/

	// cloud->clear();
	for(int i = 0; i < polygons[0].vertices.size(); ++i)
	{
		border->push_back(output->points[polygons[0].vertices[i]]);
	}

	// Ҫ��֤border�еĵ�p1,p2,...,pn,����ʱ�����еġ��������ַ��򣬴�ĴָҪָ��������ⲿ
	Eigen::Vector3f v01, v12;
	v01 << border->points[1].x - border->points[0].x,
		   border->points[1].y - border->points[0].y,
		   border->points[1].z - border->points[0].z;
	v12 << border->points[2].x - border->points[1].x,
		   border->points[2].y - border->points[1].y,
		   border->points[2].z - border->points[1].z;
	v01.normalize();
	v12.normalize();
	Eigen::Vector3f v_dir;
	v_dir = v01.cross(v12);
	v_dir.normalize();
	if(v_dir.dot(pn) < 0)
	{
		border->clear();
		for(int i = polygons[0].vertices.size()-1; i >= 0; --i)
		{
			border->push_back(output->points[polygons[0].vertices[i]]);
		}
	}
	output->clear();
}
// ����ͶӰ��ƽ����
void projPoint2Plane( pcl::PointXYZ &source_point, pcl::PointXYZ &dest_point, Eigen::Vector4f &plane_param )
{
	float lambda = 2.0*(plane_param(0)*source_point.x + plane_param(1)*source_point.y + plane_param(2)*source_point.z + plane_param(3));
	dest_point.x = source_point.x - lambda/2.0*plane_param(0);
	dest_point.y = source_point.y - lambda/2.0*plane_param(1);
	dest_point.z = source_point.z - lambda/2.0*plane_param(2);
}

// ƽ�����
// ԭ��һ���ֵ㱾������ƽ�浫����û�б����Ϊƽ���Ա
// �����Ŀ���ǽ��ⲿ��ƽ���ǳɶ�Ӧ��ƽ��
//bool isFirstPostProcess = true;
int plane_start_index = 0;
void postProcessPlanes()
{
	/*
	// ��������
	isFirstUseRegulateNormal = true;	// ���Ʒ�����ʱʹ��
	sinkPointsSet.clear();
	for(int i = 0; i < plane_clouds.size(); ++i)
	{
		plane_clouds[i].border->clear();
		plane_clouds[i].coeff.values.clear();
		plane_clouds[i].points_set->clear();
		plane_clouds[i].triangles.clear();
	}
	plane_clouds.clear();
	for(int i = 0; i < ps_cloud->size(); ++i)
	{
		param_space[i].point_indices.clear();
		param_space[i].sink = param_space[i].sink_init = -1;
		param_space[i].vote = 0;
	}
	delete []param_space;
	param_space = NULL;

	// ����ÿ��ƽ��ķ���
	for(int i = 0; i < plane_clouds_final.size(); ++i)
	{
		Eigen::Vector4f param;
		float curvature;
		std::vector<int>index;
		for(int j = 0; j < plane_clouds_final[i].points_set->size(); ++j)
			index.push_back(j);
		pcl::computePointNormal<pcl::PointXYZ>(*plane_clouds_final[i].points_set, index, param, curvature);
		Eigen::Vector4f v;
		v << plane_clouds_final[i].coeff.values[0], 
			plane_clouds_final[i].coeff.values[1], 
			plane_clouds_final[i].coeff.values[2], 
			0;
		if(v.dot(param) < 0)
			param = -1.0f * param;
		plane_clouds_final[i].coeff.values.clear();
		plane_clouds_final[i].coeff.values.push_back(param[0]);
		plane_clouds_final[i].coeff.values.push_back(param[1]);
		plane_clouds_final[i].coeff.values.push_back(param[2]);
		plane_clouds_final[i].coeff.values.push_back(param[3]);
	}
	
	// Ȼ�����Щû�б���ǵĵ���ȡ����
	if(isFirstPostProcess)
	{
		*source_cloud_backup = *source_cloud;	// ����ԭʼ����(������һ�Σ�
		*source_normal_backup = *source_normal;
		isFirstPostProcess = false;
	}
	

	PointCloudT::Ptr cloud (new PointCloudT);	
	*cloud = *source_cloud;
	source_cloud->clear();

	if(isProcessed) delete []isProcessed;
	isProcessed = new bool[cloud->size()];
	memset(isProcessed, 0, sizeof(bool)*cloud->size());

	std::vector<int> indices;
	std::vector<float> dist;
	kdtree_source.setInputCloud(cloud);
	for(int ii = 0; ii < plane_clouds_final.size(); ++ii)
	{
		for(int i = 0; i < plane_clouds_final[ii].points_set->size(); ++i)
		{
			kdtree_source.nearestKSearch(plane_clouds_final[ii].points_set->points[i],1,indices, dist);
			isProcessed[indices[0]] = true;
		}
	}

	// ����Щ��������ƽ��ĵ��ٹ���һ��
	for(int i = 0; i < cloud->size(); ++i)
	{
		if(i == cloud->size()/4)
		{
			cout << "i = " << i << endl;
		}
		if(i == cloud->size()/2)
		{
			cout << "i = " << i << endl;
		}
		if(i == cloud->size()/4*3)
		{
			cout << "i = " << i << endl;
		}

		if(isProcessed[i]) continue;
	
		pcl::PointXYZ p = cloud->points[i];
		for(int ii = plane_start_index; ii < plane_clouds_final.size(); ++ii)
		{
			if(isPointInPoly(p, plane_clouds_final[ii]))
			{
				isProcessed[i] = true;
				plane_clouds_final[ii].points_set->push_back(p);
			}
		}
	}
	// ����plane_start_index
	plane_start_index = plane_clouds_final.size();

	for(int i = 0; i < cloud->size(); ++i)
	{
		if(!isProcessed[i])
		{
			source_cloud->push_back(cloud->points[i]);
		}
	}
	*/
	// just for testing:
	/*Plane plane_ = plane_clouds_final[0];
	plane_clouds_final.clear();
	plane_clouds_final.push_back(plane_);
	plane_clouds_final[0].points_set->clear();
	for(int i = 0; i < cloud->size(); ++i)
	{
		if(isPointInPlane(cloud->points[i], plane_clouds_final[0]) &&
			isPointInPlane_complement(cloud->points[i], plane_clouds_final[0]))
		{
			plane_clouds_final[0].points_set->push_back(cloud->points[i]);
		}
	}*/
	///////////////////////////////////////////////////////////////////////////
	/*
	// ��ʣ�µĵ��ƽ��о��࣬ÿ���������Ŀ��������һ����ֵ
	clusterFilt();

	//*source_cloud = *cloud;
	kdtree_source.setInputCloud(source_cloud);	// ͬʱ�ǵø���kdtree
	source_normal->clear();						// ͬʱ�ǵ���շ�����
	cout << "updated source_cloud->size() = " << source_cloud->size() << endl;
	delete []isProcessed;
	isProcessed = NULL;
	
	isFirstRun = false;					// �ǵ�һ�������procedure��

	*/
}
// ��Դ���ƽ��о���ָ������Ԫ���������ٵľ���
void clusterFilt()
{
	if(source_cloud->size() == 0) 
	{
		cout << "all points have been processed." << endl;
		return;
	}
	// ���Ұ뾶��radius_localʵ��ƽ����������ʱ��Ѱ�Ҿֲ�ƽ��İ뾶
	// ��ֵ��T_cluster_numÿ���������С������ֵ
	if(isProcessed!=NULL)
	{
		delete []isProcessed;
	}
	isProcessed = new bool[source_cloud->size()];
	memset(isProcessed, 0, sizeof(bool)*source_cloud->size());

	bool*isValid = new bool[source_cloud->size()];
	memset(isValid, 1, sizeof(bool)*source_cloud->size());

	kdtree_source.setInputCloud(source_cloud);
	std::vector<int> indices_search;
	std::vector<float> dist_search;
	
	int seed_index = -1;
	while(true)
	{
		seed_index = getSeedIndex(isProcessed, source_cloud->size());
		// cout << "seed_index = " << seed_index << endl;
		if(seed_index == -1)
			break;
		std::vector<int> indices;	// �����������е���Դ�����е�����
		indices.push_back(seed_index);
		std::queue<int> Q;
		Q.push(seed_index);
		while(!Q.empty())
		{
			int cur_index = Q.front();
			Q.pop();
			isProcessed[cur_index] = true;

			kdtree_source.radiusSearch(source_cloud->points[cur_index], radius_local, indices_search, dist_search);
			for(int i = 1; i < indices_search.size(); ++i)
			{
				if(isProcessed[indices_search[i]]) continue;
				Q.push(indices_search[i]);
				indices.push_back(indices_search[i]);
				isProcessed[indices_search[i]] = true;
			}
		}
		// while��������һ�����ѱ��������
		if(indices.size() <= T_cluster_num)
		{			
			for(int i = 0; i < indices.size(); ++i)
			{
				isValid[indices[i]] = false;
			}
		}
		// ������������
		seed_index = -1;
	}

	PointCloudT::Ptr cloud (new PointCloudT);
	*cloud = *source_cloud;
	source_cloud->clear();
	for(int i = 0; i < cloud->size(); ++i)
	{
		if(isValid[i])
			source_cloud->push_back(cloud->points[i]);
	}
	cloud->clear();
	// �ǵ������ڴ�ռ�
	delete []isProcessed;
	isProcessed = NULL;
	delete []isValid;
}
// ���ǻ�ƽ��
// ǰ�᣺ƽ��ķ�����ָ�������ⲿ
//		 ƽ��߽����Ҫ�������ַ��򣺴�Ĵָָ��ƽ�����ࣨ����line(p0,p1)��ߵĵ��ڶ�����ڲ����ұߵĵ��ڶ�����ⲿ��
void trianglatePlane(Plane &plane)
{

}
// ĳ���Ƿ���ƽ����
bool isPointInPlane(pcl::PointXYZ &p, Plane &plane)
{
	Eigen::Vector4f plane_param;
	plane_param << plane.coeff.values[0],
					plane.coeff.values[1],
					plane.coeff.values[2],
					plane.coeff.values[3];
	float dist;
	pcl::PointXYZ p_proj;
	getInfoBetPointAndPlane(p, plane_param, dist, p_proj);
	// ����㵽ƽ��ľ��볬������ֵ����õ�Ϊƽ���ⲿ�ĵ�
	if(dist > T_dist_point_plane)	return false;

	// ������������ƽ���ϵĵ㣬������ͶӰ��p_proj�������߶��ϵ�ͶӰ��)
	std::vector<std::pair<pcl::PointXYZ/*ͶӰ��*/, float/*����*/>> info_p_line;
	for(int i = 0; i < plane.border->size(); ++i)
	{
		std::pair<pcl::PointXYZ, float> info;
		Eigen::Vector3f n;
		if(i==plane.border->size()-1)
		{
			n << plane.border->points[0].x - plane.border->points[plane.border->size()-1].x,
				 plane.border->points[0].y - plane.border->points[plane.border->size()-1].y,
				 plane.border->points[0].z - plane.border->points[plane.border->size()-1].z;
			n.normalize();
		}else{
			n << plane.border->points[i+1].x - plane.border->points[i].x,
				 plane.border->points[i+1].y - plane.border->points[i].y,
				 plane.border->points[i+1].z - plane.border->points[i].z;
			n.normalize();
		}
		distP2L( p_proj/*ͶӰ�㱣֤����*/, 
			     plane.border->points[i]/*�����߶ε����*/, 
				 n/*�߶εķ�������*/, 
				 info.first/*ֱ���ϵ�ͶӰ��*/, 
				 info.second/*�㵽ֱ�ߵľ���*/);
		info_p_line.push_back(info);
	}

	int index_minDist = -1;
	float minDist = FLT_MAX;
	for(int i = 0; i < info_p_line.size(); ++i)
	{
		// ֱ���ϵ�ͶӰ���Ƿ������߶�֮��
		float distAB, distPA, distPB;
		pcl::PointXYZ p_projOnLine = info_p_line[i].first;
		pcl::PointXYZ p_A, p_B;

		p_A = plane.border->points[i];
		if(i < plane.border->size()-1)
		{			
			p_B = plane.border->points[i+1];
		}else{
			p_B = plane.border->points[0];
		}
		distAB = distP2P(p_A, p_B);
		distPA = distP2P(p_projOnLine, p_A);
		distPB = distP2P(p_projOnLine, p_B);
		float ratio = abs(distPA+distPB-distAB) / distAB;
		if(ratio <= 0.001)
		{
			if(minDist > info_p_line[i].second)
			{
				minDist = info_p_line[i].second;
				index_minDist = i;
			}
		}
	}

	if(index_minDist == -1)
	{
		return false;
	}

	// ��ͶӰ��p_proj�Ƿ����߶�index_minDist�����
	pcl::PointXYZ p_A, p_B;
	p_A = plane.border->points[index_minDist];
	if(index_minDist < plane.border->size()-1)
		p_B = plane.border->points[index_minDist+1];
	else
		p_B = plane.border->points[0];

	Eigen::Vector3f dir_AB;
	dir_AB << p_B.x - p_A.x, 
			  p_B.y - p_A.y, 
			  p_B.z - p_A.z;
	dir_AB.normalize();

	Eigen::Vector3f dir_AP;
	dir_AP << p_proj.x - p_A.x,
			  p_proj.y - p_A.y,
			  p_proj.z - p_A.z;
	dir_AP.normalize();

	Eigen::Vector3f normal_plane;
	normal_plane << plane.coeff.values[0],
					plane.coeff.values[1],
					plane.coeff.values[2];
	normal_plane.normalize();

	return ((dir_AB.cross(dir_AP)).dot(normal_plane) > 0);
}
bool isPointInPlane_complement(pcl::PointXYZ &p, Plane &plane)
{
	Eigen::Vector4f plane_param;
	plane_param << plane.coeff.values[0],
					plane.coeff.values[1],
					plane.coeff.values[2],
					plane.coeff.values[3];
	float dist;
	pcl::PointXYZ p_proj;
	getInfoBetPointAndPlane(p, plane_param, dist, p_proj);
	// ����㵽ƽ��ľ��볬������ֵ����õ�Ϊƽ���ⲿ�ĵ�
	if(dist > T_dist_point_plane)	return false;

	// ������������ƽ���ϵĵ㣬������ͶӰ��p_proj�������߶��ϵ�ͶӰ��)
	std::vector<std::pair<pcl::PointXYZ/*ͶӰ��*/, float/*����*/>> info_p_line;
	for(int i = 0; i < plane.border->size(); ++i)
	{
		std::pair<pcl::PointXYZ, float> info;
		Eigen::Vector3f n;
		if(i==plane.border->size()-1)
		{
			n << plane.border->points[0].x - plane.border->points[plane.border->size()-1].x,
				 plane.border->points[0].y - plane.border->points[plane.border->size()-1].y,
				 plane.border->points[0].z - plane.border->points[plane.border->size()-1].z;
			n.normalize();
		}else{
			n << plane.border->points[i+1].x - plane.border->points[i].x,
				 plane.border->points[i+1].y - plane.border->points[i].y,
				 plane.border->points[i+1].z - plane.border->points[i].z;
			n.normalize();
		}
		distP2L( p_proj/*ͶӰ�㱣֤����*/, 
			     plane.border->points[i]/*�����߶ε����*/, 
				 n/*�߶εķ�������*/, 
				 info.first/*ֱ���ϵ�ͶӰ��*/, 
				 info.second/*�㵽ֱ�ߵľ���*/);
		info_p_line.push_back(info);
	}	

	int index_minDist = -1;
	// float minDist = FLT_MAX;
	// ����������������p��ĳ���߶ε�ͶӰ��֮��û�����κ������߶��ཻ���ҳ�����߶ε�index
	pcl::PointXYZ p_inter;
	for(int i = 0; i < info_p_line.size(); ++i)
	{
		// ֱ���ϵ�ͶӰ���Ƿ������߶�֮��
		float distAB, distPA, distPB;
		pcl::PointXYZ p_projOnLine = info_p_line[i].first;
		pcl::PointXYZ p_A, p_B;

		p_A = plane.border->points[i];
		if(i < plane.border->size()-1)
		{			
			p_B = plane.border->points[i+1];
		}else{
			p_B = plane.border->points[0];
		}
		distAB = distP2P(p_A, p_B);
		distPA = distP2P(p_projOnLine, p_A);
		distPB = distP2P(p_projOnLine, p_B);
		float ratio = abs(distPA+distPB-distAB) / distAB;
		if(ratio <= 0.001)	// ͶӰ�������߶�i��
		{
			// ���������p��ͶӰ����߶�������߶�i֮������������߶��Ƿ��н��㣬ֻ��ȫ��û�н���ſ����ж�p�Ƿ��Ƕ�����ڵĵ�
			bool is_clean = true;// Ĭ������һ�߶ξ��޽���
			for(int j = 0; j < info_p_line.size(); ++j)
			{
				if(i == j) continue;
				pcl::PointXYZ begin_point, end_point;
				begin_point = plane.border->points[j];
				if(j<info_p_line.size()-1)
				{
					end_point = plane.border->points[j+1];
				}else{
					end_point = plane.border->points[0];
				}
				if(isBothLineSegsIntersect(begin_point, end_point, p_proj, info_p_line[i].first,p_inter))
				{
					is_clean = false;
					break;
				}
			}
			if(is_clean)
			{
				// û�н��㣬���Ըĵ���Ϊ�о���
				index_minDist = i;
				break;
			}
		}
	}

	if(index_minDist == -1)
	{
		return false;
	}

	// ��ͶӰ��p_proj�Ƿ����߶�index_minDist�����
	pcl::PointXYZ p_A, p_B;
	p_A = plane.border->points[index_minDist];
	if(index_minDist < plane.border->size()-1)
		p_B = plane.border->points[index_minDist+1];
	else
		p_B = plane.border->points[0];

	Eigen::Vector3f dir_AB;
	dir_AB << p_B.x - p_A.x, 
			  p_B.y - p_A.y, 
			  p_B.z - p_A.z;
	dir_AB.normalize();

	Eigen::Vector3f dir_AP;
	dir_AP << p_proj.x - p_A.x,
			  p_proj.y - p_A.y,
			  p_proj.z - p_A.z;
	dir_AP.normalize();

	Eigen::Vector3f normal_plane;
	normal_plane << plane.coeff.values[0],
					plane.coeff.values[1],
					plane.coeff.values[2];
	normal_plane.normalize();

	return ((dir_AB.cross(dir_AP)).dot(normal_plane) > 0);
}
bool isPointInPoly(pcl::PointXYZ &p, Plane &plane)
{
	Eigen::Vector4f plane_param;
	plane_param << plane.coeff.values[0],
					plane.coeff.values[1],
					plane.coeff.values[2],
					plane.coeff.values[3];
	float dist;
	pcl::PointXYZ p_proj;
	getInfoBetPointAndPlane(p, plane_param, dist, p_proj);
	// ����㵽ƽ��ľ��볬������ֵ����õ�Ϊƽ���ⲿ�ĵ�
	if(dist > T_dist_point_plane)	return false;

	std::vector<int> count_for_intersect;
	
	srand(time(0));

	pcl::PointXYZ p_base = p_proj;
	pcl::PointXYZ p_base_lambda, p_inter;
	float lambda = 10000;
	Eigen::Vector3f line_dir_p;
	Eigen::Vector3f line_dir, plane_norm;
	pcl::PointXYZ start_p, end_p;
	plane_norm << plane.coeff.values[0], plane.coeff.values[1], plane.coeff.values[2];
	for(int i = 0; i < 10; ++i)
	{
		int index = rand() % plane.border->size();
		start_p = plane.border->points[index];
		if(index == plane.border->size()-1)
			end_p = plane.border->points[0];
		else
			end_p = plane.border->points[index+1];

		line_dir << end_p.x - start_p.x,
					end_p.y - start_p.y,
					end_p.z - start_p.z;
		line_dir.normalize();
		line_dir_p = line_dir.cross(plane_norm);
		line_dir_p.normalize();
		p_base_lambda.x = p_base.x + lambda * line_dir_p[0];
		p_base_lambda.y = p_base.y + lambda * line_dir_p[1];
		p_base_lambda.z = p_base.z + lambda * line_dir_p[2];

		// �����߶�[p_base, p_base_lambda]�������бߵĽ�������
		int count = 0;
		for(int j = 0; j < plane.border->size(); ++j)
		{
			start_p = plane.border->points[j];
			if(j==plane.border->size()-1)
				end_p = plane.border->points[0];
			else
				end_p = plane.border->points[j+1];

			if(isBothLineSegsIntersect(start_p, end_p, p_base, p_base_lambda,p_inter))
				++count;
		}
		count_for_intersect.push_back(count);
	}

	int count = 0;
	for(int i = 0; i < count_for_intersect.size(); ++i)
		count += count_for_intersect[i] % 2;
	//cout << "count = " << count << endl;
	return count >= count_for_intersect.size()/2;
}

bool isBothLineSegsIntersect(pcl::PointXYZ &pa, pcl::PointXYZ &pb, pcl::PointXYZ &pc, pcl::PointXYZ &pd, pcl::PointXYZ &p_inter)
{
	p_inter.x = p_inter.y = p_inter.z = FLT_MIN;
	Eigen::Vector3f nab, ncd;
	nab << pb.x - pa.x,
		   pb.y - pa.y, 
		   pb.z - pa.z;
	nab.normalize();
	ncd << pd.x - pc.x, 
		   pd.y - pc.y, 
		   pd.z - pc.z;
	ncd.normalize();
	Eigen::Vector3f pa_pc;
	pa_pc << pc.x - pa.x,
			 pc.y - pa.y,
			 pc.z - pa.z;
	float lambda1, lambda2;
	lambda1 = lambda2 = FLT_MIN;
	if( abs(nab.dot(ncd)) <= 0.001f )
	{
		lambda1 = nab.dot(pa_pc);
		lambda2 = -1.0f*ncd.dot(pa_pc);
	}else if( nab.dot(ncd) >= 0.9999f ){
		return false;
	}else{
		float c1 = 1.0f - nab.dot(ncd) * nab.dot(ncd);
		float c2 = nab.dot(pa_pc) * nab.dot(ncd) - ncd.dot(pa_pc);
		lambda2 = c2/c1;
		lambda1 = (lambda2 + ncd.dot(pa_pc)) / nab.dot(ncd);
	}
	pcl::PointXYZ p1, p2;
	p1.x = pa.x + lambda1*nab[0];
	p1.y = pa.y + lambda1*nab[1];
	p1.z = pa.z + lambda1*nab[2];

	p2.x = pc.x + lambda2*ncd[0];
	p2.y = pc.y + lambda2*ncd[1];
	p2.z = pc.z + lambda2*ncd[2];

	p_inter.x = (p1.x + p2.x)/2.0f;
	p_inter.y = (p1.y + p2.y)/2.0f;
	p_inter.z = (p1.z + p2.z)/2.0f;
	
	// �жϽ����Ƿ����߶�ab���߶�cd�ڲ�
	// line segment ab
	float dist_pa, dist_pb, dist_ab, 
		  dist_pc, dist_pd, dist_cd;
	dist_pa = distP2P(p_inter, pa);
	dist_pb = distP2P(p_inter, pb);
	dist_ab = distP2P(pa, pb);
	dist_pc = distP2P(p_inter, pc);
	dist_pd = distP2P(p_inter, pd);
	dist_cd = distP2P(pc, pd);
	if( abs(dist_pa+dist_pb-dist_ab) < 0.001f &&
		abs(dist_pc+dist_pd-dist_cd) < 0.001f )
	{
		return true;
	}
	return false;
}

// ��ȡ�㵽ƽ��ľ�����ͶӰ
void getInfoBetPointAndPlane(pcl::PointXYZ &p, Eigen::Vector4f &plane_param, float &dist, pcl::PointXYZ &p_proj)
{
	projPoint2Plane(p, p_proj, plane_param);
	dist = distP2P(p, p_proj);
}
// �㵽ֱ�ߵľ��뺯��
void distP2L(pcl::PointXYZ &p, pcl::PointXYZ &p_base, Eigen::Vector3f &n, pcl::PointXYZ &p_projOnLine, float &dist)
{
	float delta_x = p_base.x - p.x;
	float delta_y = p_base.y - p.y;
	float delta_z = p_base.z - p.z;
	float lambda = -1.0*(n[0]*delta_x + n[1]*delta_y + n[2]*delta_z);
	p_projOnLine.x = p_base.x + lambda*n[0];
	p_projOnLine.y = p_base.y + lambda*n[1];
	p_projOnLine.z = p_base.z + lambda*n[2];
	dist = distP2P(p, p_projOnLine);
}
///////////////////////////////////////////////////////////////////////////////
int id_sink = 0;
int id_plane = 0;
int growth_unit_id = 0;
// ��ѡȡ�¼��ص�����
int point_index = -1;
// ����α༭��
pcl::PointXYZ g_selected_point;
int g_selected_poly_id = -1;
PointCloudT::Ptr poly_centroids_cloud (new PointCloudT);
pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_poly_centroids_cloud;
std::vector<bool> g_is_poly_del;
// ������޼���
pcl::PointXYZ first_point, second_point;
void pointPickingEventOccurred ( const pcl::visualization::PointPickingEvent &event,
								 void* viewer_void)
{
	if(strcmp(state, "estimate normal") == 0 ||
		strcmp(state, "regulate normal") == 0)
	{
		selected_point_index = event.getPointIndex();
		cout << "selected point index = " << selected_point_index << endl;
		cout << "open config.txt, and set the value of is_norm_direction_valid" << endl;

		PointCloudT::Ptr selected_point (new PointCloudT);
		selected_point->push_back(source_cloud->points[selected_point_index]);
		pcl::PointCloud<pcl::Normal>::Ptr selected_normal (new pcl::PointCloud<pcl::Normal>);
		selected_normal->push_back(source_normal->points[selected_point_index]);

		viewer_cloud.removePointCloud("selected normal");
		viewer_cloud.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>
			(selected_point, selected_normal,1,normal_length_for_selected,"selected normal");

        point_index = event.getPointIndex();
        PointCloudT::Ptr point (new PointCloudT);
        point->push_back(source_cloud->points[point_index]);
        viewer_cloud.removePointCloud("point");
        viewer_cloud.addPointCloud(point, "point");
        viewer_cloud.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 255, 0, 0, "point");
        viewer_cloud.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "point");
	}	

	if(strcmp(state, "segment planes") == 0)
	{
		cout << "point_index = " << point_index << endl;
		cout << "growth_unit_id = " << growth_unit_id << endl;
		
		PointCloudT::Ptr plane (new PointCloudT);
		plane = plane_clouds[growth_unit_id].points_set;

		std::vector<int> indices;
		std::vector<int> knn_indices;
		std::vector<float> knn_dist;

		kdtree_source.nearestKSearch(plane->points[0], 1, indices, knn_dist);

		Eigen::Vector3f point_normal;
		point_normal << source_normal->points[indices[0]].normal_x,
						source_normal->points[indices[0]].normal_y,
						source_normal->points[indices[0]].normal_z;
		Eigen::Vector3f growth_unit_normal, local_point_normal;

		// ���ȼ�������ƽ��ķ�����
		Eigen::Vector4f plane_param;
		float curvature;
		for(int i = 0; i < plane->size(); ++i)
		{
			kdtree_source.nearestKSearch(plane->points[i], 1, knn_indices, knn_dist);
			indices.push_back(knn_indices[0]);
		}
		pcl::computePointNormal(*source_cloud, indices, plane_param, curvature);
		growth_unit_normal << plane_param[0], plane_param[1], plane_param[2];
		if(growth_unit_normal.dot(point_normal) < 0)
			growth_unit_normal *= -1.0f;
		indices.clear();
		// r_localȻ�����ֲ�������
		kdtree_source.radiusSearch(source_cloud->points[point_index], r_local, indices, knn_dist);
		// cout << "indices.size() = " << indices.size() << endl;
		pcl::computePointNormal(*source_cloud, indices, plane_param, curvature);
		local_point_normal << plane_param[0], plane_param[1], plane_param[2];
		Eigen::Vector3f source_point_normal;
		source_point_normal << source_normal->points[point_index].normal_x,
								source_normal->points[point_index].normal_y,
								source_normal->points[point_index].normal_z;
		if(local_point_normal.dot(source_point_normal) < 0)
			local_point_normal *= -1.0f;

		float angle = arccos(growth_unit_normal.dot(local_point_normal)) * 180.0f / PI;
		cout << "angle = " << angle << endl;

		viewer_cloud.removeShape("sphere");
		viewer_cloud.addSphere(source_cloud->points[point_index], r_local, 255,0,0,"sphere");

		// ��ʾ������Ԫ�ķ�����
		pcl::PointXYZ p;
		p.x = p.y = p.z = 0;
		for(int i = 0; i < plane->size(); ++i)
		{
			p.x += plane->points[i].x;
			p.y += plane->points[i].y;
			p.z += plane->points[i].z;
		}
		p.x /= float(plane->size());
		p.y /= float(plane->size());
		p.z /= float(plane->size());
		kdtree_source.nearestKSearch(p, 1, indices, knn_dist);
		p = source_cloud->points[indices[0]];

		pcl::Normal pn;
		pn.normal_x = growth_unit_normal[0];
		pn.normal_y = growth_unit_normal[1];
		pn.normal_z = growth_unit_normal[2];

		PointCloudT::Ptr cloud1 (new PointCloudT);
		cloud1->push_back(p);
		pcl::PointCloud<pcl::Normal>::Ptr normal1 (new pcl::PointCloud<pcl::Normal>);
		normal1->push_back(pn);
		viewer_cloud.removePointCloud("growth_unit_normal");
		viewer_cloud.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>
			(cloud1, normal1,1,normal_length_for_selected,"growth_unit_normal");
		// ��ʾѡ�е�ľֲ�������
		p = source_cloud->points[point_index];
		pn.normal_x = local_point_normal[0];
		pn.normal_y = local_point_normal[1];
		pn.normal_z = local_point_normal[2];
		PointCloudT::Ptr cloud2 (new PointCloudT);
		cloud2->push_back(p);
		pcl::PointCloud<pcl::Normal>::Ptr normal2 (new pcl::PointCloud<pcl::Normal>);
		normal2->push_back(pn);
		viewer_cloud.removePointCloud("local_normal");
		viewer_cloud.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>
			(cloud2, normal2,1,normal_length_for_selected/2.0,"local_normal");
	}

    if(strcmp(state, "del poly") == 0 || strcmp(state, "prune poly") ==0)
    {
        if(plane_clouds_final.size()==0) return;

        QString cloud_id = QString::number(g_selected_poly_id, 10);
        if(!g_is_poly_del[g_selected_poly_id])
        {
            // ��֮ǰѡȡ�Ķ������ɫ�ظ�
            viewer_cloud.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 255, 255, 255, cloud_id.toStdString());
            viewer_cloud.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloud_id.toStdString());
        }

        event.getPoint(g_selected_point.x, g_selected_point.y, g_selected_point.z);
        // cout << "g_selected_point = " << g_selected_point << endl;

        // ��ȡ��ѡ��Ķ���ε�id
        std::vector<int> indices;
        std::vector<float> dists;
        kdtree_poly_centroids_cloud.nearestKSearch(g_selected_point, 1, indices, dists);
        g_selected_poly_id = indices[0];

        // ��ʾ�Ѿ�ѡȡ�ĵ�
        PointCloudT::Ptr point (new PointCloudT);
        point->push_back(poly_centroids_cloud->points[g_selected_poly_id]);
        viewer_cloud.removePointCloud("point");
        viewer_cloud.addPointCloud(point, "point");
        viewer_cloud.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 255, 0, 0, "point");
        viewer_cloud.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "point");

        // ����ɫ��ʾ��ѡ�еĶ����
        cloud_id = QString::number(g_selected_poly_id, 10);
        viewer_cloud.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 255, 0, cloud_id.toStdString());
        viewer_cloud.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, cloud_id.toStdString());
    }

    if(strcmp(state, "prune poly select points") == 0)
    {
        event.getPoint(g_selected_point.x, g_selected_point.y, g_selected_point.z);
        // ��ʾ�Ѿ�ѡȡ�ĵ�
        PointCloudT::Ptr point (new PointCloudT);
        point->push_back(g_selected_point);
        viewer_cloud.removePointCloud("point");
        viewer_cloud.addPointCloud(point, "point");
        viewer_cloud.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 255, 0, 0, "point");
        viewer_cloud.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "point");
    }
}
void areaPickingEventOccurred ( const pcl::visualization::AreaPickingEvent &event,
								 void* viewer_void)
{
	std::vector<int> indices;
	event.getPointsIndices(indices);
	if(indices.size() == 0)
	{
		cout << "areaPickingEventOccurred: indices.size() == 0" << endl;
		return;
	}
	if(strcmp(state, "test search_radius") == 0)
	{		
		point_index = indices[0];
		cout << "point_index = " << point_index << endl;
		viewer_cloud.removeAllShapes();
		viewer_cloud.addSphere(source_cloud->points[point_index], r_for_estimate_normal, 0, 255, 0, "sphere");
		viewer_cloud.removePointCloud("point");
		PointCloudT::Ptr point (new PointCloudT);
		point->push_back(source_cloud->points[point_index]);
		viewer_cloud.addPointCloud(point, "point");
		viewer_cloud.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 255, 0, 0, "point");
		viewer_cloud.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "point");
	}

	if(strcmp(state, "estimate normal") == 0 ||
		strcmp(state, "regulate normal") == 0)
	{
		selected_point_index = indices[0];
		cout << "selected point index = " << selected_point_index << endl;
		cout << "open config.txt, and set the value of is_norm_direction_valid" << endl;

		PointCloudT::Ptr selected_point (new PointCloudT);
		selected_point->push_back(source_cloud->points[selected_point_index]);
		pcl::PointCloud<pcl::Normal>::Ptr selected_normal (new pcl::PointCloud<pcl::Normal>);
		selected_normal->push_back(source_normal->points[selected_point_index]);

		viewer_cloud.removePointCloud("selected normal");
		viewer_cloud.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>
			(selected_point, selected_normal,1,normal_length_for_selected,"selected normal");

		
		point_index = indices[0];
		PointCloudT::Ptr point (new PointCloudT);
		point->push_back(source_cloud->points[point_index]);
		viewer_cloud.removePointCloud("point");
		viewer_cloud.addPointCloud(point, "point");
		viewer_cloud.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 255, 0, 0, "point");
		viewer_cloud.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "point");
	}
}

//int id_sink = 0;
//int id_plane = 0;
// �����¼�������
void keyboardEventOccurred_ps (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void)
{
	if(event.getKeySym() == "Return" && event.keyDown())
	{
		if(strcmp(state,"segment ps")==0)
		{
			// �Բ����ռ�
			cout << "id_sink = " << id_sink << endl;
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
			int sink_index = sinkPointsSet[id_sink].index;
			for(int i = 0; i < ps_cloud->size(); ++i)
			{
				if(param_space[i].sink == sink_index)
				{
					pcl::PointXYZRGB p = ps_cloud->points[i];
					p.r = p.g = p.b = 255;
					cloud->push_back(p);
				}
			}
			viewer_ps.removePointCloud("sink field");
			viewer_ps.addPointCloud(cloud, "sink field");
			cout << "ps points size = " << cloud->size() << endl;

			// �Բ����ռ�
			PointCloudT::Ptr plane (new PointCloudT);
			for(int i = 0; i < ps_cloud->size(); ++i)
			{
				if(param_space[i].sink == sink_index)
				{
					for(int j = 0; j < param_space[i].point_indices.size(); ++j)
					{
						plane->push_back(source_cloud->points[param_space[i].point_indices[j]]);
					}
				}
			}
			viewer_cloud.removePointCloud("plane");
			viewer_cloud.addPointCloud(plane, "plane");
			viewer_cloud.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 255, 0, "plane");
			cout << "plane has " << plane->size() << " point.s" << endl;

			id_sink = ++id_sink % sinkPointsSet.size();
		}
	}

	if(event.getKeySym() == "F1" && event.keyDown())
	{
		viewer_ps.removePointCloud("sink field");
		viewer_cloud.removePointCloud("plane");
		id_sink = id_plane = 0;
	}
}

// �����¼�������
pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> color_handler(colored_cloud);
void keyboardEventOccurred_cloud (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void)
{
	if(event.getKeySym() == "Return" && event.keyDown())
	{
		if(strcmp(state,"segment planes")==0 /*|| strcmp(state,"grow planes")==0*/)
		{
			cout << "id_plane = " << id_plane << endl;
			PointCloudT::Ptr plane (new PointCloudT);
			plane = plane_clouds[id_plane].points_set;
			viewer_cloud.removePointCloud("plane");
			viewer_cloud.addPointCloud(plane, "plane");
			viewer_cloud.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 255, 0, "plane");
			cout << "plane has " << plane->size() << " point.s" << endl;
			growth_unit_id = id_plane;
			id_plane = ++id_plane % plane_clouds.size();
		}

		if(strcmp(state,"grow planes")==0)
		{
			cout << "id_plane = " << id_plane << endl;
			PointCloudT::Ptr growth_unit (new PointCloudT);
			PointCloudT::Ptr plane (new PointCloudT);
			viewer_cloud.removePointCloud("plane");
			viewer_cloud.removePointCloud("growth_unit");
			growth_unit = growth_unit_set[id_plane].points_set;
			plane = plane_clouds[id_plane].points_set;
			viewer_cloud.addPointCloud(plane,"plane");
			viewer_cloud.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 255, 0, "plane");
			viewer_cloud.addPointCloud(growth_unit,"growth_unit");
			viewer_cloud.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 255, 0, 0, "growth_unit");
			cout << "plane has " << plane->size() << " point.s" << endl;
			cout << "growth unit has " << growth_unit->size() << " point.s" << endl;
			id_plane = ++id_plane % plane_clouds.size();
		}

		if(strcmp(state,"merge planes")==0 || strcmp(state,"do")==0)
		{
			cout << "id_plane = " << id_plane << endl;
			PointCloudT::Ptr plane (new PointCloudT);
			plane = plane_clouds_final[id_plane].points_set;
			viewer_cloud.removePointCloud("plane");
			viewer_cloud.addPointCloud(plane, "plane");
			viewer_cloud.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 255, 0, "plane");
			cout << "plane has " << plane->size() << " point.s" << endl;
			id_plane = ++id_plane % plane_clouds_final.size();
		}

		/*if(strcmp(state,"poly planes")==0)
		{
			viewer_cloud.removeAllShapes();
			cout << "id_plane = " << id_plane << endl;
			PointCloudT::Ptr border (new PointCloudT);
			border = plane_clouds_final[id_plane].border;
			for(int i = 0; i < border->size()-1;++i)
			{
				std::stringstream ss;
				ss << i;
				viewer_cloud.addLine(border->points[i], border->points[i+1], 255, 255, 0, ss.str());
			}
			std::stringstream ss;
			ss << border->size()-1;
			viewer_cloud.addLine(border->points[border->size()-1], border->points[0], 255, 255, 0, ss.str());
			id_plane = ++id_plane % plane_clouds_final.size();
		}*/

		if(strcmp(state,"postProcess planes") == 0)
		{			
			id_plane = ++id_plane % plane_clouds_final.size();
		}
	}

	if(event.getKeySym() == "F1" && event.keyDown())
	{
		viewer_cloud.removePointCloud("source");
	}
	if(event.getKeySym() == "F2" && event.keyDown())
	{
		viewer_cloud.removePointCloud("source");
		viewer_cloud.addPointCloud(source_cloud, "source");
	}
	if(event.getKeySym() == "F3" && event.keyDown())
	{
		viewer_cloud.removeAllShapes();
		pcl::PolygonMesh polygons;
		PointCloudT vertices;
		for(int ii = 0; ii < plane_clouds_final.size(); ++ii)
		{
			Plane plane = plane_clouds_final[ii];
			for(int i = 0; i < plane.border->size(); ++i)
			{
				vertices.push_back(plane.border->points[i]);
			}
		}
		pcl::toPCLPointCloud2(vertices, polygons.cloud);
		int base_index = 0;
		for(int ii = 0; ii < plane_clouds_final.size(); ++ii)
		{
			pcl::Vertices polygon;
			int num_of_vertices = plane_clouds_final[ii].border->size();
			for(int i = base_index; i < base_index+num_of_vertices; ++i)
			{
				polygon.vertices.push_back(i);
			}
			polygon.vertices.push_back(base_index);
			polygons.polygons.push_back(polygon);
			base_index += num_of_vertices;
		}

		viewer_cloud.addPolylineFromPolygonMesh(polygons, "polyline");
		/*for(int ii = 0; ii < plane_clouds_final.size(); ++ii)
		{
			Plane plane = plane_clouds_final[ii];
			std::vector<pcl::Vertices> vertices;
			pcl::Vertices v;
			for(int i = 0; i < plane.border->size(); ++i)
			{
				v.vertices.push_back(i);
			}
			v.vertices.push_back(0);
			vertices.push_back(v);
			pcl::PolygonMesh pm;
			pm.polygons = vertices;
			pcl::toPCLPointCloud2(*plane.border, pm.cloud);
			std::stringstream ss;
			ss << ii;
			viewer_cloud.addPolylineFromPolygonMesh(pm,ss.str());
		}*/
	}
	if(event.getKeySym() == "F4" && event.keyDown())
	{
		viewer_cloud.removeAllShapes();
		viewer_cloud.removePointCloud("point");
	}
	if(event.getKeySym() == "F5" && event.keyDown())
	{
		viewer_cloud.removePointCloud("source backup");
		viewer_cloud.addPointCloud(source_cloud_backup, "source backup");
	}
	if(event.getKeySym() == "F6" && event.keyDown())
	{
		viewer_cloud.removePointCloud("source backup");
	}
	if(event.getKeySym() == "F7" && event.keyDown())
	{
		viewer_cloud.removeAllShapes();
		for(int ii = 0; ii < plane_clouds_final.size(); ++ii)
		{
			Plane plane = plane_clouds_final[ii];
			std::stringstream ss;
			ss << "poly" << ii;
			viewer_cloud.addPolygon<pcl::PointXYZ>(plane.border,ss.str());
		}
	}

	// colored_cloud
	if(event.getKeySym() == "F8" && event.keyDown())
	{
		viewer_cloud.removeAllPointClouds();
		viewer_cloud.removeAllShapes();		
		colored_cloud->clear();
		pcl::PointXYZRGB p;
		int r,g,b;
		for(int i = 0; i < plane_clouds_final.size(); ++i)
		{
			std::stringstream ss;
			ss << "c" << i;
			r = rand()%256;
			g = rand()%256;
			b = rand()%256;
			for(int j = 0; j < plane_clouds_final[i].points_set->size(); ++j)
			{
				p.x = plane_clouds_final[i].points_set->points[j].x;
				p.y = plane_clouds_final[i].points_set->points[j].y;
				p.z = plane_clouds_final[i].points_set->points[j].z;
				p.r = r;
				p.g = g;
				p.b = b;
				colored_cloud->push_back(p);
			}
			/*viewer_cloud.addPointCloud(plane_clouds_final[i].points_set,ss.str());
			viewer_cloud.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, rand()%256, rand()%256, rand()%256, ss.str());*/
		}
		color_handler.setInputCloud(colored_cloud);
		viewer_cloud.addPointCloud<pcl::PointXYZRGB>(colored_cloud, color_handler, "colored");
		cout << "rendering done." << endl;
	}
	if(event.getKeySym() == "F9" && event.keyDown())
	{
		viewer_cloud.removeAllPointClouds();
		viewer_cloud.removeAllShapes();
		cout << "elimination done." << endl;
	}
	if(event.getKeySym() == "F10" && event.keyDown())
	{
		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
		kdtree.setInputCloud(source_cloud_backup);
		bool *isProcessed = new bool[source_cloud_backup->size()];
		memset(isProcessed, 0, sizeof(bool)*source_cloud_backup->size());
		std::vector<int> indices;
		std::vector<float> dist;

		for(int ii = 0; ii < plane_clouds_final.size(); ++ii)
		{
			Plane plane = plane_clouds_final[ii];
			for(int i = 0; i < plane.points_set->size(); ++i)
			{
				kdtree.nearestKSearch(plane.points_set->points[i],1,indices, dist);
				isProcessed[indices[0]] = true;
			}
		}
		int nCount = 0;
		for(int i = 0; i < source_cloud_backup->size(); ++i)
		{
			if(isProcessed[i])
				++nCount;
		}
		cout << "source cloud points number = " << source_cloud_backup->size() << endl;
		cout << "plane points number = " << nCount << endl;
	}

	if(event.getKeySym() == "z" && event.keyDown())
	{
		cout << "point_index = " << point_index << endl;
		Plane plane = plane_clouds_final[0];
		if(isPointInPlane(source_cloud->points[point_index], plane))
		{
			cout << "ok" << endl;
		}else
			cout << "not ok" << endl;
	}
}
// ��Ϣ������
void processStateMsg()
{
	if(strcmp(state, "create ps") == 0)
	{
		// ��ʾ�����ռ�
		int vote_max = INT_MIN;
		int vote_min = INT_MAX;
		for(int i = 0; i < ps_cloud->size(); ++i)
		{
			if(param_space[i].vote > 0)
			{
				if(param_space[i].vote > vote_max)
					vote_max = param_space[i].vote;
				if(param_space[i].vote < vote_min)
					vote_min = param_space[i].vote;
			}
		}
		cout << "vote_max = " << vote_max << endl;
		cout << "vote_min = " << vote_min << endl;
		for(int i = 0; i < ps_cloud->size(); ++i)
		{
			if(param_space[i].vote == 0)
			{
				ps_cloud->points[i].r = ps_cloud->points[i].g = ps_cloud->points[i].b = 0;
			}else{
				float ratio = (float)param_space[i].vote / (float)vote_max;
				ps_cloud->points[i].r = 0;
				ps_cloud->points[i].g = color_gain*ratio*255.0 > 255 ? 255 : color_gain*ratio*255.0;
				ps_cloud->points[i].b = 0;
			}
		}
		viewer_ps.removePointCloud("ps cloud");
		viewer_ps.addPointCloud(ps_cloud, "ps cloud");
		memset(state, 0, CMD_SIZE);
	}

	if(strcmp(state, "clear normal") == 0)
	{
		viewer_cloud.removePointCloud("normal");
		viewer_cloud.removePointCloud("processed");
		viewer_cloud.removePointCloud("point");
		memset(state, 0, CMD_SIZE);
	}

	if(strcmp(state, "filt ps") == 0)
	{
		// ��ʾ�����ռ�
		int vote_max = INT_MIN;
		int vote_min = INT_MAX;
		for(int i = 0; i < ps_cloud->size(); ++i)
		{
			if(param_space[i].vote > 0)
			{
				if(param_space[i].vote > vote_max)
					vote_max = param_space[i].vote;
				if(param_space[i].vote < vote_min)
					vote_min = param_space[i].vote;
			}
		}
		cout << "after filted:" << endl;
		cout << "vote_max = " << vote_max << endl;
		cout << "vote_min = " << vote_min << endl;
		for(int i = 0; i < ps_cloud->size(); ++i)
		{
			if(param_space[i].vote == 0)
			{
				ps_cloud->points[i].r = ps_cloud->points[i].g = ps_cloud->points[i].b = 0;
			}else{
				float ratio = (float)param_space[i].vote / (float)vote_max;
				ps_cloud->points[i].r = 0;
				ps_cloud->points[i].g = color_gain*ratio*255.0 > 255 ? 255 : color_gain*ratio*255.0;
				ps_cloud->points[i].b = 0;
			}
		}
		viewer_ps.removePointCloud("ps cloud");
		viewer_ps.addPointCloud(ps_cloud, "ps cloud");
		memset(state, 0, CMD_SIZE);
	}

	if(strcmp(state, "run again") == 0)
	{
		viewer_cloud.removeAllPointClouds();
		viewer_ps.removeAllPointClouds();

		viewer_cloud.addPointCloud(source_cloud, "source");

		memset(state, 0, CMD_SIZE);
	}
	if(strcmp(state, "postProcess planes") == 0)
	{
		viewer_cloud.removeAllPointClouds();
		viewer_cloud.addPointCloud(source_cloud, "source");
		memset(state, 0, CMD_SIZE);
	}

	if(strcmp(state, "regulate normal") == 0)
	{
		start = std::clock();
		regulateNormal();
		memset(state, 0, CMD_SIZE);
		strcpy(state, "estimate normal");
		finish = std::clock();
		cout << "time used: " << finish - start << "ms." << endl;
	}

	if(strcmp(state, "load polys") == 0)
	{
		viewer_cloud.removeAllShapes();
		for(int ii = 0; ii < plane_clouds_final.size(); ++ii)
		{
			Plane plane = plane_clouds_final[ii];
			std::vector<pcl::Vertices> vertices;
			pcl::Vertices v;
			for(int i = 0; i < plane.border->size(); ++i)
			{
				v.vertices.push_back(i);
			}
			v.vertices.push_back(0);
			vertices.push_back(v);
			pcl::PolygonMesh pm;
			pm.polygons = vertices;
			pcl::toPCLPointCloud2(*plane.border, pm.cloud);
			std::stringstream ss;
			ss << ii;
			viewer_cloud.addPolylineFromPolygonMesh(pm,ss.str());
		}
		memset(state, 0, CMD_SIZE);
	}
}

#endif  //PLANEDETECT_H