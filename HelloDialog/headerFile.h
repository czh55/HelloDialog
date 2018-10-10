#ifndef HEADFILE_H
#define HEADFILE_H

#include <iostream>
#include <pcl\io\pcd_io.h>
#include <pcl\io\io.h>
#include <pcl\point_types.h>
#include <pcl\kdtree\kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <string>
#include <boost\lexical_cast.hpp>
#include <boost\format.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/config.hpp>
#include <stdlib.h>
#include <time.h>
#include <algorithm>
#include <pcl/features/normal_3d.h>
#include <queue>
#include <utility>
#include <pcl\visualization\pcl_plotter.h>
#include <pcl\surface\concave_hull.h>
#include <pcl\filters\filter.h>
#include <pcl\surface\ear_clipping.h>
#include <pcl\surface\vtk_smoothing\vtk_utils.h>
#include <pcl/visualization/common/actor_map.h>


//define by czh  ����sac��ICP����׼
#include <pcl/registration/icp.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ia_ransac.h>
//define by czh ���ڻ���ƽ�����׼
#include <vector>
#include <pcl/registration/transformation_estimation_svd.h>
//define by czh
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

using namespace std;

/******************************************PlaneDetect.h��Registration.h���ò��֣�Ϊ����ʹ��polygon_transform()******************************************************************/
// data structures
struct PSElem
{
	std::vector<int> point_indices;
	int vote;
	int sink_init;	// create gradient field
	int sink;		// rectify sink points
};

struct SinkPoint
{
	int index;		// �����ռ��е�����
	int vote_num;	// total numbers voted for this sink point
};

struct Triangle
{
	pcl::PointXYZ p_a;
	pcl::PointXYZ p_b;
	pcl::PointXYZ p_c;
};

struct Plane
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	PointCloudT::Ptr border;		// ƽ��ı߽��
	PointCloudT::Ptr points_set;	// ƽ��㼯
	pcl::ModelCoefficients coeff;					// ƽ�������������ָ�������ⲿ
	std::vector<Triangle, Eigen::aligned_allocator<Triangle>> triangles;	// ����ε���������Ƭ
};

// ���յ�ƽ�漯��
std::vector<Plane, Eigen::aligned_allocator<Plane>> plane_clouds_final;
/************************************************************************************************************************************/
// viewer instances
pcl::visualization::PCLVisualizer viewer_ps;		// display param space
pcl::visualization::PCLVisualizer viewer_cloud;		// display point clouds

//typedef pcl::PointXYZRGB PointT;
const int ARGU_BUFFER_SIZE = 256;		// ������������С
extern char g_argu_buffer[ARGU_BUFFER_SIZE];	// ����������

char* getParameterFromConfigFile(const char *itemName)
{	
	memset(g_argu_buffer,0,ARGU_BUFFER_SIZE*sizeof(char));	// ��ջ�����
	// ���ȴ������ļ�
	std::ifstream file("config.txt", std::ios::in);
	if(!file){std::cout << "cannnot find config file!" << std::endl; return NULL;}
	while(file.getline(g_argu_buffer,ARGU_BUFFER_SIZE-1))	// ����'\n'�����BUFSIZE-1�ֽ���ֹ
	{		
		if(g_argu_buffer[0] == '#')	// ��ע��
			continue;

		if(memcmp(g_argu_buffer,itemName,strlen(itemName)) == 0)
		{
			if(g_argu_buffer[strlen(itemName)] != ' ')
				continue;

			int len1 = strlen(g_argu_buffer);
			int len2 = strlen(itemName) + 3;
			int i;
			for(i = 0; i < len1 - len2; ++i)
				g_argu_buffer[i] = g_argu_buffer[i+len2];
			for(;i<len1; ++i)
				g_argu_buffer[i] = 0;
			return g_argu_buffer;
		}
		memset(g_argu_buffer,0,ARGU_BUFFER_SIZE*sizeof(char));	// ��ջ�����
	}
	return NULL;
}

#endif  //HEADFILE_H