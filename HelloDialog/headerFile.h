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


//define by czh  用于sac和ICP的配准
#include <pcl/registration/icp.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ia_ransac.h>
//define by czh 用于基于平面的配准
#include <vector>
#include <pcl/registration/transformation_estimation_svd.h>
//define by czh
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

using namespace std;

/******************************************PlaneDetect.h和Registration.h公用部分：为了能使用polygon_transform()******************************************************************/
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
	int index;		// 参数空间中的索引
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
	PointCloudT::Ptr border;		// 平面的边界点
	PointCloudT::Ptr points_set;	// 平面点集
	pcl::ModelCoefficients coeff;					// 平面参数，法向量指向物体外部
	std::vector<Triangle, Eigen::aligned_allocator<Triangle>> triangles;	// 多边形的三角形面片
};

// 最终的平面集合
std::vector<Plane, Eigen::aligned_allocator<Plane>> plane_clouds_final;
/************************************************************************************************************************************/
// viewer instances
pcl::visualization::PCLVisualizer viewer_ps;		// display param space
pcl::visualization::PCLVisualizer viewer_cloud;		// display point clouds

//typedef pcl::PointXYZRGB PointT;
const int ARGU_BUFFER_SIZE = 256;		// 参数缓冲区大小
extern char g_argu_buffer[ARGU_BUFFER_SIZE];	// 参数缓冲区

char* getParameterFromConfigFile(const char *itemName)
{	
	memset(g_argu_buffer,0,ARGU_BUFFER_SIZE*sizeof(char));	// 清空缓冲区
	// 首先打开配置文件
	std::ifstream file("config.txt", std::ios::in);
	if(!file){std::cout << "cannnot find config file!" << std::endl; return NULL;}
	while(file.getline(g_argu_buffer,ARGU_BUFFER_SIZE-1))	// 遇到'\n'或读满BUFSIZE-1字节终止
	{		
		if(g_argu_buffer[0] == '#')	// 是注释
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
		memset(g_argu_buffer,0,ARGU_BUFFER_SIZE*sizeof(char));	// 清空缓冲区
	}
	return NULL;
}

#endif  //HEADFILE_H