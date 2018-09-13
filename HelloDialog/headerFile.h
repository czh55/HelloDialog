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


typedef pcl::PointXYZRGB PointT;
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