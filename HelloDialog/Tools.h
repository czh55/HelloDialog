#ifndef TOOLS_H
#define TOOLS_H

#include "PlaneDetect.h"

class Tools
{
public:
	Tools();
	~Tools();
public:
	//Executing the transformation
	void transformation( Eigen::Matrix4d transformation_matrix);
	//void transformation(const PointCloudT &souce_cloud, PointCloudT &icp_cloud, Eigen::Matrix4d transformation_matrix);

	void print4x4Matrix(const Eigen::Matrix4d & matrix);

	void visualization();
	//void visualization(PointCloudT::Ptr source_cloud1_registration, PointCloudT::Ptr source_cloud2_registration, PointCloudT::Ptr cloud_tr, PointCloudT::Ptr cloud_icp, int iterations);

	void savePointCloudFile();
	//void savePointCloudFile(PointCloudT::Ptr source_cloud1_registration, PointCloudT::Ptr cloud_icp, int iterations);
};


#endif //TOOLS_H