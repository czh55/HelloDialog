#pragma once
//define by czh
#ifndef TRIANGULARMESHING_H
#define TRIANGULARMESHING_H

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <pcl/io/PLY_io.h>

/*����PCL�ĵ������񻯣������������.pcd��ʽ������������ǣ�.ply��ʽ
���룺cloud��Ҫ�������񻯵ĵ�������
�������أ�result_mesh 
*/
void triangularMeshing(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PolygonMesh &result_mesh)
{
	cout << "��ʼ�������񻯣�" << endl;
	// Normal estimation�����������ƣ�
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(20);
	n.compute(*normals); //����
	//* normals should not contain the point normals + surface curvatures������ͬʱ������ķ������ͱ�������ʣ�
	// Concatenate the XYZ and normal fields �������ƺͷ��߷���һ��
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);
	// Initialize objects ����ʼ������
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	//����������������ڴ洢���
	pcl::PolygonMesh triangles;
	//���ò���
	gp3.setSearchRadius(1.5); // �������ӵ�֮��������루���߳�������ȷ��k���ڵ���뾶��Ĭ��Ϊ0��
	gp3.setMu(2.5); // ��������ھ���ĳ��ӣ��ѵõ�ÿ��������������뾶��Ĭ��Ϊ0��
	gp3.setMaximumNearestNeighbors(100); //��������������ڵ���������
	gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees ���ƽ���
	gp3.setMinimumAngle(M_PI / 18); // 10 degrees ÿ�����ǵ����Ƕ�
	gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
	gp3.setNormalConsistency(false); //��������һ�£���Ϊtrue
	// ���������������������
	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);
	//ִ���ع������������triangles��
	gp3.reconstruct(triangles);
	//��������ͼ
	//pcl::io::saveVTKFile("mymesh.vtk", triangles); //������û��
	//pcl::io::savePLYFile("result_mesh/result_mesh.ply", triangles);
	//���ص���
	result_mesh = triangles;

	/*//Additional vertex information
	std::vector<int> parts = gp3.getPartIDs();
	std::vector<int> states = gp3.getPointStates();
	fstream fs;
	fs.open("partsID.txt", ios::out);
	if (!fs)
	{
		return ;
	}
	fs << "��������Ϊ��" << parts.size() << "\n";
	for (int i = 0; i < parts.size(); i++)
	{
		if (parts[i] != 0)
		{
			fs << parts[i] << "\n"; //���fs����
		}
	}*/
	//��ʾ���ͼ
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0); //���ñ���
	viewer->addPolygonMesh(triangles, "my"); //������ʾ������
	viewer->addCoordinateSystem(1.0); //��������ϵ
	viewer->initCameraParameters();
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	// Finish
	cout << "�������񻯽���" << endl;
	return;
}

#endif // !TRIANGULARMESHING_H

