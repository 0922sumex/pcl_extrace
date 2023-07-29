#pragma once
#include "plane_set.h"
#include "plfh_solver.h"
#include <random>
#include<pcl/visualization/pcl_plotter.h>

pcl::RGB generateRandomColor()
{
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_int_distribution<> dis(0, 255);

	pcl::RGB color;
	color.r = static_cast<uint8_t>(dis(gen));
	color.g = static_cast<uint8_t>(dis(gen));
	color.b = static_cast<uint8_t>(dis(gen));

	return color;
}

void LoadClouds(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("acmh_model.pcd", *cloud) == -1) //* load the file
	{
		PCL_ERROR("Couldn't read file test_pcd.pcd \n");
		exit(-1);
	}
	std::cout << "Loaded "
		<< cloud->width * cloud->height
		<< " data points from test_pcd.pcd with the following fields: "
		<< std::endl;
}

pcl::PointXYZ RandomSelectSingleCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
	int j = 0;
	pcl::PointXYZ point_single;
	point_single.x = cloud->points[j].x;
	point_single.y = cloud->points[j].y;
	point_single.z = cloud->points[j].z;//随机选取一个点
	return point_single;
}

void Viewer_Results_clouds(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, plane_set PlaneSet, cylinder_set CylinderSet) {
	vector<plane> cloud_plane_S = PlaneSet.getPlaneSet();
	vector<cylinder> cloud_cylinder_S = CylinderSet.getCylinderSet();

	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("��Ͻ��"));
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");

	cout << "平面个数为：" << PlaneSet.getPlaneNumber() << endl;
	cout << "圆柱个数为：" << CylinderSet.getCylinderNumber() << endl;

	///*
	for (int i = 0; i < PlaneSet.getPlaneNumber(); i++) {
		//生成随机颜色
		pcl::RGB color1 = generateRandomColor();
		cout << "输出第" << i<<"个平面的点云" << endl;

		for (auto& point : cloud_plane_S[i].cloud_plane.points)
		{
			point.r = color1.r;
			point.g = color1.g;
			point.b = color1.b;
		}
		viewer->addPointCloud<pcl::PointXYZRGB>(cloud_plane_S[i].cloud_plane.makeShared(), std::to_string(i));
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, std::to_string(i));
	}
	/*while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
	}*/
	///*
	for (int i = 0; i < CylinderSet.getCylinderNumber(); i++) {
		//生成随机颜色
		pcl::RGB color1 = generateRandomColor();
		cout << "输出第" << i << "个圆柱的点云" << endl;

		for (auto& point : cloud_cylinder_S[i].cloud_cylinder.points)
		{
			point.r = color1.r;
			point.g = color1.g;
			point.b = color1.b;
		}
		viewer->addPointCloud<pcl::PointXYZRGB>(cloud_cylinder_S[i].cloud_cylinder.makeShared(), std::to_string(i+10));
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, std::to_string(i+10));
	}
	//*/
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
	}
}

void Viewer_Plotter(vector<pcl::PLFH_gather> plfh_connected_set) {
	pcl::visualization::PCLPlotter* plotter = new pcl::visualization::PCLPlotter();
	int length = plfh_connected_set[0].features_histogram.size();

	std::vector<double> array_x(length), array_y(length);
	for (int i = 0; i < length; ++i)
	{
		array_x[i] = i;
		array_y[i] = plfh_connected_set[0].features_histogram[i];
	}
	plotter->addPlotData(array_x, array_y, "plfh", vtkChart::LINE);
	plotter->plot();
	while (!plotter->wasStopped())
	{
		plotter->spinOnce();
	}
}

void Viewer_PLFH_data(plane_set PlaneSet, vector<pcl::PLFH_gather> plfh_connected_set) {
	for (int i = 0; i < PlaneSet.getPlaneNumber(); i++) {
		//cout << "��" << i << "��ƽ��ĽǶ�ֱ��ͼ��";
		int j = 0;
		for (; j < (plfh_connected_set[i].nr_dimensions_) / 2; j++) {
			cout << plfh_connected_set[i].features_histogram[j] << "    ";
		}
		cout << endl;

		//cout << "��" << i << "��ƽ��ľ���ֱ��ͼ��";
		for (; j < plfh_connected_set[i].nr_dimensions_; j++) {
			cout << plfh_connected_set[i].features_histogram[j] << "    ";
		}
		cout << endl;
	}
}