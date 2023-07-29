#pragma once

#include <pcl/sample_consensus/ransac.h>
#include <vector>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include<pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/sac_segmentation.h>   //RANSAC�ָ�
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h> // ���ƽ��
#include <pcl/sample_consensus/method_types.h>   //����������Ʒ���
#include <pcl/sample_consensus/model_types.h>    //ģ�Ͷ���
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>//����������
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/segmentation/sac_segmentation.h>
#include<string>
#include <thread>

using namespace std;

class plane {
public:
	//ƽ��ϵ��
	pcl::ModelCoefficients cofficient_set;//ģ��ϵ��
	pcl::PointCloud<pcl::PointXYZRGB> cloud_plane;//���Ƽ�
	Eigen::Vector3f normal;
	
	//���캯��
	plane() {
		normal(0) = 0;
		normal(1) =0;
		normal(2) = 0;
	};
	plane(pcl::ModelCoefficients cofficientset, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane_) {
		cofficient_set = cofficientset;
		pcl::copyPointCloud(*cloud_plane_, cloud_plane);
		
		normal(0) = cofficient_set.values[0];
		normal(1) = cofficient_set.values[1];
		normal(2) = cofficient_set.values[2];
	}
};

class plane_set
{
public:
	
	//��ȡƽ�溯�� ���յõ�һ��ƽ�漯
	void Extrace_plane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int MAX_ITERATION_PLANE, float CylinderThresold);
	int getPlaneNumber() {
		return plane_number;
	};
	vector<plane> getPlaneSet() {
		return planeset;
	};
	plane_set() {
		plane_number = 0;
		planeset.clear();
	}
	
private:
	int plane_number;
	vector<plane> planeset;//ƽ�漯��

};

class cylinder {
public:
	//ƽ��ϵ��
	//pcl::ModelCoefficients cofficient_set;//ģ��ϵ��
	Eigen::VectorXf cofficient_set;//ģ��ϵ��
	pcl::PointCloud<pcl::PointXYZRGB> cloud_cylinder;//���Ƽ�
	//Eigen::Vector3f normal;
	//���캯��
	cylinder(Eigen::VectorXf& cofficientset, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder_) {
		cofficient_set = cofficientset;
		pcl::copyPointCloud(*cloud_cylinder_, cloud_cylinder);
	}
};

class cylinder_set
{
public:

	//��ȡƽ�溯�� ���յõ�һ��ƽ�漯
	void Extrace_cylinder(int minPointNu, float CylinderThresold);
	int getCylinderNumber() {
		return cylinder_number;
	};
	vector<cylinder> getCylinderSet() {
		return cylinderset;
	};
	cylinder_set() {
		cylinder_number = 0;
		cylinderset.clear();
	}

private:
	int cylinder_number;
	vector<cylinder> cylinderset;//Բ������
};