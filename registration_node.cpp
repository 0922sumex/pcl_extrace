#include <pcl/visualization/pcl_plotter.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "Viewer_Results.h"
#include "PrePorcess.h"
#include "plfh_solver.h"
#include "Bilateral_consensus.h"
#include "Cyclinder_set.h"


//����
int MAX_ITERATION_PLANE = 1;//��ȡ��ƽ�����
int minPointNum = 4000;  //Բ������������ڵ����
float PlaneThresold = 0.035; //ƽ����ڵ������ֵ
float CylinderThresold = 0.07;  //Բ�����ڵ������ֵ
float DownSampleLeafSize =0.009f;    //�²��������������С
bool If_OutFilter = false;//�ж��Ƿ��²���
int
main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);//��ȡ�ĵ���
	pcl::PointCloud<pcl::PointXYZRGB> cloud_copy;//���Ƹ���
    plane_set PlaneSet_S;
    cylinder_set CylinderSet_S;

    LoadClouds(cloud);//��ȡ����
	cloud_copy = *cloud;
    PrePorcessingOfPointCloud(cloud_copy.makeShared(), cloud,DownSampleLeafSize, If_OutFilter);//Ԥ����

	PlaneSet_S.Extrace_plane(cloud, MAX_ITERATION_PLANE, PlaneThresold);//��ȡƽ��
    CylinderSet_S.Extrace_cylinder(minPointNum, CylinderThresold);//��ȡԲ��
    Viewer_Results_clouds(cloud, PlaneSet_S, CylinderSet_S);//���ӻ�����
   
}

