#include <pcl/visualization/pcl_plotter.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "Viewer_Results.h"
#include "PrePorcess.h"
#include "plfh_solver.h"
#include "Bilateral_consensus.h"
#include "Cyclinder_set.h"


//调参
int MAX_ITERATION_PLANE = 1;//提取的平面个数
int minPointNum = 3500;  //圆柱聚类的最少内点个数
float PlaneThresold = 0.05; //平面的内点距离阈值
float CylinderThresold = 0.02;  //圆柱的内点距离阈值
float DownSampleLeafSize =0.02f;    //下采样的体素网格大小
bool If_OutFilter =true;//判断是否下采样

int
main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);//读取的点云
	pcl::PointCloud<pcl::PointXYZRGB> cloud_copy;//点云副本
    plane_set PlaneSet_S;
    cylinder_set CylinderSet_S;

    LoadClouds(cloud);//读取点云
	cloud_copy = *cloud;
    PrePorcessingOfPointCloud(cloud_copy.makeShared(), cloud,DownSampleLeafSize, If_OutFilter);//预处理

	PlaneSet_S.Extrace_plane(cloud, MAX_ITERATION_PLANE, PlaneThresold);//提取平面
    CylinderSet_S.Extrace_cylinder(minPointNum, CylinderThresold);//提取圆柱
    Viewer_Results_clouds(cloud, PlaneSet_S, CylinderSet_S);//可视化点云
   
}

