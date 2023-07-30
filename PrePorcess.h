#pragma once
#include <pcl/filters/voxel_grid.h>           //用于体素网格化的滤波类头文件 
#include <pcl/filters/filter.h>             //滤波相关头文件
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>          //滤波相关类头文件
#include <pcl/filters/statistical_outlier_removal.h> //统计方法去除离群点
#include <pcl/filters/radius_outlier_removal.h> //统计方法去除离群点
#include <pcl/filters/approximate_voxel_grid.h>  //ApproximateVoxelGrid 


//点云下采样
void DownSample(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out,float DownSampleLeafSize)
{
	//down sample
	std::cout << "begin downSample cloud_in size: " << cloud_in->size() << std::endl;
	pcl::VoxelGrid<pcl::PointXYZRGB> downSampled;  //创建滤波对象
	downSampled.setInputCloud(cloud_in);            //设置需要过滤的点云给滤波对象
	downSampled.setLeafSize(DownSampleLeafSize, DownSampleLeafSize, DownSampleLeafSize);  //设置滤波时创建的体素体积为1cm的立方体（1为米，0.01就是1cm）
	downSampled.filter(*cloud_out);  //执行滤波处理，存储输出

	std::cout << "success downSample, size: " << cloud_out->size() << std::endl;
}

//去除离群点
void OutlierFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out)
{
	std::cout << "begin outlierFilter cloud_in size: " << cloud_in->size() << std::endl;

	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> pcFilter;  //创建滤波器对象
	pcFilter.setInputCloud(cloud_in);             //设置待滤波的点云
	pcFilter.setMeanK(50);					//设置领域点的个数
	pcFilter.setStddevMulThresh(1);		//置标准差阈值系数，值越小，滤波效果越强
	pcFilter.filter(*cloud_out);        //滤波结果存储到cloud_filtered
	std::cout << "success OutlierFilter, size: " << cloud_out->size() << std::endl;

}


//对点云的预处理，就是点云下采样，去离群点滤波，
void PrePorcessingOfPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out, float DownSampleLeafSize,bool if_outfilter)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZRGB>());
	DownSample(cloud_in, cloud_out, DownSampleLeafSize);
	if(if_outfilter==true)OutlierFilter(cloud_out, cloud_out);//scan1和acmh不需要去离群点，acmm需要

	// 保存点云数据
	const std::string saved_pcd_path = "./cloud_tmp.pcd";   // 存储路径为完整路径，且包含后缀
	bool binary_mode = false;  
	// 成功返回0，失败返回-1
	if (-1 == pcl::io::savePCDFile(saved_pcd_path, *cloud_out, binary_mode)) {
		std::cout << "save pcd file failed." << std::endl;
		//return -1;
	}
	else {
		std::cout << "pcd file saved at " << saved_pcd_path << std::endl << std::endl;
	}
}
