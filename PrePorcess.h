#pragma once
#include <pcl/filters/voxel_grid.h>           //�����������񻯵��˲���ͷ�ļ� 
#include <pcl/filters/filter.h>             //�˲����ͷ�ļ�
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>          //�˲������ͷ�ļ�
#include <pcl/filters/statistical_outlier_removal.h> //ͳ�Ʒ���ȥ����Ⱥ��
#include <pcl/filters/radius_outlier_removal.h> //ͳ�Ʒ���ȥ����Ⱥ��
#include <pcl/filters/approximate_voxel_grid.h>  //ApproximateVoxelGrid 


//�����²���
void DownSample(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out,float DownSampleLeafSize)
{
	//down sample
	std::cout << "begin downSample cloud_in size: " << cloud_in->size() << std::endl;
	pcl::VoxelGrid<pcl::PointXYZRGB> downSampled;  //�����˲�����
	downSampled.setInputCloud(cloud_in);            //������Ҫ���˵ĵ��Ƹ��˲�����
	downSampled.setLeafSize(DownSampleLeafSize, DownSampleLeafSize, DownSampleLeafSize);  //�����˲�ʱ�������������Ϊ1cm�������壨1Ϊ�ף�0.01����1cm��
	downSampled.filter(*cloud_out);  //ִ���˲������洢���

	std::cout << "success downSample, size: " << cloud_out->size() << std::endl;
}

//ȥ����Ⱥ��
void OutlierFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out)
{
	std::cout << "begin outlierFilter cloud_in size: " << cloud_in->size() << std::endl;

	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> pcFilter;  //�����˲�������
	pcFilter.setInputCloud(cloud_in);             //���ô��˲��ĵ���
	pcFilter.setMeanK(50);					//���������ĸ���
	pcFilter.setStddevMulThresh(1);		//�ñ�׼����ֵϵ����ֵԽС���˲�Ч��Խǿ
	pcFilter.filter(*cloud_out);        //�˲�����洢��cloud_filtered
	std::cout << "success OutlierFilter, size: " << cloud_out->size() << std::endl;

}


//�Ե��Ƶ�Ԥ�������ǵ����²�����ȥ��Ⱥ���˲���
void PrePorcessingOfPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out, float DownSampleLeafSize,bool if_outfilter)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZRGB>());
	DownSample(cloud_in, cloud_out, DownSampleLeafSize);
	if(if_outfilter==true)OutlierFilter(cloud_out, cloud_out);//scan1��acmh����Ҫȥ��Ⱥ�㣬acmm��Ҫ

	// �����������
	const std::string saved_pcd_path = "./cloud_tmp.pcd";   // �洢·��Ϊ����·�����Ұ�����׺
	bool binary_mode = false;  
	// �ɹ�����0��ʧ�ܷ���-1
	if (-1 == pcl::io::savePCDFile(saved_pcd_path, *cloud_out, binary_mode)) {
		std::cout << "save pcd file failed." << std::endl;
		//return -1;
	}
	else {
		std::cout << "pcd file saved at " << saved_pcd_path << std::endl << std::endl;
	}
}
