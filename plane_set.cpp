#include "plane_set.h"

//#define MAX_ITERATION_PLANE   1
//#define MAX_ITERATION_CYCLINDER   2
//#define minPointNum 4000

pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);//索引内点的容器
pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);//索引内点的容器
pcl::PointCloud<pcl::PointXYZ>::Ptr plane_segment(new pcl::PointCloud<pcl::PointXYZ>);//分割出的平面
pcl::PointCloud<pcl::PointXYZ>::Ptr cylinder_segment(new pcl::PointCloud<pcl::PointXYZ>);//分割出的平面
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane_in(new pcl::PointCloud<pcl::PointXYZ>);
pcl::SACSegmentation<pcl::PointXYZ> seg_plane;//分割对象
pcl::ExtractIndices<pcl::PointXYZ> extract_plane;//平面提取器
pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg_cylinder;//分割对象
pcl::ExtractIndices<pcl::PointXYZ> extract_cylinder;//平面提取器
pcl::ExtractIndices<pcl::Normal> extract_normals;    ///点法线特征　提取对象
pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);//法线特征
pcl::ModelCoefficients::Ptr cofficient_plane(new pcl::ModelCoefficients);//模型系数
//pcl::ModelCoefficients::Ptr cofficient_cylinder(new pcl::ModelCoefficients);//模型系数
Eigen::VectorXf cofficient_cylinder;//圆柱模型系数

pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

void normals_estimate(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered)
{
	// Estimate point normals
	ne.setSearchMethod(tree);
	ne.setInputCloud(cloud_filtered);
	ne.setKSearch(50);
	ne.compute(*cloud_normals);
}

void
plane_set::Extrace_plane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int MAX_ITERATION_PLANE, float PlaneThresold)
{
	pcl::copyPointCloud(*cloud, *cloud_plane_in);
	for (int i = 0; i < MAX_ITERATION_PLANE; i++) {
		//拟合平面
		seg_plane.setOptimizeCoefficients(true);		//使用内部点重新估算模型参数
		seg_plane.setModelType(pcl::SACMODEL_PLANE);   //设置模型类型
		seg_plane.setMethodType(pcl::SAC_RANSAC);      //设置随机采样一致性方法类型
		seg_plane.setMaxIterations(100);         //最大迭代次数 ***
		seg_plane.setDistanceThreshold(PlaneThresold);          //设定距离阀值来决定平面内点
		seg_plane.setInputCloud(cloud_plane_in);
		seg_plane.segment(*inliers_plane, *cofficient_plane);

		//提取探测出来的平面
		extract_plane.setInputCloud(cloud_plane_in);
		extract_plane.setIndices(inliers_plane);
		extract_plane.setNegative(false);
		extract_plane.filter(*plane_segment);

		//剔除探测出的平面，在剩余点中继续探测平面
		extract_plane.setNegative(true);
		extract_plane.filter(*cloud_plane_in);

		cout << "平面内点的个数为" << plane_segment->size() << endl;
		//存储
		plane plane_(*cofficient_plane, plane_segment);//构建平面
		planeset.push_back(plane_);//加入平面集
		plane_number++;//平面数加一
		cout << "平面方程为： " << cofficient_plane->values[0] << "x + " << cofficient_plane->values[1] << "y + " << cofficient_plane->values[2] << "z + "
			<< cofficient_plane->values[3] << " = 0" << endl;
	}
	
}


void
cylinder_set::Extrace_cylinder(int minPointNum, float CylinderThresold)
{	
	// 估计各点的法向量
	normals_estimate(cloud_plane_in);

	std::vector<int> totalInliers;
	std::vector<int> indices(cloud_plane_in->points.size());
	std::iota(std::begin(indices), std::end(indices), (int)0);
	int fitPointNum = 0;
	int i = 1;
	do
	{
		std::vector<int> inliers;
		pcl::SampleConsensusModelCylinder< pcl::PointXYZ, pcl::Normal >::Ptr
			model(new pcl::SampleConsensusModelCylinder< pcl::PointXYZ, pcl::Normal >(cloud_plane_in, indices));		//创建圆柱模型
		model->setInputNormals(cloud_normals);
		model->setRadiusLimits(0.1, 3000);		//这里设置圆柱半径大小的范围

		pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model);
		ransac.setDistanceThreshold(CylinderThresold);//圆柱面内点大小
		ransac.setMaxIterations(100000);
		ransac.computeModel();
		ransac.getInliers(inliers);
		ransac.getModelCoefficients(cofficient_cylinder);
		fitPointNum = inliers.size();
		totalInliers.insert(totalInliers.end(), inliers.begin(), inliers.end());

		if (fitPointNum < minPointNum)		//判断是否满足聚类数目要求
			break;

		//提取剩余的点云数据
		std::vector<int> newIndices;
		pcl::ExtractIndices<pcl::PointXYZ> ei(true);
		ei.setInputCloud(cloud_plane_in);
		ei.setNegative(true);			//false代表提取指定的索引点，true则代表提取未指定的索引点
		ei.setIndices(std::make_shared <std::vector<int>>(totalInliers));//指定点的索引
		ei.filter(newIndices);
		indices = newIndices;
		inliers_cylinder->indices== indices;

		//提取数据
		for (int i = 0; i < inliers.size(); i++) {
			cylinder_segment->points.push_back(cloud_plane_in->points[inliers[i]]);//
		}
		if (cylinder_segment->points.empty())
			std::cerr << "Can't find the cylindrical component." << std::endl;
		else
		{
			std::cerr << "PointCloud representing the cylindrical component: " << cylinder_segment->points.size() << " data points." << std::endl;
		}

		//保存圆柱
		cylinder cylinder_(cofficient_cylinder, cylinder_segment);//构建圆柱
		cylinderset.push_back(cylinder_);//加入圆柱集合
		cylinder_number++;//圆柱个数数加一
		cylinder_segment->clear();
		cout << "圆柱系数" << endl;
		for (int j = 0; j<cofficient_cylinder.size(); j++) {
			cout << cofficient_cylinder(j)<<" ";
		}
		cout << endl;
	} while (true);
}
