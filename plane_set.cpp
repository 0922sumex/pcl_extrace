#include "plane_set.h"

//#define MAX_ITERATION_PLANE   1
//#define MAX_ITERATION_CYCLINDER   2
//#define minPointNum 4000

pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);//�����ڵ������
pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);//�����ڵ������
pcl::PointCloud<pcl::PointXYZ>::Ptr plane_segment(new pcl::PointCloud<pcl::PointXYZ>);//�ָ����ƽ��
pcl::PointCloud<pcl::PointXYZ>::Ptr cylinder_segment(new pcl::PointCloud<pcl::PointXYZ>);//�ָ����ƽ��
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane_in(new pcl::PointCloud<pcl::PointXYZ>);
pcl::SACSegmentation<pcl::PointXYZ> seg_plane;//�ָ����
pcl::ExtractIndices<pcl::PointXYZ> extract_plane;//ƽ����ȡ��
pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg_cylinder;//�ָ����
pcl::ExtractIndices<pcl::PointXYZ> extract_cylinder;//ƽ����ȡ��
pcl::ExtractIndices<pcl::Normal> extract_normals;    ///�㷨����������ȡ����
pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);//��������
pcl::ModelCoefficients::Ptr cofficient_plane(new pcl::ModelCoefficients);//ģ��ϵ��
//pcl::ModelCoefficients::Ptr cofficient_cylinder(new pcl::ModelCoefficients);//ģ��ϵ��
Eigen::VectorXf cofficient_cylinder;//Բ��ģ��ϵ��

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
		//���ƽ��
		seg_plane.setOptimizeCoefficients(true);		//ʹ���ڲ������¹���ģ�Ͳ���
		seg_plane.setModelType(pcl::SACMODEL_PLANE);   //����ģ������
		seg_plane.setMethodType(pcl::SAC_RANSAC);      //�����������һ���Է�������
		seg_plane.setMaxIterations(100);         //���������� ***
		seg_plane.setDistanceThreshold(PlaneThresold);          //�趨���뷧ֵ������ƽ���ڵ�
		seg_plane.setInputCloud(cloud_plane_in);
		seg_plane.segment(*inliers_plane, *cofficient_plane);

		//��ȡ̽�������ƽ��
		extract_plane.setInputCloud(cloud_plane_in);
		extract_plane.setIndices(inliers_plane);
		extract_plane.setNegative(false);
		extract_plane.filter(*plane_segment);

		//�޳�̽�����ƽ�棬��ʣ����м���̽��ƽ��
		extract_plane.setNegative(true);
		extract_plane.filter(*cloud_plane_in);

		cout << "ƽ���ڵ�ĸ���Ϊ" << plane_segment->size() << endl;
		//�洢
		plane plane_(*cofficient_plane, plane_segment);//����ƽ��
		planeset.push_back(plane_);//����ƽ�漯
		plane_number++;//ƽ������һ
		cout << "ƽ�淽��Ϊ�� " << cofficient_plane->values[0] << "x + " << cofficient_plane->values[1] << "y + " << cofficient_plane->values[2] << "z + "
			<< cofficient_plane->values[3] << " = 0" << endl;
	}
	
}


void
cylinder_set::Extrace_cylinder(int minPointNum, float CylinderThresold)
{	
	// ���Ƹ���ķ�����
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
			model(new pcl::SampleConsensusModelCylinder< pcl::PointXYZ, pcl::Normal >(cloud_plane_in, indices));		//����Բ��ģ��
		model->setInputNormals(cloud_normals);
		model->setRadiusLimits(0.1, 3000);		//��������Բ���뾶��С�ķ�Χ

		pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model);
		ransac.setDistanceThreshold(CylinderThresold);//Բ�����ڵ��С
		ransac.setMaxIterations(100000);
		ransac.computeModel();
		ransac.getInliers(inliers);
		ransac.getModelCoefficients(cofficient_cylinder);
		fitPointNum = inliers.size();
		totalInliers.insert(totalInliers.end(), inliers.begin(), inliers.end());

		if (fitPointNum < minPointNum)		//�ж��Ƿ����������ĿҪ��
			break;

		//��ȡʣ��ĵ�������
		std::vector<int> newIndices;
		pcl::ExtractIndices<pcl::PointXYZ> ei(true);
		ei.setInputCloud(cloud_plane_in);
		ei.setNegative(true);			//false������ȡָ���������㣬true�������ȡδָ����������
		ei.setIndices(std::make_shared <std::vector<int>>(totalInliers));//ָ���������
		ei.filter(newIndices);
		indices = newIndices;
		inliers_cylinder->indices== indices;

		//��ȡ����
		for (int i = 0; i < inliers.size(); i++) {
			cylinder_segment->points.push_back(cloud_plane_in->points[inliers[i]]);//
		}
		if (cylinder_segment->points.empty())
			std::cerr << "Can't find the cylindrical component." << std::endl;
		else
		{
			std::cerr << "PointCloud representing the cylindrical component: " << cylinder_segment->points.size() << " data points." << std::endl;
		}

		//����Բ��
		cylinder cylinder_(cofficient_cylinder, cylinder_segment);//����Բ��
		cylinderset.push_back(cylinder_);//����Բ������
		cylinder_number++;//Բ����������һ
		cylinder_segment->clear();
		cout << "Բ��ϵ��" << endl;
		for (int j = 0; j<cofficient_cylinder.size(); j++) {
			cout << cofficient_cylinder(j)<<" ";
		}
		cout << endl;
	} while (true);
}
