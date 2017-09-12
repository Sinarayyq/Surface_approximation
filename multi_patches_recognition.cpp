#include "multi_patches_recognition.h"


//pcl::PointCloud<pcl::PointXYZ>::Ptr ExtractandProjectPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud, pcl::PointCloud<pcl::Normal>::Ptr *cloud_normals,
//	                                                       pcl::PointIndices::Ptr inliers, pcl::ModelCoefficients::Ptr coefficients)
//{
//
//}



int MultiPatchesPartition(pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud, pcl::PointCloud<pcl::Normal>::Ptr *cloud_normals, int threshold_inliers, 
	                    int *patch_count, Eigen::MatrixXf **patch_data, pcl::PointCloud<pcl::PointXYZ>::Ptr **sourceClouds)
{
	std::cerr << std::endl << "Two patches recognition:" << std::endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_copy(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_copy(new pcl::PointCloud<pcl::Normal>);

	pcl::copyPointCloud(**cloud, *cloud_copy);
	pcl::copyPointCloud(**cloud_normals, *cloud_normals_copy);

	PLANE_TOL /= 2;
	CYL_TOL /= 2;
	CONE_TOL /= 2;


	Tree T(cloud_copy, cloud_normals_copy, threshold_inliers);



	return 0;
}