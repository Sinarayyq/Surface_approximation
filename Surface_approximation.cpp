#include <iostream>
#include <time.h>
#include <deque>
#include <iterator>
#include <list>
#include <direct.h>
using namespace std;
//#define NOMINMAX
//CGAL headers
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/convex_hull_2.h>

#include <pcl/common/geometry.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

//#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/config.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/prim_minimum_spanning_tree.hpp>
#include "boost/graph/topological_sort.hpp"
#include <boost/property_map/property_map.hpp>
//#undef BOOST_NO_CXX11_SCOPED_ENUMS


#include "read_parameters.h"
#include "cloud_visualizer.h"
#include "single_patch_recognition.h"
#include "multi_patches_recognition.h"
//#include "two_patches_recognition.h"
//#include "check_patch_validity.h"
#include "geometry_tools.h"
#include "border_definition.h"
#include "segmentation.h"
#include "io.h"
#include "mesh.h"

#include "ui_Alpha_shapes_2.h"
#include <CGAL/Qt/resources.h>
//#include <GL/glut.h>

using namespace std;


bool IsSupportPos(std::string* load_file)
{
	/*std::cerr << *((*load_file).end() - 1) << std::endl;
	std::cerr << *((*load_file).end() - 2) << std::endl;
	std::cerr << *((*load_file).end() - 3) << std::endl;
	std::cerr << *((*load_file).end() - 4) << std::endl;*/

	if (*((*load_file).end() - 1) == '"')
	{
		if ((*((*load_file).end() - 2) == 'l') || (*((*load_file).end() - 2) == 'L') &&
			(*((*load_file).end() - 3) == 't') || (*((*load_file).end() - 3) == 'T') &&
			(*((*load_file).end() - 4) == 'S') || (*((*load_file).end() - 4) == 's') &&
			(*((*load_file).end() - 5) == ('.')))
		{
			(*load_file).erase((*load_file).begin(), (*load_file).begin() + 1);
			(*load_file).erase((*load_file).begin() + (*load_file).size() - 1, (*load_file).begin() + (*load_file).size());
			return true;
		}

	}
	else
	{
		if ((*((*load_file).end() - 1) == 'l') || (*((*load_file).end() - 1) == 'L') &&
			(*((*load_file).end() - 2) == 't') || (*((*load_file).end() - 2) == 'T') &&
			(*((*load_file).end() - 3) == 'S') || (*((*load_file).end() - 2) == 's') &&
			(*((*load_file).end() - 4) == ('.')))
			return true;
	}

	return false;
}

//void ThreePointsToPlane(pcl::PointXYZ point_a, const pcl::PointXYZ point_b,const pcl::PointXYZ point_c,pcl::ModelCoefficients::Ptr plane)
//{
//	// Create Eigen plane through 3 points 
//	Eigen::Hyperplane<float, 3> eigen_plane = 
//	Eigen::Hyperplane<float, 3>::Through(point_a.getArray3fMap(),point_b.getArray3fMap(),point_c.getArray3fMap());
//
//	plane->values.resize(4);
//
//	for (int i = 0; i < plane->values.size(); i++)
//		plane->values[i] = eigen_plane.coeffs()[i];
//}

void TransformCloseToCoordinateSystem(pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud)
{
	//pcl::ModelCoefficients::Ptr plane;
	//ThreePointsToPlane((cloud->at(0))., cloud->at(1), cloud->at(2), plane);
	// Ground plane estimation:
	Eigen::VectorXf ground_coeffs;
	ground_coeffs.resize(4);
	std::vector<int> clicked_points_indices;
	for (int i = 0; i < 3; i++)
	{
		clicked_points_indices.push_back(i);
	}
	pcl::SampleConsensusModelPlane<pcl::PointXYZ> model_plane(*cloud);
	model_plane.computeModelCoefficients(clicked_points_indices, ground_coeffs);
	std::cout << "Ground plane: " << ground_coeffs(0) << " " << ground_coeffs(1) << " " << ground_coeffs(2) << " " << ground_coeffs(3) << std::endl;
	pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients());
	plane_coefficients->values.push_back((*cloud)->at(0).x - 1000);
	plane_coefficients->values.push_back((*cloud)->at(0).y - 1000);
	plane_coefficients->values.push_back((*cloud)->at(0).z - 1000);
	
	for (int i = 0; i < 4; i++)
	{
		plane_coefficients->values.push_back(ground_coeffs(i));
	}
	plane_coefficients->values.push_back(0.8);
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	transformed_cloud = transformConicalPatchPoints(*cloud, plane_coefficients->values);
	(*cloud).swap(transformed_cloud);

	//pcl::PointCloud<pcl::Normal>::Ptr transformed_cloud_normals(new pcl::PointCloud<pcl::Normal>);
	//transformed_cloud_normals = transformPlanarPatchPoints(cloud_normal, plane_coefficients->values);
	//(*cloud_normal).swap(*transformed_cloud_normals);
}



int main(int argc, char** argv)
{
	using namespace std;
	using namespace pcl;
	using namespace boost;
	using namespace Eigen;

	//Create temp folder if not already existed.
	
	if ((CreateDirectory((PATH_HEAD + "\\temp").c_str(), NULL) || ERROR_ALREADY_EXISTS == GetLastError()))
	{
		
	}
	else
	{
		mkdir((PATH_HEAD + "\\temp").c_str());
	}
		

	//Read the parameters from the external text file
	if (!readParameterFile(PATH_HEAD + "\\source\\input extract indices.txt"))
	{
		return(0);
	}

	string load_file;
	while (getline(cin, load_file))
	{
		//std::cerr << "Input a STL file:" << std::endl;
		if (!IsSupportPos(&load_file))
		{
			std::cout << "ERROR: File is not stl." << std::endl;
			continue;
		}


		/*int length_of_load_file = load_file.size();
		bool flag_space = false;
		for (int i = 0; i < length_of_load_file - 1; i++)
		{
		if (load_file[i] == ' ')
		{
		flag_space = true;
		}
		}
		if (flag_space == true)
		{
		std::cout << "ERROR: File name contains space." << std::endl;
		continue;
		}*/




		/*string::size_type pos = 0;
		while ((pos = load_file.find_first_of('\\', pos)) != string::npos)
		{
		load_file.insert(pos, "\\");
		pos = pos + 2;
		}*/

		//cout << "1load_file:" << load_file << endl;

		//load_file = load_file + ".stl";
		//string load_path = "C:\\Extract_indices\\STL\\" + load_file;


		// Define the point clouds for points and for normals that will be used in the process
		pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::Normal>::Ptr original_cloud_normals(new pcl::PointCloud<pcl::Normal>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	
		ReadSTLFile(load_file.c_str(), &original_cloud, &original_cloud_normals);
		pcl::copyPointCloud(*original_cloud, *cloud);
		pcl::copyPointCloud(*original_cloud_normals, *cloud_normals);
		//visualizePointCloud(original_cloud, "original_cloud",xy);
		//visualizePointCloud(original_cloud_normals, "original_cloud_normals",xy);
		//visualizePointCloud(cloud, "cloud",xy);
		//visualizePointCloud(cloud_normals, "cloud_normals",xy);
		//pcl::io::loadPCDFile(PATH_PCD_DOC, *cloud_filtered);
		//// Fill the initial point clouds reading from the PCD file
		//if (!fillClouds(PATH_PCD_DOC, cloud_filtered, cloud_normals))
		//{
		//	return (-1);
		//}
		//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		//Define the object for writing in output PCD files the segmented sub clouds
		//pcl::PCDReader  reader;
		//reader.read("C:\\Development\\Surface_approximation\\build\\table_scene_lms400.pcd", *cloud);

		// Visualize the initial point cloud
		//visualizePointCloud(cloud,"INITIAL POINT CLOUD");

		// Set the minimum number of inliers required for each surface fitting 
		const int nr_points = (int)original_cloud->points.size();
		const double threshold_inliers = MIN_INLIERS * nr_points;
		cerr << endl << "INITIAL NUMBER OF POINTS IN THE CLOUD: " << nr_points << endl;
		//std::cerr << std::endl << "MIN_INLIERS = " << MIN_INLIERS << std::endl;
		//cerr << endl << "threshold_inliers = " << threshold_inliers << endl;

		int patch_count = 0;
		const int max_patches = nr_points / threshold_inliers;  //max number of patches that we could find
		Eigen::MatrixXf *patch_data = new Eigen::MatrixXf[max_patches]; //vector of matrices
		pcl::PointCloud<pcl::PointXYZ>::Ptr *sourceClouds = new pcl::PointCloud<pcl::PointXYZ>::Ptr[max_patches]; //vector of pointers to cloud
																												  //vector < PointCloud<PointXYZ>::Ptr, Eigen::aligned_allocator <PointCloud <PointXYZ>::Ptr > > sourceClouds;
		visualizePointCloud(cloud, "cloud before transforming", xy);
		TransformCloseToCoordinateSystem(&cloud);
		visualizePointCloud(cloud, "cloud after transforming", xy);

		//int model_with_maximum_points[4];   // [1/2/3][num_plane][num_cylinder][num_cone]
		                                    // [1/2/3]: the model with maximum points,0 = plane,1=cylinder,2=cone

		/*Models_recognition_results results_single_patch_recognition;
		                          
		if (!SinglePatchPartition(&cloud, &cloud_normals, threshold_inliers, &patch_count, &patch_data, &sourceClouds, &results_single_patch_recognition))
		{

			TwoPatchesPartition(&cloud, &cloud_normals, threshold_inliers, &patch_count, &patch_data, &sourceClouds, results_single_patch_recognition);
		}*/

		//if (!SinglePatchPartition(&cloud, &cloud_normals, threshold_inliers, &patch_count, &patch_data, &sourceClouds))
		//{
			MultiPatchesPartition(&cloud, &cloud_normals, threshold_inliers, &patch_count, &patch_data, &sourceClouds);
		//}



		visualizePointCloud(cloud, "cloud", xy);

		

		
		/*PlaneRecognition(original_cloud, threshold_inliers, &inliers, &coefficients);
		FindPlaneBorder(&original_cloud, &original_cloud_normals, inliers, coefficients,&patch_count, &patch_data, &sourceClouds);

		CylinderRecognition(original_cloud, original_cloud_normals, threshold_inliers, &inliers, &coefficients);
		FindCylinderBorder(&original_cloud, &original_cloud_normals,inliers, coefficients,&patch_count, &patch_data, &sourceClouds);

		ConeRecognition(original_cloud, original_cloud_normals, threshold_inliers, &inliers, &coefficients);
		FindConeBorder(&original_cloud, &original_cloud_normals, inliers, coefficients, &patch_count, &patch_data, &sourceClouds);*/

		
		


		//// ----> Match of the found patches, according to the previously identified border lines.
		//// Print of the info found so far
		//std::cerr << std::endl << "========================================================================" << std::endl;
		//std::cerr << "============================ PATCH MATCHING ============================" << std::endl << std::endl;
		//std::cerr << "NUMBER OF CYLINDRICAL and CONICAL PATCHES:" << patch_count<< endl <<endl;
		//std::cerr << "Print of the data related to the found chains of candidate lines" << endl;
		//std::cerr << "(1st column: index of the head of the line" << endl;
		//std::cerr << " 2nd column: index of the tail of the line" << endl;
		//std::cerr << " 3rd column: index of the chain of lines it belongs to)" << endl << endl;
		//for (int i=0; i < patch_count;i++)
		//{
		//	cerr << endl << "- " << i+1 << "th patch." <<endl<<endl;
		//	//cerr<<endl<< "NUMBER OF CANDIDATE LINES = " << patch_data[i].rows() <<endl;
		//	for (int j =0;j<patch_data[i].rows();j++)
		//	{
		//		cerr<< patch_data[i](j )<<" "<<patch_data[i](j+patch_data[i].rows())<<" "<< patch_data[i](j+2*(patch_data[i].rows()))<<endl;
		//	}
		//}

		//// Put in corrispondence patches that has close enough border lines
		//merge_patches(patch_data, sourceClouds,patch_count,TOLERANCE_FOR_ADJACENT_CHAINS);

		//std::cout << "matrix 1: "  << std::endl;
		//std::cout << data[0].indexes_one << std::endl;

		//std::cout << "matrix 2: "  << std::endl;
		//std::cout << data[0].indexes_two << std::endl;
		//

		//// ---> Program terminates.
		//std::cerr << std::endl << "============================================" << std::endl;
		//std::cerr << "============================================" << std::endl << std::endl;
		//std::cerr << cloud_filtered->size() << " points have not been fitted in any surface." << std::endl;
		std::cerr << "====================END=====================" << std::endl << std::endl;
		std::system("pause");
	}
	return 0;
}