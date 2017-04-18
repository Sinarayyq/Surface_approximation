#include <iostream>
#include <time.h>
#include <deque>
#include <iterator>
#include <list>
using namespace std;
#define NOMINMAX
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
//#include "check_patch_validity.h"
#include "geometry_tools.h"
#include "border_definition.h"
#include "segmentation.h"
#include "io.h"
#include "mesh.h"
#include <iostream>
#include <direct.h>
#include "ui_Alpha_shapes_2.h"
#include <CGAL/Qt/resources.h>

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

string exePath()
{
	char buffer[MAX_PATH];
	GetModuleFileName(NULL, buffer, MAX_PATH);
	string::size_type pos = string(buffer).find_last_of("\\/");
	return string(buffer).substr(0, pos);
}

int main(int argc, char** argv)
{
	using namespace std;
	using namespace pcl;
	using namespace boost;
	using namespace Eigen;

	//Create temp folder if not already existed.
	string project_path = exePath() + "\\..\\..";
	if ((CreateDirectory((project_path + "\\temp").c_str(), NULL) || ERROR_ALREADY_EXISTS == GetLastError()))
	{
		
	}
	else
	{
		mkdir((project_path + "\\temp").c_str());
	}
		

	//Read the parameters from the external text file
	if (!readParameterFile(exePath() + "\\..\\..\\source\\input extract indices.txt"))
	{
		return(0);
	}

	string load_file;
	while (getline(cin, load_file))
	{
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
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>), cloud_p(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>), cloud_normals_f(new pcl::PointCloud<pcl::Normal>);
		ReadSTLFile(load_file.c_str(), &cloud_filtered, &cloud_normals);
		//pcl::io::loadPCDFile(PATH_PCD_DOC, *cloud_filtered);
		//// Fill the initial point clouds reading from the PCD file
		//if (!fillClouds(PATH_PCD_DOC, cloud_filtered, cloud_normals))
		//{
		//	return (-1);
		//}

		// Define the object for writing in output PCD files the segmented sub clouds
		pcl::PCDWriter writer;

		// Visualize the initial point cloud
		//visualizePointCloud(cloud_filtered,"Cluster viewer: INITIAL POINT CLOUD");	

		// Set the minimum number of inliers required for each surface fitting 
		const int nr_points = (int)cloud_filtered->points.size();
		const double threshold_inliers = MIN_INLIERS * nr_points;
		cerr << endl << "INITIAL NUMBER OF POINTS IN THE CLOUD: " << nr_points << endl;
		//std::cerr << std::endl << "MIN_INLIERS = " << MIN_INLIERS << std::endl;
		cerr << endl << "threshold_inliers = " << threshold_inliers << endl;

		// Define 
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

		// Create the filtering object
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		pcl::ExtractIndices<pcl::Normal> extract_normals;

		int i = 0;
		int num_inliers = 0;
		int patch_count = 0;
		const int max_patches = nr_points / threshold_inliers;  //max number of patches that we could find
		Eigen::MatrixXf *patch_data = new Eigen::MatrixXf[max_patches]; //vector of matrices
		pcl::PointCloud<pcl::PointXYZ>::Ptr *sourceClouds = new pcl::PointCloud<pcl::PointXYZ>::Ptr[max_patches]; //vector of pointers to cloud
																												  //vector < PointCloud<PointXYZ>::Ptr, Eigen::aligned_allocator <PointCloud <PointXYZ>::Ptr > > sourceClouds;
		int count_flattened_cloud = 0;



//
//#pragma region plane
//
//
//
//		// while at least 'threshold_inliers' points from the original cloud are still not fitted (e.g. 30%)
//		while (cloud_filtered->points.size() > threshold_inliers)
//		{
//			readParameterFile(exePath() + "\\..\\..\\source\\input extract indices.txt");
//			// ---> planar segmentation
//			// create the segmentation object for plane
//			pcl::SACSegmentation<pcl::PointXYZ> seg;
//
//			// set all the parameters for the segmentation object for plane
//			setSegmentationParametersForPlane(seg);
//
//
//			std::cerr << std::endl << std::endl << std::endl << "============================ plane ============================" << std::endl;
//
//			// segment the largest planar component from the remaining cloud
//			seg.setInputCloud(cloud_filtered);
//			seg.segment(*inliers, *coefficients);
//			num_inliers = inliers->indices.size();
//
//			if (num_inliers == 0)
//			{
//				std::cerr << std::endl;
//				std::cerr << "no plane found." << num_inliers << std::endl << std::endl;
//				break;
//			}
//
//			std::cerr << std::endl;
//			std::cerr << "found a plane! number of inliers = " << num_inliers << std::endl << std::endl;
//			if (num_inliers < threshold_inliers)
//			{
//				std::cerr << num_inliers << " < " << threshold_inliers << ". min num of inliers not reached ----->>> patch discarded" << std::endl << std::endl << std::endl << std::endl << std::endl;
//				break;
//			}
//
//			// extract the inliers
//			extract.setInputCloud(cloud_filtered);
//			extract.setIndices(inliers);
//			extract.setNegative(false);
//			extract.filter(*cloud_p);
//			//std::cerr << "pointcloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;
//
//			// visualize the fitted points
//			//visualizePointCloud(cloud_p,"found planar patch");
//			visualizeTwoPointClouds(cloud_filtered, cloud_p, "found planar patch");
//			std::cerr << "Do you want to discard the plane panel and try other recognition?" << std::endl;
//			std::cerr << "If yes, input 0;" << std::endl;
//			std::cerr << "if you want to try plane recognition again, change the tolerence before inputting 1;" << std::endl;
//			std::cerr << "if you think it is good enough, input anything else. " << std::endl;
//			int check_again;
//			std::cin >> check_again;
//			cin.clear();
//			cin.ignore(numeric_limits<streamsize>::max(), '\n');
//			if (check_again == 0)
//			{
//				break;
//			}
//			else
//			{
//				if (check_again == 1)
//				{
//					continue;
//				}
//			}
//
//
//
//			bool good_patch_marker_plane = 1;
//			patch_data[patch_count] = MainPlanarPatch(cloud_p, coefficients->values, &good_patch_marker_plane, &count_flattened_cloud);
//			if (good_patch_marker_plane == 0)
//			{
//				std::cerr << "The patch is not good enough to fabricate so discarded." << std::endl;
//				std::cerr << "Do you want to try plane recognition gain ?" << std::endl;
//				std::cerr << "if not, input 0; if yes, input anything else." << std::endl;
//				int check_again = 1;
//				std::cin >> check_again;
//				cin.clear();
//				cin.ignore(numeric_limits<streamsize>::max(), '\n');
//				if (check_again == 0)
//				{
//					break;
//				}
//				else
//				{
//					continue;
//				}
//				//continue;
//			}
//
//			PointCloud<PointXYZ>::Ptr sourceCloud(new PointCloud<PointXYZ>);
//			*sourceCloud = *cloud_p;
//			sourceClouds[patch_count] = sourceCloud;
//			patch_count++;
//
//
//
//			// update the cloud of points still to fit in a patch
//			extract.setNegative(true);
//			extract.filter(*cloud_f);
//			cloud_filtered.swap(cloud_f);
//			extract_normals.setNegative(true);
//			extract_normals.setInputCloud(cloud_normals);
//			extract_normals.setIndices(inliers);
//			extract_normals.filter(*cloud_normals_f);
//			cloud_normals.swap(cloud_normals_f);
//
//			//// check validity of the patch (discard strips and check if there are groups of fitted points to be separated)
//			//if(checkpatchvalidity("plane", cloud_p, cloud_filtered))
//			//{
//
//			//}
//
//			// print the coefficients of the found plane model:
//			std::cerr << std::endl << "coef plane: " << std::endl;
//			std::cerr << "Ax + By + Cz + D = 0" << std::endl;
//			std::cerr << "A: " << coefficients->values[0] << std::endl;
//			std::cerr << "B: " << coefficients->values[1] << std::endl;
//			std::cerr << "C: " << coefficients->values[2] << std::endl;
//			std::cerr << "D: " << coefficients->values[3] << std::endl;
//			//cout << "load_file:" << load_file << endl;
//			//outputCloudOnExcel(coefficients, load_file,"plane");
//
//
//			std::cerr << std::endl;
//			// information to save somewhere
//
//
//
//			//pcl::pointcloud<pcl::PointXYZ>::ptr transformed_planar_patch = transformplanarpatchpoints(cloud_p, coefficients->values);
//			//getminarearectofplanarpatch(cloud_p, coefficients->values);
//
//			// make a pcd file containing the fitted points
//			//std::stringstream ss;
//			//ss << "c:\\extract_indices\\build\\debug\\output\\resulting_plane_" << i << ".pcd";
//			//writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);
//
//			std::cerr << "------ remaining points: " << cloud_filtered->size() << " data points." << std::endl << std::endl << std::endl << std::endl << std::endl;
//			i++;
//		}
//#pragma endregion




#pragma region cylinder
		// ---> cylindrical segmentation


		//i = 0;
		while (cloud_filtered->points.size() > threshold_inliers) // && i<8)
		{
			readParameterFile(exePath() + "\\..\\..\\source\\input extract indices.txt");
			// create the segmentation object
			pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg_cyl;
			//pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
			//pcl::search::kdtree<pcl::PointXYZ>::ptr tree (new pcl::search::kdtree<pcl::PointXYZ> ());
			//pcl::pointcloud<pcl::normal>::ptr cloud_normals (new pcl::pointcloud<pcl::normal>), cloud_normals_f(new pcl::pointcloud<pcl::normal>);
			//pcl::extractindices<pcl::normal> extract_normals;

			//// estimate point normals
			//ne.setsearchmethod (tree);
			//ne.setInputCloud (cloud_filtered);
			//ne.setksearch (k_for_normal_search);
			//ne.compute (*cloud_normals);

			// set all the parameters for cylinder segmentation object
			setSegmentationParametersForCylinder(seg_cyl);

			std::cerr << std::endl << "============================ cylinder ============================" << std::endl;

			// segment the largest cylindrical component from the remaining cloud
			seg_cyl.setInputCloud(cloud_filtered);
			seg_cyl.setInputNormals(cloud_normals);
			seg_cyl.segment(*inliers, *coefficients);
			num_inliers = inliers->indices.size();

			if (num_inliers == 0)
			{
				std::cerr << std::endl;
				std::cerr << "no cylinder found." << num_inliers << std::endl;
				break;
			}

			std::cerr << std::endl;
			std::cerr << "found a cylinder! number of inliers = " << num_inliers << std::endl;
			if (num_inliers < threshold_inliers)
			{
				std::cerr << num_inliers << " < " << threshold_inliers << ". min num of inliers not reached ----->>> patch discarded" << std::endl << std::endl << std::endl << std::endl << std::endl;
				break;
			}

			// extract the inliers
			extract.setInputCloud(cloud_filtered);
			extract.setIndices(inliers);
			extract.setNegative(false);
			extract.filter(*cloud_p);
			//std::cerr << "pointcloud representing the cylindrical component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

			// visualize the fitted points
			//cloud_p->push_back (pcl::PointXYZ (0,0,0));
			//visualizePointCloud(cloud_p,"found cylindrical patch");
			visualizeTwoPointClouds(cloud_filtered, cloud_p, "found cylindrical patch");
			std::cerr << "Do you want to discard the cylinder panel and try cone recognition?" << std::endl;
			std::cerr << "If yes, input 0;" << std::endl;
			std::cerr << "if you want to try cylinder recognition again, change the tolerence before inputting 1;" << std::endl;
			std::cerr << "if you think it is good enough, input anything else. " << std::endl;
			int check_again;
			std::cin >> check_again;
			cin.clear();
			cin.ignore(numeric_limits<streamsize>::max(), '\n');
			if (check_again == 0)
			{
				break;
			}
			else
			{
				if (check_again == 1)
				{
					continue;
				}
			}
			//pcl::pointcloud<pcl::PointXYZ>::iterator u;
			//u= cloud_p->end()

			//cloud_p->points.erase(u-1);

			// determine the possible shared border lines of the patch
			bool good_patch_marker_cylinder = 1;
			patch_data[patch_count] = MainCylindricalPatch(cloud_p, coefficients->values, &good_patch_marker_cylinder, &count_flattened_cloud);
			if (good_patch_marker_cylinder == 0)
			{
				std::cerr << "The patch is not good enough to fabricate so discarded." << std::endl;
				std::cerr << "Do you want to try cylinder recognition gain ?" << std::endl;
				std::cerr << "if not, input 0; if yes, input anything else." << std::endl;

				int check_again = 1;
				cin >> check_again;
				cin.clear();
				cin.ignore(numeric_limits<streamsize>::max(), '\n');
				if (check_again == 0)
				{
					break;
				}
				else
				{
					continue;
				}
				//continue;
			}

			//patch_data[patch_count] = MainCylindricalPatch(cloud_p, coefficients->values);
			PointCloud<PointXYZ>::Ptr sourceCloud(new PointCloud<PointXYZ>);
			*sourceCloud = *cloud_p;
			sourceClouds[patch_count] = sourceCloud;
			patch_count++;

			// update the cloud of points still to fit in a patch
			extract.setNegative(true);
			extract.filter(*cloud_f);
			cloud_filtered.swap(cloud_f);
			extract_normals.setNegative(true);
			extract_normals.setInputCloud(cloud_normals);
			extract_normals.setIndices(inliers);
			extract_normals.filter(*cloud_normals_f);
			cloud_normals.swap(cloud_normals_f);

			// check validity of the patch (discard strips and check if there are groups of fitted points to be separated)
			/*if(checkpatchvalidity("cylinder", cloud_p, cloud_filtered))
			{

			}*/
			// print the coefficients of the found cylinder model:
			std::cerr << std::endl;
			std::cerr << "coef cylinder: " << std::endl;
			std::cerr << "appex:        " << coefficients->values[0] << ", " << coefficients->values[1] << ", " << coefficients->values[2] << std::endl;
			std::cerr << "central axis: " << coefficients->values[3] << ", " << coefficients->values[4] << ", " << coefficients->values[5] << std::endl;
			std::cerr << "radius:       " << coefficients->values[6] << std::endl << std::endl;
			std::cerr << std::endl;

			//outputCloudOnExcel(coefficients, load_file, "cylinder");



			// information to save+ somewhere





			//pcl::pointcloud<pcl::PointXYZ>::ptr transformed_cyl_patch = transformcylindricalpatchpoints(cloud_p, coefficients->values);
			//std::vector<point> hull_points = convexhullforplanarpoints(cloud_p);

			//// make a pcd file containing the fitted points
			//std::stringstream ss;
			//ss << "c:\\extract_indices\\build\\debug\\output\\resulting_cylinder_" << i << ".pcd";
			//writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);
			////  ----- to add normal information also

			std::cerr << "------ remaining points: " << cloud_filtered->size() << " data points." << std::endl << std::endl << std::endl << std::endl << std::endl;
			//std::cerr << "------ remaining normals: " << all_cloud_normals->size() << " data points." << std::endl;
			i++;
		}
#pragma endregion 





#pragma region CONE

		// ---> CONICAL SEGMENTATION

		//i = 0;

		while (cloud_filtered->points.size() > threshold_inliers)
		{
			readParameterFile(exePath() + "\\..\\..\\source\\input extract indices.txt");
			// Create the segmentation object for cone
			pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg_cone;
			// Set all the parameters for cone segmentation object
			setSegmentationParametersForCone(seg_cone);
			std::cerr << std::endl << "============================ CONE ============================" << std::endl;

			// Segment the largest conical component from the remaining cloud
			seg_cone.setInputCloud(cloud_filtered);
			seg_cone.setInputNormals(cloud_normals);
			seg_cone.segment(*inliers, *coefficients);
			num_inliers = inliers->indices.size();

			if (num_inliers == 0)
			{
				std::cerr << std::endl;
				std::cerr << "NO CONE FOUND." << num_inliers << std::endl;
				break;
			}

			std::cerr << std::endl;
			std::cerr << "FOUND A CONE! Number of inliers = " << num_inliers << std::endl;
			if (num_inliers < threshold_inliers)
			{
				std::cerr << num_inliers << " < " << threshold_inliers << ". Min num of inliers not reached ----->>> PATCH DISCARDED" << std::endl << std::endl << std::endl << std::endl << std::endl;
				break;
			}

			// Extract the inliers
			extract.setInputCloud(cloud_filtered);
			extract.setIndices(inliers);
			extract.setNegative(false);
			extract.filter(*cloud_p);
			//std::cerr << "PointCloud representing the conical component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

			// Visualize the fitted points
			//visualizePointCloud(cloud_p,"found conical patch");
			visualizeTwoPointClouds(cloud_filtered, cloud_p, "found conical patch");
			std::cerr << "Do you want to discard the cone panel?" << std::endl;
			std::cerr << "If yes, input 0;" << std::endl;
			std::cerr << "if you want to try cone recognition again, change the tolerence before inputting 1;" << std::endl;
			std::cerr << "if you think it is good enough, input anything else. " << std::endl;
			int check_again;
			std::cin >> check_again;
			cin.clear();
			cin.ignore(numeric_limits<streamsize>::max(), '\n');
			if (check_again == 0)
			{
				break;
			}
			else
			{
				if (check_again == 1)
				{
					continue;
				}
			}


			// Determine the possible shared border lines of the patch
			bool good_patch_marker_cone = 1;
			patch_data[patch_count] = MainConicalPatch(cloud_p, coefficients->values, &good_patch_marker_cone, &count_flattened_cloud);
			if (good_patch_marker_cone == 0)
			{
				std::cerr << "The patch is not good enough to fabricate so discarded." << std::endl;
				std::cerr << "Do you want to try cone recognition gain ?" << std::endl;
				std::cerr << "if not, input 0; if yes, input anything else." << std::endl;

				int check_again = 1;
				cin >> check_again;
				cin.clear();
				cin.ignore(numeric_limits<streamsize>::max(), '\n');
				if (check_again == 0)
				{
					break;
				}
				else
				{
					continue;
				}
			}

			//patch_data[patch_count] = MainConicalPatch(cloud_p, coefficients->values);
			PointCloud<PointXYZ>::Ptr sourceCloud(new PointCloud<PointXYZ>);
			*sourceCloud = *cloud_p;
			sourceClouds[patch_count] = sourceCloud;
			patch_count++;


			// Update the cloud of points still to fit in a patch
			extract.setNegative(true);
			extract.filter(*cloud_f);
			cloud_filtered.swap(cloud_f);
			extract_normals.setNegative(true);
			extract_normals.setInputCloud(cloud_normals);
			extract_normals.setIndices(inliers);
			extract_normals.filter(*cloud_normals_f);
			cloud_normals.swap(cloud_normals_f);

			// Print the coefficients of the found cylinder model:
			// Print the coefficients of the found cylinder model:	
			std::cerr << std::endl;
			std::cerr << "coef cone: " << std::endl;
			std::cerr << "appex:        " << coefficients->values[0] << ", " << coefficients->values[1] << ", " << coefficients->values[2] << std::endl;
			std::cerr << "central axis: " << coefficients->values[3] << ", " << coefficients->values[4] << ", " << coefficients->values[5] << std::endl;
			std::cerr << "semi-angle:   " << coefficients->values[6] << std::endl;
			std::cerr << std::endl;

			//outputCloudOnExcel(coefficients, load_file, "cone");


			// INFORMATION TO SAVE SOMEWHERE





			//// Make a pcd file containing the fitted points
			//std::stringstream ss;
			//ss << "C:\\Extract_indices\\build\\Debug\\output\\resulting_cone_" << i << ".pcd";
			//writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);
			////  ----- TO ADD NORMAL INFORMATION ALSO

			std::cerr << "------ Remaining points: " << cloud_filtered->size() << " data points." << std::endl << std::endl << std::endl << std::endl << std::endl;
			//std::cerr << "------ Remaining normals: " << cloud_normals->size() << " data points." << std::endl;
			i++;
		}

#pragma endregion 





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