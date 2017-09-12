#include "patch.h"

Patch::Patch(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normal, Shape model, int threshold):cloud_input(cloud), cloud_input_normals(normal),
            current_model(model), cloud_inlier(new pcl::PointCloud<pcl::PointXYZ>), cloud_inlier_normals(new pcl::PointCloud<pcl::Normal>), 
	        cloud_remainder(new pcl::PointCloud<pcl::PointXYZ>), cloud_remainder_normals(new pcl::PointCloud<pcl::Normal>), 
	        inliers(new pcl::PointIndices()), coefficients(new pcl::ModelCoefficients()), threshold_inliers(threshold)
{
	// Use input_cloud to run RANSAC	
	// Obtain the initial inliers and corresponding properties
	std::cout << std::endl << "shape:" << model << std::endl;
	std::cout << "input_cloud.size = " << cloud_input->points.size() << std::endl;

	//Use RANSAC/PROSAC method to look for developable surface in the input cloud
	if (RecognizeAndSegment(cloud_input, cloud_input_normals, current_model, &inliers, &coefficients) < threshold)
	{
		std::cout << "There is no";
		switch (current_model)
    	{
		   case Shape::plane: 
		   {
			   std::cout << " plane ";
		   }
		   break;

           case Shape::cylinder:
		   {
			   std::cout << " cylinder ";
		   }
		   break;
		
	       case Shape::cone:
		   {
			   std::cout << " cone ";
		   }
		   break;
	    }
		std::cout << "in the input cloud." << std::endl;
		inliers->indices.clear();
		return;
	}
	//cloud_inlier = ExtractCloud(cloud_input, (this->inliers), false);
	//cloud_remainder = ExtractCloud(cloud_input, (this->inliers), true);
	//cloud_inlier_normals = ExtractNormal(cloud_input_normals, inliers, false);
	//cloud_remainder_normals = ExtractNormal(cloud_input_normals, inliers, false);

	//After finding the model, extracting it from the whole panel
	//Here, there is no need to worry about if the parameters exist because RecognizeAndSegment Function already checked for remaining operations

	
	ExtractCloudAndNormal(cloud_input, cloud_input_normals, inliers, &cloud_inlier, &cloud_inlier_normals, &cloud_remainder, &cloud_remainder_normals);
	DisplayCoefficients(current_model, inliers, coefficients);
	visualizePointCloud(cloud_input, cloud_inlier, "before fixing holes and fragments", xy);
	//visualizePointCloud(cloud_inlier, "before fixing holes and fragments", xy);

		/*for (auto it = cloud_inlier->begin(); it != cloud_inlier->end(); ++it)
		{
			indices_map.insert(std::pair<int, int>(distance(cloud_inlier->begin(), it), ));
		};*/
	//Project the inliers on the RANSAC parametric model.
	ProjectInliersOnTheModel(&cloud_inlier, current_model, coefficients);
	float max_gap;
	pcl::PointCloud<pcl::PointXYZ>::Ptr flattened_cloud = FlattenPatches(cloud_inlier, current_model, coefficients, &max_gap);

	//alpha shape
	FixHoleAndFragmentation(&flattened_cloud, &max_gap);
	visualizePointCloud(cloud_input, cloud_inlier, "after fixing holes and fragments", xy);
	//visualizePointCloud(cloud_inlier, "after fixing holes and fragments", xy);

	FindBorders(flattened_cloud, max_gap);
	if (CheckPiecesInCloudRemainder(cloud_remainder))
	{
		std::cout << "more than 1 clusters in cloud_remainder" << std::endl;
		inliers->indices.clear();
	}


	std::cout << "cloud_remainder.size = " << cloud_remainder->points.size() << std::endl;


}

void DisplayCoefficients(Shape model, pcl::PointIndices::Ptr inliers, pcl::ModelCoefficients::Ptr coefficients)
{
	std::cout << "inliers.size = " << inliers->indices.size() << std::endl;
	switch (model)
	{
	       case Shape::plane:
		   {
			   std::cerr << "coef plane: " << std::endl;
			   std::cerr << "Ax + By + Cz + D = 0" << std::endl;
			   std::cerr << "A: " << coefficients->values[0] << std::endl;
			   std::cerr << "B: " << coefficients->values[1] << std::endl;
			   std::cerr << "C: " << coefficients->values[2] << std::endl;
			   std::cerr << "D: " << coefficients->values[3] << std::endl;
		   }
	       break;

	       case Shape::cylinder:
	       {
			   std::cerr << "coef cylinder: " << std::endl;
			   std::cerr << "appex:        " << coefficients->values[0] << ", " << coefficients->values[1] << ", " << coefficients->values[2] << std::endl;
			   std::cerr << "central axis: " << coefficients->values[3] << ", " << coefficients->values[4] << ", " << coefficients->values[5] << std::endl;
			   std::cerr << "radius:       " << coefficients->values[6] << std::endl;
	       }
	       break;

	       case Shape::cone:
	       {
			   std::cerr << "coef cone: " << std::endl;
			   std::cerr << "appex:        " << coefficients->values[0] << ", " << coefficients->values[1] << ", " << coefficients->values[2] << std::endl;
			   std::cerr << "central axis: " << coefficients->values[3] << ", " << coefficients->values[4] << ", " << coefficients->values[5] << std::endl;
			   std::cerr << "semi-angle:   " << coefficients->values[6] << std::endl;
	       }
	       break;
	}
}

bool CheckPiecesInCloudRemainder(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_remainder)
{
	if (cloud_remainder->points.size() == 0)
	{
		return true;
	}
	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud_remainder);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(20); 
	ec.setMinClusterSize(10);
	ec.setMaxClusterSize(500000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud_remainder);
	ec.extract(cluster_indices);

	int size = cluster_indices.size();
	if (size > 1)
	{
		return true; //It means there is only one piece in cloud_remainder
	}
	else
	{
		return false;  //It means there is more than one piece in cloud_remainder
	}
	
}

int Patch::FindBorders(pcl::PointCloud<pcl::PointXYZ>::Ptr flattened_cloud, float max_gap)
{
	double convex_hull_area;
	Eigen::MatrixXf temp_patch_data;
	switch (current_model)
	{
		   case Shape::plane:
       	   {
			   temp_patch_data = MainPlanarPatch(flattened_cloud, coefficients->values);		   
	       }
	       break;

	       case Shape::cylinder:
	       {
			   temp_patch_data = MainCylindricalPatch(flattened_cloud, coefficients->values);
	       }
	       break;

	       case Shape::cone:
	       {
			   temp_patch_data = MainConicalPatch(flattened_cloud, coefficients->values, max_gap);
	       }
	       break;
	}
	int size_temp_patch_data = temp_patch_data.size() / 3;
	if (size_temp_patch_data < 1)
	{
		std::cout << "Cannot find borders" << std::endl;
		return 0;
	}
	std::vector<int> sub_serial_number_boundary;
	sub_serial_number_boundary.push_back(temp_patch_data(0, 0));
	sub_serial_number_boundary.push_back(temp_patch_data(0, 1));
	for (int i = 1; i < size_temp_patch_data; i++)
	{
		if (temp_patch_data(i, 2) == temp_patch_data(i - 1, 2))
		{
			//sub_serial_number_boundary.push_back(temp_patch_data(i, 0));
			sub_serial_number_boundary.push_back(temp_patch_data(i, 1));
		}
		else
		{
			serial_number_boundary.push_back(sub_serial_number_boundary);
			sub_serial_number_boundary.clear();
			sub_serial_number_boundary.push_back(temp_patch_data(i, 0));
			sub_serial_number_boundary.push_back(temp_patch_data(i, 1));
		}
	}
	serial_number_boundary.push_back(sub_serial_number_boundary);
	return 0;
}

//There is no need to worry about if the parameters exist because RecognizeAndSegment Function already checked for remaining operations
pcl::PointCloud<pcl::PointXYZ>::Ptr FlattenPatches(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Shape model, 
	            pcl::ModelCoefficients::Ptr coefficients, float *max_gap)
{
	switch (model)
	{
	       case Shape::plane:
		   {
			   pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_pla_patch_cloud(new pcl::PointCloud<pcl::PointXYZ>);
			   transformed_pla_patch_cloud = transformPlanarPatchPoints(cloud, coefficients->values);
			   return transformed_pla_patch_cloud;
		   }
		   break;

	       case Shape::cylinder:
	       {
			   //We flatten the cloud corresponding to the cylindrical patch
			   pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cyl_patch_cloud(new pcl::PointCloud<pcl::PointXYZ>);
			   transformed_cyl_patch_cloud = transformCylindricalPatchPoints(cloud, coefficients->values);
			   pcl::PointCloud<pcl::PointXYZ>::Ptr flattened_cloud(new pcl::PointCloud<pcl::PointXYZ>);
			   flattened_cloud = flattenCylindricalPatch(transformed_cyl_patch_cloud, coefficients->values);
			   return flattened_cloud;
		   }
		   break;

	       case Shape::cone:
	       {
			   //rotate the cone so that the axis aligns with the +Z axis, the origin point is the conic point
			   pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cone_patch_cloud = transformConicalPatchPoints(cloud, coefficients->values);
			   //visualizePointCloud(transformed_cone_patch_cloud, "transformed_cone_patch_cloud", xy);
			   //flatten 3d point cloud into XOZ-Plane
			   pcl::PointCloud<pcl::PointXYZ>::Ptr flattened_cloud = FlattenCloud(transformed_cone_patch_cloud, coefficients->values, max_gap);
			   //visualizePointCloud(flattened_cloud, "flattened_cloud", xy);
			   return flattened_cloud;
	       }
		   break;
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr flattened_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	return flattened_cloud;
}

int ProjectInliersOnTheModel(pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud, Shape model, pcl::ModelCoefficients::Ptr coefficients)
{
	int cloud_size = (*cloud)->points.size();
	int coefficients_size = coefficients->values.size();
	if (cloud_size == 0)
	{
		std::cout << "There are no points in the inlier cloud." << std::endl;
		return -1;
	}
	if (coefficients_size == 0)
	{
		std::cout << "There are no values in the coefficients parameters." << std::endl;
		return -1;
	}
	if (!sizeof(model))
	{
		std::cout << "There is no model input." << std::endl;
		return -1;
	}

	pcl::ProjectInliers<pcl::PointXYZ> proj;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
	switch (model)
	{
	       case Shape::plane: 
		   {
			   proj.setModelType(pcl::SACMODEL_PLANE);
		   }
		   break;

           case Shape::cylinder:
		   {
			   proj.setModelType(pcl::SACMODEL_CYLINDER);
		   }
		   break;
		
	       case Shape::cone:
		   {
			   proj.setModelType(pcl::SACMODEL_CONE);
		   }
		   break;
	}
	proj.setInputCloud(*cloud);
	proj.setModelCoefficients(coefficients);
	proj.filter(*cloud_projected);
	(*cloud).swap(cloud_projected);

	return 0;
}

int RecognizeAndSegment( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normal, Shape model,
	                    pcl::PointIndices::Ptr *inliers, pcl::ModelCoefficients::Ptr *coefficients)
{
	int cloud_size = cloud->points.size();
	int normal_size = normal->points.size();
	if (normal_size != cloud_size)
	{
		std::cout << "The numbers of points in the input cloud and the input normal are not equal." << std::endl;
		return -1;
	}
	if (cloud_size == 0)
	{
		std::cout << "There are no points in the input cloud." << std::endl;
		return -1;
	}
	if (!sizeof(model))
	{
		std::cout << "There is no model input." << std::endl;
		return -1;
	}

	switch (model)
	{
	       case Shape::plane: 
		   {
			   pcl::SACSegmentation<pcl::PointXYZ> seg;
			   setSegmentationParametersForPlane(seg);
			   seg.setInputCloud(cloud);
			   seg.segment(**inliers, **coefficients);
		   }
		   break;

           case Shape::cylinder:
		   {
			   pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg_cyl;
			   setSegmentationParametersForCylinder(seg_cyl);
			   seg_cyl.setInputCloud(cloud);
			   seg_cyl.setInputNormals(normal);
			   seg_cyl.segment(**inliers, **coefficients);
		   }
		   break;
		
	       case Shape::cone:
		   {
			   pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg_cone;
			   setSegmentationParametersForCone(seg_cone);
			   seg_cone.setInputCloud(cloud);
			   seg_cone.setInputNormals(normal);
			   seg_cone.segment(**inliers, **coefficients);
		   }
		   break;
	}
	return (*inliers)->indices.size();
}


pcl::PointCloud<pcl::PointXYZ>::Ptr ExtractCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointIndices::Ptr inliers, bool inside_or_outside)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_result(new pcl::PointCloud<pcl::PointXYZ>);
	int cloud_size = cloud->points.size();
	int inliers_size = inliers->indices.size();

	if (inliers_size == 0)
	{
		return cloud;
	}
	if (cloud_size == 0)
	{
		return 0;
	}

	// extract the inliers
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(inliers);
	switch (inside_or_outside)
	{
	    case true:
		     extract.setNegative(true);   //get outside points
		     break;
	    case false:
		     extract.setNegative(false);  //get inside points
		     break;
	}
	extract.filter(*cloud_result);
	//std::cout << "cloud.size="<< cloud_result->points.size() <<" bool=" << inside_or_outside<< std::endl;

	return cloud_result;
}

pcl::PointCloud<pcl::Normal>::Ptr ExtractNormal(pcl::PointCloud<pcl::Normal>::Ptr normal, pcl::PointIndices::Ptr inliers, bool inside_or_outside)
{

	pcl::PointCloud<pcl::Normal>::Ptr normal_result(new pcl::PointCloud<pcl::Normal>);
	int normal_size = normal->points.size();
	int inliers_size = inliers->indices.size();
	if (inliers_size == 0)
	{
		return normal;
	}
	if (normal_size == 0)
	{
		return 0;
	}


	pcl::ExtractIndices<pcl::Normal> extract_normals;
	extract_normals.setInputCloud(normal);
	extract_normals.setIndices(inliers);
	switch (inside_or_outside)
	{
     	case true:
	     	 extract_normals.setNegative(true);   //get outside points
		     break;

    	case false:
		     extract_normals.setNegative(false);  //get inside points
		     break;
	}
	extract_normals.filter(*normal_result);

	return normal_result;
}

int ExtractCloudAndNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normal, pcl::PointIndices::Ptr inliers, 
	                       pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud_result, pcl::PointCloud<pcl::Normal>::Ptr *normal_result, bool inside_or_outside)
{
	int cloud_size = cloud->points.size();
	int normal_size = normal->points.size();
	int inliers_size = inliers->indices.size();
	if (normal_size != cloud_size)
	{
		std::cout << "The numbers of points in the input cloud and the input normal are not equal." << std::endl;
		return -1;
	}
	if (inliers_size == 0)
	{
		pcl::copyPointCloud(*cloud, **cloud_result);
		pcl::copyPointCloud(*normal, **normal_result);
		return 0;
	}
	if (cloud_size == 0)
	{
		std::cout << "There are no points in the input cloud." << std::endl;
		return -1;
	}
	

	pcl::ExtractIndices<pcl::PointXYZ> extract;
	pcl::ExtractIndices<pcl::Normal> extract_normals;

	extract.setInputCloud(cloud);
	extract.setIndices(inliers);
	extract_normals.setInputCloud(normal);
	extract_normals.setIndices(inliers);
	switch (inside_or_outside)
	{
	       case true:
	       {
			   extract.setNegative(true);
		       extract_normals.setNegative(true);   //get outside points
	       }
		   break;
		   
		   case false:
		   {
			   extract.setNegative(false);
			   extract_normals.setNegative(false);   //get outside points
		   }
		   break;
	}
	extract.filter(**cloud_result);
	extract_normals.filter(**normal_result);
	return 0;
}
 
int ExtractCloudAndNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normal, pcl::PointIndices::Ptr inliers,
	pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud_inside, pcl::PointCloud<pcl::Normal>::Ptr *normal_inside, 
	pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud_outside, pcl::PointCloud<pcl::Normal>::Ptr *normal_outside)
{
	int cloud_size = cloud->points.size();
	int normal_size = normal->points.size();
	int inliers_size = inliers->indices.size();
	if (normal_size != cloud_size)
	{
		std::cout << "The numbers of points in the input cloud and the input normal are not equal." << std::endl;
		return -1;
	}
	if (inliers_size == 0)
	{
		pcl::copyPointCloud(*cloud, **cloud_outside);
		pcl::copyPointCloud(*normal, **normal_outside);
		return 0;
	}
	if (cloud_size == 0)
	{
		std::cout << "There are no points in the input cloud." << std::endl;
		return -1;
	}

	pcl::ExtractIndices<pcl::PointXYZ> extract;
	pcl::ExtractIndices<pcl::Normal> extract_normals;

	extract.setInputCloud(cloud);
	extract.setIndices(inliers);
	extract_normals.setInputCloud(normal);
	extract_normals.setIndices(inliers);

	extract.setNegative(true);
	extract_normals.setNegative(true);   //get outside points
	extract.filter(**cloud_outside);
	extract_normals.filter(**normal_outside);

	extract.setNegative(false);
	extract_normals.setNegative(false);   //get outside points
	extract.filter(**cloud_inside);
	extract_normals.filter(**normal_inside);

	return 0;
}


//bool SortPolygonList(const Polygon_2& lhs, const Polygon_2& rhs)
//{ 
//	return lhs.area() > rhs.area();
//}

bool SortPolygonList(const Polygon_2& lhs, const Polygon_2& rhs)
{
	return lhs.area() > rhs.area();
}

int Patch::FixHoleAndFragmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr *flattened_cloud, float *max_gap)
{
	//flatten
	//pcl::PointCloud<pcl::PointXYZ>::Ptr flattened_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	//FlattenInlierPointsBasedOnModel(&flattened_cloud, Patch::current_model, Patch::cloud_inlier, Patch::coefficients);
	//displayAlphaShape(*flattened_cloud);
	Polygon_list polygon_list = getAlphaShape(*flattened_cloud);
	

	if (polygon_list.size() == 1)  // there is one piece without hole inside
	{
		return 0;
	}

	//sort polygons based on area
	/*std::sort(polygon_list.begin(), 
		      polygon_list.end(),
		      SortPolygonList);*/
	polygon_list.sort(SortPolygonList);

	/*for (int i = 0; i < polygon_list.size(); i++)
	{
		std::cout << "area = " << polygon_list
	}*/



	Polygon_2 polygon_max_area = polygon_list.front();
	polygon_list.pop_front();

	//check if there is a hole inside the polygon with maximun area
	Polygon_2 hole = CheckHoleInside(polygon_max_area, &polygon_list);
    
	//when polygon_list.size()==0 at this moment, there is only one piece with one hole inside
	//when polygon_list.size()> 0, there are other pieces which need to be given back to cloud_input
	if (polygon_list.size() > 0) 
	{
		// give back other pieces to cloud_remainder and only leave main part in flattened_cloud
		GiveBackOtherPiecesToCloud(&(this->cloud_inlier), &(this->cloud_inlier_normals), &(this->cloud_remainder), 
			                       &(this->cloud_remainder_normals), flattened_cloud, polygon_max_area);
	}
	
	if (hole.size() == 0)
	{
		return 0;
	}
	else
	{
		FillHole(hole, *flattened_cloud, &(this->cloud_inlier), &(this->cloud_inlier_normals), 
			     &(this->cloud_remainder), &(this->cloud_remainder_normals), &(this->inliers));
		*flattened_cloud = FlattenPatches(cloud_inlier, current_model, coefficients, max_gap);
	}
	
	return 0;
}

//void FlattenInlierPointsBasedOnModel(pcl::PointCloud<pcl::PointXYZ>::Ptr *flattened_cloud, Shape model,
//	                                 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inlier,pcl::ModelCoefficients::Ptr coefficients)
//{
//	if (model == plane)
//	{
//
//		*flattened_cloud = transformPlanarPatchPoints(cloud_inlier, coefficients->values);
//	}
//	else
//	{
//		if (model == cylinder)
//		{
//			pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cyl_patch_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//			transformed_cyl_patch_cloud = transformCylindricalPatchPoints(cloud_inlier, coefficients->values);
//			*flattened_cloud = flattenCylindricalPatch(transformed_cyl_patch_cloud, coefficients->values);
//		}
//		else
//		{
//			if (model == cone)
//			{
//				pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cone_patch_cloud = transformConicalPatchPoints(cloud_inlier, coefficients->values);
//				*flattened_cloud = FlattenCloud(transformed_cone_patch_cloud, coefficients->values);
//			}
//		}
//	}
//}

Polygon_2 CheckHoleInside(Polygon_2 polygon_max_area, Polygon_list *polygon_list)
{
	Polygon_2 hole;
	std::cout << "polygon_max_area area = " << polygon_max_area.area() << std::endl;

	for (Polygon_list::iterator it = (*polygon_list).begin(); it != (*polygon_list).end(); ++it)
	{
		std::cout << "area = " << (*it).area() << std::endl;
		if (CGAL::bounded_side_2(polygon_max_area.vertices_begin(), polygon_max_area.vertices_end(),
			Point((*((*it).vertices_begin())).x(), (*((*it).vertices_begin())).y()), K()) != CGAL::ON_UNBOUNDED_SIDE)
		{
			hole = *it;
			(*polygon_list).remove(*it);
			return hole;
		}
	}
	return hole;
}

void GiveBackOtherPiecesToCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud_inlier, pcl::PointCloud<pcl::Normal>::Ptr *cloud_inlier_normals,
	                            pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud_remainder, pcl::PointCloud<pcl::Normal>::Ptr *cloud_remainder_normals,
	                            pcl::PointCloud<pcl::PointXYZ>::Ptr *flattened_cloud, Polygon_2 polygon_max_area)
{
	
	pcl::PointIndices real_indices = GetIndicesOfPointsInsideAndOnPolygon(*flattened_cloud, polygon_max_area);
	
	
	//update cloud data
	pcl::PointIndices::Ptr real_inliers(new pcl::PointIndices(real_indices));

	pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr temp_cloud_normals(new pcl::PointCloud<pcl::Normal>);

	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(*cloud_inlier);
	extract.setIndices(real_inliers);
	extract.setNegative(true);
	extract.filter(*temp_cloud);
	extract.setNegative(false);
	extract.filter(**cloud_inlier);
	

	pcl::ExtractIndices<pcl::Normal> extract_normals;
	extract_normals.setInputCloud(*cloud_inlier_normals);
	extract_normals.setIndices(real_inliers);
	extract_normals.setNegative(true);
	extract_normals.filter(*temp_cloud_normals);
	extract_normals.setNegative(false);
	extract_normals.filter(**cloud_inlier_normals);
	
	(**cloud_remainder) += (*temp_cloud);
	(**cloud_remainder_normals) += (*temp_cloud_normals);

	//visualizePointCloud(*flattened_cloud, "before extract", xz);
	pcl::ExtractIndices<pcl::PointXYZ> extract_flattened_cloud;
	extract_flattened_cloud.setInputCloud(*flattened_cloud);
	extract_flattened_cloud.setIndices(real_inliers);
	extract_flattened_cloud.setNegative(false);
	extract_flattened_cloud.filter(**flattened_cloud);
	//visualizePointCloud(*flattened_cloud, "after extract", xz);

}

//problem
void FillHole(Polygon_2 hole, pcl::PointCloud<pcl::PointXYZ>::Ptr flattened_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud_inlier,
	pcl::PointCloud<pcl::Normal>::Ptr *cloud_inlier_normals, pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud_remainder,
	pcl::PointCloud<pcl::Normal>::Ptr *cloud_remainder_normals, pcl::PointIndices::Ptr *inliers)
{
	pcl::PointIndices indices_hole = GetIndicesOfPointsOnPolygon(flattened_cloud, hole);

	//get cloud of hole
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hole(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_hole = GetCloudBasedOnIndices(indices_hole, *cloud_inlier);
	//visualizePointCloud(*cloud_inlier, cloud_hole, "cloud_inlier+cloud_hole", xy);

	//get the plane comprised by the hole throught RANSAC method
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	pcl::PointIndices::Ptr inliers_plane_hole(new pcl::PointIndices());
	pcl::ModelCoefficients::Ptr coefficients_plane_hole(new pcl::ModelCoefficients());
	setSegmentationParametersForPlane(seg);
	seg.setInputCloud(cloud_hole);
	seg.segment(*inliers_plane_hole, *coefficients_plane_hole);

	//Project cloud_remainder on the plane
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_remainder_projected(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_remainder_projected = GetProjectedCloud(plane, coefficients_plane_hole, *cloud_remainder);

	//Tranform to xy-plane
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud_hole(new pcl::PointCloud<pcl::PointXYZ>);
	transformed_cloud_hole = transformPlanarPatchPoints(cloud_hole, coefficients_plane_hole->values);
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud_remainder_projected(new pcl::PointCloud<pcl::PointXYZ>);
	transformed_cloud_remainder_projected = transformPlanarPatchPoints(cloud_remainder_projected, coefficients_plane_hole->values);
	visualizePointCloud(transformed_cloud_remainder_projected, transformed_cloud_hole, "hole", xy);

	//displayAlphaShape(transformed_cloud_hole);
	//displayAlphaShape(transformed_cloud_remainder_projected);


	//sequence the points so that we could use CGAL::bounded_side_2 function to decide 
	//which part of points in transformed_cloud_remainder_projected is inside transformed_cloud_hole
	std::vector<point> convex_hull_cloud_hole = convexHullForPoints(transformed_cloud_hole);

	Polygon_2 hole_on_transformed_plane;
	for (int i = 0; i < convex_hull_cloud_hole.size(); i++)
	{
		hole_on_transformed_plane.push_back(Point(boost::get<0>(convex_hull_cloud_hole[i]), boost::get<1>(convex_hull_cloud_hole[i])));
	}

	//move the points inside the polygon from cloud_remainder to cloud_inlier
	pcl::PointIndices indices_points_inside_hole = GetIndicesOfPointsInsidePolygon(transformed_cloud_remainder_projected, hole_on_transformed_plane);
	MovePartsFromCloudToCloud(indices_points_inside_hole, &cloud_remainder, &cloud_remainder_normals, &cloud_inlier, &cloud_inlier_normals);
	
	UpdateInliers(&inliers, indices_points_inside_hole);
	


}

void UpdateInliers(pcl::PointIndices::Ptr **inliers, pcl::PointIndices indices_points_inside_hole)
{
	pcl::PointIndices::Ptr inliers_temp(new pcl::PointIndices());
	//int size_update_inliers = (**inliers)->indices.size() + (**inliers)->indices.size();
	int size_update_inliers = (**inliers)->indices.size();
	int size_update_hole = indices_points_inside_hole.indices.size();

	for (int i = 0; i < size_update_inliers; i++)
	{
		inliers_temp->indices.push_back((**inliers)->indices.at(i));
	}

	for (int i = 0; i < size_update_hole; i++)
	{
		inliers_temp->indices.push_back(indices_points_inside_hole.indices.at(i));
	}

	(**inliers)->indices.clear();
	(**inliers) = inliers_temp;
}

pcl::PointIndices GetIndicesOfPointsInsideAndOnPolygon(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Polygon_2 polygon)
{
	pcl::PointIndices indices;
	int size_flattened_cloud = cloud->points.size();
	for (int i = 0; i < size_flattened_cloud; i++)
	{
		if (CGAL::bounded_side_2(polygon.vertices_begin(), polygon.vertices_end(),
			Point(cloud->at(i).x, cloud->at(i).z), K()) != CGAL::ON_UNBOUNDED_SIDE)
		{
			indices.indices.push_back(i);
		}
	}
	return indices;
}

pcl::PointIndices GetIndicesOfPointsOnPolygon(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Polygon_2 polygon)
{
	pcl::PointIndices indices;
	int size_flattened_cloud = cloud->points.size();
	for (int i = 0; i < size_flattened_cloud; i++)
	{
		if (CGAL::bounded_side_2(polygon.vertices_begin(), polygon.vertices_end(),
			Point(cloud->at(i).x, cloud->at(i).z), K()) == CGAL::ON_BOUNDARY)
		{
			indices.indices.push_back(i);
		}
	}
	return indices;
}

pcl::PointIndices GetIndicesOfPointsInsidePolygon(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Polygon_2 polygon)
{
	pcl::PointIndices indices;
	int size_flattened_cloud = cloud->points.size();
	for (int i = 0; i < size_flattened_cloud; i++)
	{
		if (CGAL::bounded_side_2(polygon.vertices_begin(), polygon.vertices_end(),
			Point(cloud->at(i).x, cloud->at(i).z), K()) == CGAL::ON_BOUNDED_SIDE)
		{
			indices.indices.push_back(i);
		}
	}
	return indices;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr GetCloudBasedOnIndices(pcl::PointIndices indices, pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud)
{
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices(indices));
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_indices(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(original_cloud);
	extract.setIndices(inliers);
	extract.setNegative(false);
	extract.filter(*cloud_indices);
	return cloud_indices;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr GetProjectedCloud(Shape shape, pcl::ModelCoefficients::Ptr coefficients, pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ProjectInliers<pcl::PointXYZ> proj;
	if (shape == plane)
	{
		proj.setModelType(pcl::SACMODEL_PLANE);
	}
	else
	{
		if (shape == cylinder)
		{
			proj.setModelType(pcl::SACMODEL_CYLINDER);
		}
		else
		{
			proj.setModelType(pcl::SACMODEL_CONE);
		}
	}
	
	proj.setInputCloud(original_cloud);
	proj.setModelCoefficients(coefficients);
	proj.filter(*cloud_projected);
	return cloud_projected;
}

void MovePartsFromCloudToCloud(pcl::PointIndices indices, pcl::PointCloud<pcl::PointXYZ>::Ptr **cloud_from,pcl::PointCloud<pcl::Normal>::Ptr **normal_from, 
	                           pcl::PointCloud<pcl::PointXYZ>::Ptr **cloud_to, pcl::PointCloud<pcl::Normal>::Ptr **normal_to)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_parts(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_remaining_in_cloud_from(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices(indices));

	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(**cloud_from);
	extract.setIndices(inliers);
	extract.setNegative(false);
	extract.filter(*cloud_parts);
	extract.setNegative(true);
	extract.filter(*cloud_remaining_in_cloud_from);
	(**cloud_from).swap(cloud_remaining_in_cloud_from);

	//visualizePointCloud(**cloud_from, "cloud_from", xy);
	//visualizePointCloud(cloud_parts, "cloud_parts", xy);
	//visualizePointCloud(cloud_remaining_in_cloud_from, "cloud_remaining_in_cloud_from", xy);

	//visualizePointCloud(**cloud_from, cloud_parts, "cloud_from+cloud_parts", xy);
	//visualizePointCloud(cloud_remaining_in_cloud_from, cloud_parts, "cloud_remaining_in_cloud_from+cloud_parts", xy);
	//visualizePointCloud(**cloud_from, cloud_remaining_in_cloud_from, "cloud_from+cloud_remaining_in_cloud_from", xy);

	pcl::ExtractIndices<pcl::Normal> extract_normals;
	pcl::PointCloud<pcl::Normal>::Ptr normal_parts(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Normal>::Ptr normal_remaining_in_normal_from(new pcl::PointCloud<pcl::Normal>);
	extract_normals.setInputCloud(**normal_from);
	extract_normals.setIndices(inliers);
	extract_normals.setNegative(false);
	extract_normals.filter(*normal_parts);
	extract_normals.setNegative(true);
	extract_normals.filter(*normal_remaining_in_normal_from);
	(**normal_from).swap(normal_remaining_in_normal_from);

	(***cloud_to) += (*cloud_parts);
	(***normal_to) += (*normal_parts);

}

//void Patch::CheckBoundary()
//{
//
//}



Node::Node()
{
	lchild = NULL;
	mchild = NULL;
	rchild = NULL;
}

Node::Node(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normal, Shape model, int threshold) : Patch(cloud, normal, model, threshold)
{

	if (inliers->indices.size() < threshold)
	{
		return;
	}
}


Node::~Node()
{
	//delete lchild;
	//delete mchild;
	//delete rchild;
}





Tree::Tree(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, int threshold)
{
	root = new Node;
	root->lchild = NULL;
	root->mchild = NULL;
	root->rchild = NULL;
	root->cloud_remainder = cloud;
	root->cloud_remainder_normals = normals;

	CreateTree(root, threshold);
}

Tree::~Tree()
{
	DestroyTree(root);

}

void DestroyTree(Node *leaf)
{
	if (leaf != NULL)
	{
		DestroyTree(leaf->lchild);
		DestroyTree(leaf->mchild);
		DestroyTree(leaf->rchild);
		delete leaf;
	}
	return;
}

void CreateTree(Node *node, int threshold_inliers)
{
	//there is not enough points to be fitted in any kind of models
	if ((node->cloud_remainder->points.size() < threshold_inliers))
	{
		node->lchild = NULL;
		node->mchild = NULL;
		node->rchild = NULL;
		//DestroyTree(node);
		node = NULL;
		return;
	}

	if ((node->inliers) && (node->inliers->indices.size() < threshold_inliers))
	{
		node->lchild = NULL;
		node->mchild = NULL;
		node->rchild = NULL;
		//DestroyTree(node);
		node = NULL;
		return;
	}
	/*node->lchild->patch->cloud_input = node->patch->cloud_remainder;
	node->lchild->patch->cloud_input_normals = node->patch->cloud_remainder_normals;

	node->mchild->patch->cloud_input = node->patch->cloud_remainder;
	node->mchild->patch->cloud_input_normals = node->patch->cloud_remainder_normals;

	node->rchild->patch->cloud_input = node->patch->cloud_remainder;
	node->rchild->patch->cloud_input_normals = node->patch->cloud_remainder_normals;*/

	node->lchild = new Node(node->cloud_remainder, node->cloud_remainder_normals, plane, threshold_inliers);
	node->mchild = new Node(node->cloud_remainder, node->cloud_remainder_normals, cylinder, threshold_inliers);
	node->rchild = new Node(node->cloud_remainder, node->cloud_remainder_normals, cone, threshold_inliers);


	CreateTree(node->lchild, threshold_inliers);
	CreateTree(node->mchild, threshold_inliers);
	CreateTree(node->rchild, threshold_inliers);


}

