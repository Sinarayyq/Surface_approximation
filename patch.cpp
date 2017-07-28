#include "patch.h"

Patch::Patch(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normal, Shape model):cloud_input(cloud), cloud_input_normals(normal), 
             model(model), cloud_inlier(new pcl::PointCloud<pcl::PointXYZ>), cloud_inlier_normals(new pcl::PointCloud<pcl::Normal>), cloud_remainder(new pcl::PointCloud<pcl::PointXYZ>), 
           	cloud_remainder_normals(new pcl::PointCloud<pcl::Normal>), inliers(new pcl::PointIndices()), coefficients(new pcl::ModelCoefficients())
{
	//pcl::PointIndices::Ptr inliners(new pcl::PointIndices(this->indices));
	//pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients(this->coeff));
	// Use input_cloud to run RANSAC	
	// Obtain the initial inliers and corresponding properties
	std::cout << "shape:" << model << std::endl;
	std::cout << "cloud.size =" << cloud->points.size() << std::endl;
	switch (model)
	{
	       case Shape::plane: 
		   {
			   pcl::SACSegmentation<pcl::PointXYZ> seg;
			   setSegmentationParametersForPlane(seg);
			   seg.setInputCloud(cloud_input);
			   seg.segment(*inliers, *coefficients);
		   }
		   break;

           case Shape::cylinder:
		   {
			   pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg_cyl;
			   setSegmentationParametersForCylinder(seg_cyl);
			   seg_cyl.setInputCloud(cloud_input);
			   seg_cyl.setInputNormals(cloud_input_normals);
			   seg_cyl.segment(*(this->inliers), *(this->coefficients));
		   }
		   break;
		
	       case Shape::cone:
		   {
			   pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg_cone;
			   setSegmentationParametersForCone(seg_cone);
			   seg_cone.setInputCloud(cloud_input);
			   seg_cone.setInputNormals(cloud_input_normals);
			   seg_cone.segment(*(this->inliers), *(this->coefficients));
		   }
	}

	//cloud_inlier = ExtractCloud(cloud_input, (this->inliers), false);
	//cloud_remainder = ExtractCloud(cloud_input, (this->inliers), true);
	//cloud_inlier_normals = ExtractNormal(cloud_input_normals, inliers, false);
	//cloud_remainder_normals = ExtractNormal(cloud_input_normals, inliers, false);

	ExtractCloudAndNormal(cloud_input, cloud_input_normals, inliers, &cloud_inlier, &cloud_inlier_normals, &cloud_remainder, &cloud_remainder_normals);
	visualizePointCloud(cloud_input, cloud_inlier, "patch", xy);

		/*for (auto it = cloud_inlier->begin(); it != cloud_inlier->end(); ++it)
		{
			indices_map.insert(std::pair<int, int>(distance(cloud_inlier->begin(), it), ));
		};*/


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

int Patch::FixHoleAndFragmentation()
{
	//flatten
	pcl::PointCloud<pcl::PointXYZ>::Ptr flattened_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	FlattenInlierPointsBasedOnModel(&flattened_cloud, Patch::model, Patch::cloud_inlier, Patch::coefficients);

	Polygon_list polygon_list = getAlphaShape(flattened_cloud);

	if (polygon_list.size() == 1)  // there is one piece without hole inside
	{
		return 0;
	}

	//sort polygons based on area
	/*std::sort(polygon_list.begin(), 
		      polygon_list.end(),
		      [](const Polygon_2& lhs, const Polygon_2& rhs) { return lhs.area() > rhs.area(); });*/
	//polygon_list.sort(SortPolygonList);


	Polygon_2 polygon_max_area = polygon_list.back();
	polygon_list.pop_back();

	//check if there is a hole inside the polygon with maximun area
	Polygon_2 hole = CheckHoleInside(polygon_max_area, &polygon_list);
    
	//when polygon_list.size()==0 at this moment, there is only one piece with one hole inside
	//when polygon_list.size()> 0, there are other pieces which need to be given back to cloud_input
	if (polygon_list.size() > 0) 
	{
		GiveBackOtherPiecesToCloud(&(this->cloud_inlier), &(this->cloud_inlier_normals), &(this->cloud_remainder), 
			                       &(this->cloud_remainder_normals), flattened_cloud, polygon_max_area);
	}
	
	if (hole.size() == 0)
	{
		return 0;
	}
	else
	{
		FillHole(hole, flattened_cloud, &(this->cloud_inlier), &(this->cloud_inlier_normals), &(this->cloud_remainder), &(this->cloud_remainder_normals));
	}
	
	return 0;
}

void FlattenInlierPointsBasedOnModel(pcl::PointCloud<pcl::PointXYZ>::Ptr *flattened_cloud, Shape model,
	                                 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inlier,pcl::ModelCoefficients::Ptr coefficients)
{
	if (model == plane)
	{

		*flattened_cloud = transformPlanarPatchPoints(cloud_inlier, coefficients->values);
	}
	else
	{
		if (model == cylinder)
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cyl_patch_cloud(new pcl::PointCloud<pcl::PointXYZ>);
			transformed_cyl_patch_cloud = transformCylindricalPatchPoints(cloud_inlier, coefficients->values);
			*flattened_cloud = flattenCylindricalPatch(transformed_cyl_patch_cloud, coefficients->values);
		}
		else
		{
			if (model == cone)
			{
				pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cone_patch_cloud = transformConicalPatchPoints(cloud_inlier, coefficients->values);
				*flattened_cloud = FlattenCloud(transformed_cone_patch_cloud, coefficients->values);
			}
		}
	}
}

Polygon_2 CheckHoleInside(Polygon_2 polygon_max_area, Polygon_list *polygon_list)
{
	for (Polygon_list::iterator it = (*polygon_list).begin(); it != (*polygon_list).end(); ++it)
	{
		if (CGAL::bounded_side_2(polygon_max_area.vertices_begin(), polygon_max_area.vertices_end(),
			Point((*((*it).vertices_begin())).x(), (*((*it).vertices_begin())).y()), K()) != CGAL::ON_UNBOUNDED_SIDE)
		{
			(*polygon_list).remove(*it);
			return *it;

		}
	}
	Polygon_2 hole;
	return hole;
}

void GiveBackOtherPiecesToCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud_inlier, pcl::PointCloud<pcl::Normal>::Ptr *cloud_inlier_normals,
	                            pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud_remainder, pcl::PointCloud<pcl::Normal>::Ptr *cloud_remainder_normals,
	                            pcl::PointCloud<pcl::PointXYZ>::Ptr flattened_cloud, Polygon_2 polygon_max_area)
{
	
	pcl::PointIndices real_indices = GetIndicesOfPointsInsideAndOnPolygon(flattened_cloud, polygon_max_area);
	
	
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

}

void FillHole(Polygon_2 hole, pcl::PointCloud<pcl::PointXYZ>::Ptr flattened_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud_inlier,
	pcl::PointCloud<pcl::Normal>::Ptr *cloud_inlier_normals, pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud_remainder,
	pcl::PointCloud<pcl::Normal>::Ptr *cloud_remainder_normals)
{
	pcl::PointIndices indices_hole = GetIndicesOfPointsOnPolygon(flattened_cloud, hole);

	//get cloud of hole
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hole(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_hole = GetCloudBasedOnIndices(indices_hole, *cloud_inlier);

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

	//move the points inside the polygon from cloud_remainder to cloud_inlier
	pcl::PointIndices indices_points_inside_hole = GetIndicesOfPointsInsidePolygon(cloud_remainder_projected, hole);
	MovePartsFromCloudToCloud(indices_points_inside_hole, &cloud_remainder, &cloud_remainder_normals, &cloud_inlier, &cloud_inlier_normals);

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

Node::Node(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normal, Shape model) : Patch(cloud, normal, model)
{

}


Node::~Node()
{
	delete lchild;
	delete mchild;
	delete rchild;
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

void Tree::DestroyTree(Node *leaf)
{
	if (leaf != NULL)
	{
		DestroyTree(leaf->lchild);
		DestroyTree(leaf->mchild);
		DestroyTree(leaf->rchild);
		delete leaf;
	}
}

void CreateTree(Node *node, int threshold_inliers)
{
	if (node->cloud_remainder->points.size() < threshold_inliers)
	{
		node->lchild = NULL;
		node->mchild = NULL;
		node->rchild = NULL;
		return;
	}
	/*node->lchild->patch->cloud_input = node->patch->cloud_remainder;
	node->lchild->patch->cloud_input_normals = node->patch->cloud_remainder_normals;

	node->mchild->patch->cloud_input = node->patch->cloud_remainder;
	node->mchild->patch->cloud_input_normals = node->patch->cloud_remainder_normals;

	node->rchild->patch->cloud_input = node->patch->cloud_remainder;
	node->rchild->patch->cloud_input_normals = node->patch->cloud_remainder_normals;*/

	node->lchild = new Node(node->cloud_remainder, node->cloud_remainder_normals, plane);
	node->mchild = new Node(node->cloud_remainder, node->cloud_remainder_normals, cylinder);
	node->rchild = new Node(node->cloud_remainder, node->cloud_remainder_normals, cone);


	CreateTree(node->lchild, threshold_inliers);
	CreateTree(node->mchild, threshold_inliers);
	CreateTree(node->rchild, threshold_inliers);


}

