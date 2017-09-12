#pragma once
#ifndef PATCH_H
#define PATCH_H

#include <vector>
#include <string>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/io/pcd_io.h>

#include "alpha_shape_polygons.h"
#include "border_definition.h"
#include "geometry_tools.h"
#include "io.h"
#include "segmentation.h"
#include "single_patch_recognition.h"
#include "multi_patches_recognition.h"


typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
enum Shape { plane, cylinder, cone };

class Patch
{
public:
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inlier;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_remainder;
	pcl::PointCloud<pcl::Normal>::Ptr cloud_input_normals;
	pcl::PointCloud<pcl::Normal>::Ptr cloud_inlier_normals;
	pcl::PointCloud<pcl::Normal>::Ptr cloud_remainder_normals;

	pcl::PointIndices::Ptr inliers;
	//pcl::PointIndices indices;
	pcl::ModelCoefficients::Ptr coefficients;
	//pcl::ModelCoefficients coeff;
	std::list<std::vector<int>> serial_number_boundary;
	Shape current_model;
	int threshold_inliers;
	
	Patch() {};
	Patch(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normal, Shape model, int threshold);
	~Patch() {};
	//void CreatePatch();
	int FixHoleAndFragmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr *flattened_cloud, float *max_gap);
	//void CheckBoundary();
	int FindBorders(pcl::PointCloud<pcl::PointXYZ>::Ptr flattened_cloud, float max_gap);
};

class Node : public Patch
{
public:
	Node *lchild;
	Node *mchild;
	Node *rchild;


	Node();
	Node(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normal, Shape model, int threshold);
	~Node();
};

class Tree
{
private:
	Node *root;

public:
	Tree(pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::Normal>::Ptr, int);
	~Tree();

	
	
};

void UpdateInliers(pcl::PointIndices::Ptr **inliers, pcl::PointIndices indices_points_inside_hole);

void DisplayCoefficients(Shape model, pcl::PointIndices::Ptr inliers, pcl::ModelCoefficients::Ptr coefficients);

bool CheckPiecesInCloudRemainder(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_remainder);

pcl::PointCloud<pcl::PointXYZ>::Ptr FlattenPatches(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Shape model, pcl::ModelCoefficients::Ptr coefficients, float *max_gap);

int ProjectInliersOnTheModel(pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud, Shape model, pcl::ModelCoefficients::Ptr coefficients);

int RecognizeAndSegment(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normal, Shape model,
	pcl::PointIndices::Ptr *inliers, pcl::ModelCoefficients::Ptr *coefficients);

void DestroyTree(Node *leaf);

pcl::PointCloud<pcl::PointXYZ>::Ptr ExtractCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointIndices::Ptr inliers, bool inside_or_outside);

pcl::PointCloud<pcl::Normal>::Ptr ExtractNormal(pcl::PointCloud<pcl::Normal>::Ptr normal, pcl::PointIndices::Ptr inliers, bool inside_or_outside);

int ExtractCloudAndNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normal, pcl::PointIndices::Ptr inliers,
	pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud_result, pcl::PointCloud<pcl::Normal>::Ptr *normal_result, bool inside_or_outside);

int ExtractCloudAndNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normal, pcl::PointIndices::Ptr inliers,
	pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud_inside, pcl::PointCloud<pcl::Normal>::Ptr *normal_inside,
	pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud_outside, pcl::PointCloud<pcl::Normal>::Ptr *normal_outside);


void CreateTree(Node *node, int threshold_inliers);

pcl::PointCloud<pcl::PointXYZ>::Ptr ExtractCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointIndices::Ptr inliers, bool inside_or_outside);

bool SortPolygonList(const Polygon_2& lhs, const Polygon_2& rhs);

//void FlattenInlierPointsBasedOnModel(pcl::PointCloud<pcl::PointXYZ>::Ptr *flattened_cloud, Shape model,
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inlier, pcl::ModelCoefficients::Ptr coefficients);

Polygon_2 CheckHoleInside(Polygon_2 polygon_max_area, Polygon_list *polygon_list);

void GiveBackOtherPiecesToCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud_inlier, pcl::PointCloud<pcl::Normal>::Ptr *cloud_inlier_normals,
	pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud_remainder, pcl::PointCloud<pcl::Normal>::Ptr *cloud_remainder_normals,
	pcl::PointCloud<pcl::PointXYZ>::Ptr *flattened_cloud, Polygon_2 polygon_max_area);

void FillHole(Polygon_2 hole, pcl::PointCloud<pcl::PointXYZ>::Ptr flattened_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud_inlier,
	pcl::PointCloud<pcl::Normal>::Ptr *cloud_inlier_normals, pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud_remainder,
	pcl::PointCloud<pcl::Normal>::Ptr *cloud_remainder_normals, pcl::PointIndices::Ptr *inliers);



pcl::PointIndices GetIndicesOfPointsInsideAndOnPolygon(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Polygon_2 polygon);

pcl::PointIndices GetIndicesOfPointsOnPolygon(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Polygon_2 polygon);

pcl::PointIndices GetIndicesOfPointsInsidePolygon(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Polygon_2 polygon);

pcl::PointCloud<pcl::PointXYZ>::Ptr GetCloudBasedOnIndices(pcl::PointIndices indices, pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud);

pcl::PointCloud<pcl::PointXYZ>::Ptr GetProjectedCloud(Shape shape, pcl::ModelCoefficients::Ptr coefficients, pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud);

void MovePartsFromCloudToCloud(pcl::PointIndices indices, pcl::PointCloud<pcl::PointXYZ>::Ptr **cloud_from, pcl::PointCloud<pcl::Normal>::Ptr **normal_from,
	pcl::PointCloud<pcl::PointXYZ>::Ptr **cloud_to, pcl::PointCloud<pcl::Normal>::Ptr **normal_to);









#endif