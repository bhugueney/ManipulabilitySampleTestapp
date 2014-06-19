#include "Octree.h"

using namespace manip_core;

Octree::Octree()
: octree(resolution)
, cloud(new pcl::PointCloud<pcl::PointXYZ>){
	// nothing
}

Octree::~Octree(){
	cloud->~PointCloud();
}

int Octree::insertToCloud(const Vector3 &point){
	pcl::PointXYZ pointXYZ;
	pointXYZ.x = point.x();
	pointXYZ.y = point.y();
	pointXYZ.z = point.z();
	cloud->push_back(pointXYZ);

	return cloud->size();
}

void Octree::init(){
	octree.setInputCloud(cloud);
	octree.addPointsFromInputCloud();
}
#include <iostream>
using namespace std;
T_Id Octree::search(const Vector3 & obstacle, float radius){
	T_Id res;
	pcl::PointXYZ searchPoint(obstacle.x(), obstacle.y(), obstacle.z());

	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

	if (octree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
	{
		for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i){
			res.push_back(i);
		}
	}

	return res;
}