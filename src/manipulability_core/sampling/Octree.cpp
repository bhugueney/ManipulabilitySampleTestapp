#include "Octree.h"
#include "Distance\Wm5DistPoint3Triangle3.h"
#include "Algebra\Wm5Vector3.h"
#include "Objects3D\Wm5Triangle3.h"

#include "Wm5CoreLIB.h"
#include <vector>

#pragma comment(lib,"user32.lib") 
using namespace manip_core;
using namespace std;

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

T_Id Octree::search(const Triangle3Df & obstacle, float radius){
	T_Id res;
	
	pcl::octree::OctreePointCloud<pcl::PointXYZ>::DepthFirstIterator tree_depth_it;
	pcl::octree::OctreePointCloud<pcl::PointXYZ>::DepthFirstIterator  tree_depth_it_end = octree.depth_end();

	for (tree_depth_it = octree.depth_begin(); tree_depth_it != tree_depth_it_end; ++tree_depth_it)
	{
		if (tree_depth_it.isLeafNode()){
			vector<int> points;
			tree_depth_it.getLeafContainer().getPointIndices(points);
			for (int i = 0; i < points.size(); i++){
				pcl::PointXYZ pclPoint = cloud->at(points[i]);

				Wm5::Vector3<float> wmpoint(pclPoint.x,pclPoint.y,pclPoint.z);
				Wm5::Triangle3<float> wmtriangle(*(new Wm5::Vector3<float>(obstacle.a_.x(), obstacle.a_.y(), obstacle.a_.z())),
					*(new Wm5::Vector3<float>(obstacle.b_.x(), obstacle.b_.y(), obstacle.b_.z())),
					*(new Wm5::Vector3<float>(obstacle.c_.x(), obstacle.c_.y(), obstacle.c_.z())));
				
				Wm5::DistPoint3Triangle3<float> distCalc(wmpoint, wmtriangle);
				float dist = distCalc.Get();
				
				if (dist < radius){
					res.push_back(points[i]);
				}
			}
		}
		cout << res.size() << endl;
	}	
	return res;
}