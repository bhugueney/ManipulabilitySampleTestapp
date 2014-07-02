#include "Octree.h"
#include "Distance\Wm5DistPoint3Triangle3.h"
#include "Algebra\Wm5Vector3.h"
#include "Objects3D\Wm5Triangle3.h"
#include "OctreeCustomBFSIterator.h"
#include "Wm5CoreLIB.h"
#include <queue>

using namespace manip_core;
using namespace std;

typedef pcl::octree::OctreeNode mNode;
typedef pcl::octree::OctreeBranchNode<pcl::PointXYZ> mBranchNode;
typedef pcl::octree::OctreeLeafNode<pcl::PointXYZ> mLeafNode;

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

#include<fstream>
#include<string>
using namespace std;

void Octree::init(){
	
	
	octree.setInputCloud(cloud);
	octree.addPointsFromInputCloud();
}

T_Id Octree::search(const Triangle3Df & obstacle, float radius){
	T_Id res;

	pcl::octree::OctreeBreadthFirstIterator<pcl::octree::OctreePointCloud<pcl::PointXYZ>::OctreeT> b = octree.breadth_begin();
	
	pcl::octree::OctreeCustomBFSIterator<pcl::octree::OctreePointCloud<pcl::PointXYZ>::OctreeT, pcl::PointXYZ> it(b);
	
	vector<int> points;
	octree.TriangleSearch(obstacle, points);

	/*
	queue <mNode *> nodeQueue;
	mNode * startNode = octree.begin().getCurrentOctreeNode();

	nodeQueue.push(startNode);

	ofstream oFile("pcldw.txt");
		
	
	while (!nodeQueue.empty()){
		mNode * node = nodeQueue.front();
		nodeQueue.pop();

		if (node->getNodeType() == pcl::octree::BRANCH_NODE){
			
			mBranchNode * curNode = (mBranchNode *)node;
			cout << curNode << endl;
			
			// calc bounds
			curNode->
			pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ> oc (1);
			
			//cout << curNode->getContainer().x << ", " << curNode->getContainer().y << ", " << curNode->getContainer().z << endl;
			for (int i = 0; i < 8; i++){
				if (curNode->hasChild(i)){
					nodeQueue.push((*curNode)[i]);
				}
			}
		}
	}
	oFile.close();
	*/

	/*
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
		}*/
		//cout << res.size() << endl;	
	return res;
}