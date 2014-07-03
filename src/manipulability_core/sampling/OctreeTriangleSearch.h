#ifndef PCL_OCTREE_TRIANGLE_SEARCH_H_
#define PCL_OCTREE_TRIANGLE_SEARCH_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <queue>
#include <vector>

#include "Objects3D\Wm5Triangle3.h"
#include "Objects3D\Wm5Box3.h"
#include "Intersection\Wm5Intersector.h"
#include "Intersection\Wm5IntrTriangle3Box3.h"
#include "Distance\Wm5DistPoint3Triangle3.h"
#include "Wm5CoreLIB.h"

#include "Triangle3Df.h"
#include "pcl/octree/octree_pointcloud.h"

#include "Octree.h"

const int debug = 0;
using namespace std;
namespace pcl
{
	namespace octree
	{
		template<typename PointT, typename LeafContainerT = OctreeContainerPointIndices, typename BranchContainerT = OctreeContainerEmpty >
		class OctreeTriangleSearch : public OctreePointCloud<PointT, LeafContainerT, BranchContainerT>
		{
			typedef boost::shared_ptr<std::vector<int> > IndicesPtr;
			typedef boost::shared_ptr<const std::vector<int> > IndicesConstPtr;

			typedef pcl::PointCloud<PointT> PointCloud;
			typedef boost::shared_ptr<PointCloud> PointCloudPtr;
			typedef boost::shared_ptr<const PointCloud> PointCloudConstPtr;

			// Boost shared pointers
			typedef boost::shared_ptr<OctreePointCloudSearch<PointT, LeafContainerT, BranchContainerT> > Ptr;
			typedef boost::shared_ptr<const OctreePointCloudSearch<PointT, LeafContainerT, BranchContainerT> > ConstPtr;

			// Eigen aligned allocator
			typedef std::vector<PointT, Eigen::aligned_allocator<PointT> > AlignedPointTVector;

			typedef OctreePointCloud<PointT, LeafContainerT, BranchContainerT> OctreeT;
			typedef typename OctreeT::LeafNode LeafNode;
			typedef typename OctreeT::BranchNode BranchNode;
			
			typedef struct NodeTuple{
				unsigned int depth;
				OctreeKey octreeKey;
				BranchNode* node;
			} ;		
			typedef boost::shared_ptr<NodeTuple> NodeTuple_Ptr;
		protected:
			queue <NodeTuple_Ptr> nodeQueue_;

		public:
			OctreeTriangleSearch(const double resolution) :
				OctreePointCloud<PointT, LeafContainerT, BranchContainerT>(resolution)
			{
			}

			virtual ~OctreeTriangleSearch()
			{
			} 
			
			bool TriangleSearch(const manip_core::Triangle3Df obstacle, vector<size_t>& point_idx_data, float radius){
				// Push root node
				BranchNode * root = this->root_node_;
				unsigned int root_depth = 1;

				// Generate new key
				OctreeKey key;
				key.x = key.y = key.z = 0;

				NodeTuple_Ptr ntuple(new NodeTuple());
				ntuple->depth = root_depth;
				ntuple->octreeKey = key;
				ntuple->node = root;
				nodeQueue_.push(ntuple);

				while (!nodeQueue_.empty()){
					NodeTuple_Ptr ntuple = (nodeQueue_.front());

					unsigned int tree_depth = ntuple->depth;
					OctreeKey key = ntuple->octreeKey;
					BranchNode * node = ntuple->node;
					
					nodeQueue_.pop();
					
					// iterate the tree leaves
					for (unsigned char child_idx = 0; child_idx < 8; child_idx++){
						const OctreeNode* child_node = this->getBranchChildPtr(*node, child_idx);

						if (!child_node){
							continue;
						}

						OctreeKey new_key;
						
						// generate new key for current branch voxel
						new_key.x = (key.x << 1) + (!!(child_idx & (1 << 2)));
						new_key.y = (key.y << 1) + (!!(child_idx & (1 << 1)));
						new_key.z = (key.z << 1) + (!!(child_idx & (1 << 0)));

						// Voxel corners
						Eigen::Vector3f lower_voxel_corner;
						Eigen::Vector3f upper_voxel_corner;
						
						// get voxel coordinates
						this->genVoxelBoundsFromOctreeKey(new_key, tree_depth, lower_voxel_corner, upper_voxel_corner);

						// Gen Triangle and Box
						// Triangle
						Wm5::Vector3<float> a(obstacle.a_.x(), obstacle.a_.y(), obstacle.a_.z());
						Wm5::Vector3<float> b(obstacle.b_.x(), obstacle.b_.y(), obstacle.b_.z());
						Wm5::Vector3<float> c(obstacle.c_.x(), obstacle.c_.y(), obstacle.c_.z());

						Wm5::Triangle3<float> triangle(a, b, c);
						
						// Box
						Wm5::Vector3<float> center
							((0.5) * (lower_voxel_corner.x() + upper_voxel_corner.x()),
							(0.5) * (lower_voxel_corner.y() + upper_voxel_corner.y()),
							(0.5) * (lower_voxel_corner.z() + upper_voxel_corner.z()));

						float extent[3] = { ((0.5) * (upper_voxel_corner.x() - lower_voxel_corner.x()),
							(0.5) * (upper_voxel_corner.y() - lower_voxel_corner.y()),
							(0.5) * (upper_voxel_corner.z() - lower_voxel_corner.z())) };
						Wm5::Vector3<float> axis[3] = {
							Wm5::Vector3<float>(1.0, 0.0, 0.0),
							Wm5::Vector3<float>(0.0, 1.0, 0.0),
							Wm5::Vector3<float>(0.0, 0.0, 1.0) };
						Wm5::Box3<float> box(center, axis, extent);
						
						if (debug == 1){
							cout << "Center: " << center << endl;;
							cout << "Triangle: " << endl;
							cout << obstacle.a_ << endl;
							cout << obstacle.b_ << endl;
							cout << obstacle.c_ << endl;
							cout << "Distance: " << endl;
							cout << Wm5::DistPoint3Triangle3<float>(center, triangle).Get() << endl;
							cout << "Cutoff: " << endl;
							cout << radius + max({ extent[0], extent[1], extent[2] }) << endl;
						}

						if (Wm5::DistPoint3Triangle3<float>(center, triangle).Get() > radius + max({ extent[0], extent[1], extent[2] })){
							
							continue;
						}


						if (child_node->getNodeType() == LEAF_NODE){
							// Get points
							vector<int> points;
							const LeafNode* child_leaf = static_cast<const LeafNode*> (child_node);

							(**child_leaf).getPointIndices(points);

							for (int i = 0; i < points.size(); i++){
								const PointT& pclPoint = this->getPointByIndex(points[i]);

								Wm5::Vector3<float> wmpoint(pclPoint.x, pclPoint.y, pclPoint.z);
								Wm5::Triangle3<float> wmtriangle(*(new Wm5::Vector3<float>(obstacle.a_.x(), obstacle.a_.y(), obstacle.a_.z())),
									*(new Wm5::Vector3<float>(obstacle.b_.x(), obstacle.b_.y(), obstacle.b_.z())),
									*(new Wm5::Vector3<float>(obstacle.c_.x(), obstacle.c_.y(), obstacle.c_.z())));

								Wm5::DistPoint3Triangle3<float> distCalc(wmpoint, wmtriangle);
								float dist = distCalc.Get();

								if (dist < radius){
									point_idx_data.push_back(points[i]);
								}
							}
						}
						else if (child_node->getNodeType() == BRANCH_NODE){
							NodeTuple_Ptr ntuple(new NodeTuple());
							ntuple->depth = tree_depth + 1;
							ntuple->octreeKey = new_key;
							ntuple->node = (BranchNode*)(child_node);

							nodeQueue_.push(ntuple);
						}
					}
				}
				cout << point_idx_data.size();
				return true;
			}
		
		};
	}
}
#endif