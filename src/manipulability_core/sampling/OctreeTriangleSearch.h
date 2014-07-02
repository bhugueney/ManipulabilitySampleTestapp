#ifndef PCL_OCTREE_TRIANGLE_SEARCH_H_
#define PCL_OCTREE_TRIANGLE_SEARCH_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>
#include "Triangle3Df.h"
#include "pcl/octree/octree_pointcloud.h"

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
		protected:
			queue <BranchNode*> nodeQueue_;

		public:
			OctreeTriangleSearch(const double resolution) :
				OctreePointCloud<PointT, LeafContainerT, BranchContainerT>(resolution)
			{
			}

			virtual ~OctreeTriangleSearch()
			{
			}
			
			bool TriangleSearch(const manip_core::Triangle3Df obstacle, std::vector<int>& point_idx_data){
				const BranchNode * root = this->root_node_;
				nodeQueue_.push(root);
				while (!nodeQueue_.empty()){
					const BranchNode * node = nodeQueue_.front();
					nodeQueue_.pop();

					// iterate the tree leaves
					for (unsigned char child_idx = 0; child_idx < 8; child_idx++){
						const OctreeNode* child_node = this->getBranchChildPtr(*node, child_idx);

						if (!child_node)
							continue;


					}
				}
				return true;
			}
		
		};
	}
}
#endif