#ifndef _CLASS_OCTREE
#define _CLASS_OCTREE

#include "MatrixDefs.h"

#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>

#include "Triangle3Df.h"
#include "OctreeTriangleSearch.h"

using namespace matrices;

namespace manip_core
{

	typedef std::vector<size_t> T_Id;
	class Octree {
	public:
		Octree();
		~Octree();
	public:
		int insertToCloud(const Vector3 & /*point*/);
		void init();
		
		T_Id search(const Triangle3Df & /*obstacle*/, float /*radius*/);
	private:
		const float resolution = 0.08f;

	private:
		pcl::PointCloud <pcl::PointXYZ>::Ptr cloud;
		pcl::octree::OctreeTriangleSearch<pcl::PointXYZ> octree;
	};
}

#endif //_CLASS_OCTREE