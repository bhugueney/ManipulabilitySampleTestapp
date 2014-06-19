#ifndef _CLASS_OCTREE
#define _CLASS_OCTREE

#include "MatrixDefs.h"
#include "pcl\point_cloud.h"
#include "pcl\octree\octree.h"

using namespace matrices;

typedef std::vector<size_t> T_Id;

namespace manip_core
{
	class Octree {
	public:
		Octree();
		~Octree();
	public:
		int insertToCloud(const Vector3 & /*point*/);
		void init();

		T_Id search(const Vector3 & /*obstacle*/, float /*radius*/);
	private:
		const float resolution = 128.0f;

	private:
		pcl::PointCloud <pcl::PointXYZ>::Ptr cloud;
		pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree;
	};
}

#endif //_CLASS_OCTREE