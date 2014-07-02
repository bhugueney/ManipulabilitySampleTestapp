#ifndef PCL_OCTREE_CUSTOMBFS_ITERATOR_
#define PCL_OCTREE_CUSTOMBFS_ITERATOR_

#include <pcl/common/common.h>
#include "pcl\octree\octree_iterator.h"
#include "Objects3D\Wm5Triangle3.h"


namespace pcl {
	namespace octree{
		template <typename OctreeT, typename ContainerT>
		class OctreeCustomBFSIterator : public OctreeBreadthFirstIterator<OctreeT>{
		public:
			inline OctreeCustomBFSIterator(const OctreeBreadthFirstIterator<OctreeT>& it)
				:OctreeBreadthFirstIterator<OctreeT>(it){
					this->reset();
				};

			inline ~OctreeCustomBFSIterator() {};

			inline int next(){
				if (FIFO_.size())
				{
					// get stack element
					IteratorState FIFO_entry = FIFO_.front();
					FIFO_.pop_front();

					FIFO_entry.depth_++;
					OctreeKey& current_key = FIFO_entry.key_;
					
					OctreeNode * n = FIFO_entry.node_;
					cout << n << endl;
					if ((this->max_octree_depth_ >= FIFO_entry.depth_) &&
						(FIFO_entry.node_->getNodeType() == BRANCH_NODE))
					{
						unsigned char child_idx;
						
						// current node is a branch node
						OctreeBranchNode<ContainerT>* current_branch =
							static_cast<OctreeBranchNode<ContainerT>*> (FIFO_entry.node_);

						// iterate over all children
						for (child_idx = 0; child_idx < 8; ++child_idx)
						{

							// if child exist
							if (current_branch->hasChild(child_idx))
							{
								// add child to stack
								current_key.pushBranch(static_cast<unsigned char> (child_idx));

								//FIFO_entry.node_ = this->octree_->getBranchChildPtr(*current_branch, child_idx);
								FIFO_entry.node_ = &current_branch[child_idx];

								FIFO_.push_back(FIFO_entry);

								current_key.popBranch();
							}
						}
					}

					if (FIFO_.size())
					{
						this->current_state_ = &FIFO_.front();
						return 1;
					}
					else
					{
						this->current_state_ = 0;
						return 0;
					}

				}
			}
				//void next(const Triangle3Df & obstacle);
		private:
			OctreeCustomBFSIterator();
		};


	}
}
#endif