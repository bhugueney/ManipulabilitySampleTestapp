
#include "FilterCollision.h"
#include "ObstacleVisitor_ABC.h"

#include "Obstacle.h"
#include "Robot.h"
#include "Tree.h"
#include "Sample.h"

#include <math.h>

using namespace matrices;
using namespace Eigen;

struct FilterCollisionPImpl
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	FilterDPImpl(NUMBER treshold, const World& world, const Tree& tree, const Obstacle& obstacle, const Robot& robot)
	: treshold_(treshold)
	, treePos_(tree.GetPosition())
	, obstacle_(obstacle)
	, world_(world)
	{
		//toObstacleCoordinates_ = robot.ToWorldCoordinates() * obstacle.BasisInv();
		toObstacleCoordinates_ = obstacle.BasisInv() * robot.ToWorldCoordinates();
		obsW_ = obstacle.GetW(); obsH_ = obstacle.GetH();
	}

	~FilterDPImpl()
	{
		// NOTHING
	}
	
	NUMBER treshold_;
	const World& world_;
	const Obstacle& obstacle_;
	Matrix4 toObstacleCoordinates_;
	Vector3 treePos_;
	NUMBER obsW_, obsH_;

};

Struct ObstacleCollisionVisitor : public ObstacleVisitor_ABC
{
	ObstacleCollisionVisitor(const Obstacle& obstacle, const Sample& sample)
		: obstacle_(obstacle)
		, ok_(true)
		, sample_(sample)
	{
		// NOTHING
	}

	~ObstacleCollisionVisitor()
	{
		// NOTHING
	}
	virtual void Visit(const Obstacle& obstacle)
	{
		if(obstacle != obstacle_)
		{

		}
	}
	bool ok_;
	const Obstacle& obstacle_;
	const Sample& sample_;
};

FilterCollision::FilterCollision(NUMBER treshold, const World& world, const Tree& tree, const Obstacle& obstacle, const Robot& robot)
	: Filter_ABC()
	, pImpl_(new FilterCollisionPImpl(treshold, world, tree, obstacle, robot))
{
	// NOTHING
}

FilterCollision::~FilterCollision()
{
	// NOTHING
}

bool FilterCollision::ApplyFilter(const Sample& sample) const
{
	// check that sample position is inside obstacle radius and not too far from its plan
	if(sample.GetPosition()(0) <=0 ) // we want to move forward
	{
		return false;
	}
	Vector3 center = matrix4TimesVect3(pImpl_->toObstacleCoordinates_, sample.GetPosition() + pImpl_->treePos_);
	NUMBER xc, yc;
	xc = center(0); yc = center(1);
	return (abs(center(2)) <= pImpl_->treshold_) && (xc >= 0 && xc <= pImpl_->obsW_) && (yc >= 0 && yc <= pImpl_->obsH_);
}

