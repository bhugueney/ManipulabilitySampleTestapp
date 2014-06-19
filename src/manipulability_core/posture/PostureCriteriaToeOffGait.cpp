#include "PostureCriteriaToeOffGait.h"
#include "world/World.h"
#include "kinematic/Tree.h"
#include "kinematic/Robot.h"

PostureCriteriaToeOffGait::PostureCriteriaToeOffGait(SpiderGaitI* gait)
	: gait_(gait)
{
	// NOTHING
}

PostureCriteriaToeOffGait::~PostureCriteriaToeOffGait()
{
	// NOTHING
}

bool PostureCriteriaToeOffGait::Evaluate(const World& world, const Robot& robot, const Tree& tree) const
{
	int id =  tree.GetId();
	if(id == 2 || id == 3)
	{
		return tree.GetEffectorPosition(tree.GetNumEffector()-1)(0) < -0.5;
	}
	if(id == 5 || id == 0)
	{
		return tree.GetEffectorPosition(tree.GetNumEffector()-1)(0) < 0.2;
	}
	return false;
	//if( gait_->MidPhase() )return false;
	if (gait_->Phase() == 0)
	{
		return id == 5;// :: || id == 2 || id == 4 || id == 6;
	}
	else
	{
		return id == 0;// || id == 3 || id == 5 || id == 7;
	}
}
