#include "PostureCriteriaToeOffBoundary.h"
#include "world/World.h"
#include "kinematic/Tree.h"
#include "kinematic/Robot.h"

PostureCriteriaToeOffBoundary::PostureCriteriaToeOffBoundary()
{
	// NOTHING
}

PostureCriteriaToeOffBoundary::~PostureCriteriaToeOffBoundary()
{
	// NOTHING
}

bool PostureCriteriaToeOffBoundary::Evaluate(const World& world, const Robot& robot, const Tree& tree) const
{
	//return !(world.IsReachable(robot, tree, tree.GetTarget()));
	//return (!(world.IsReachable(robot, tree, tree.GetTarget())) || ( tree.targetReached_ && world.IsColliding(robot, tree)));
	return (!(world.IsReachable(robot, tree, tree.GetTarget())) || ( tree.targetReached_ && world.IsSoftColliding(robot, tree)));
}
