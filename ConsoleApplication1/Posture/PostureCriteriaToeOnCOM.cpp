#include "PostureCriteriaToeOnCOM.h"

#include "skeleton\Tree.h"
#include "skeleton\Robot.h"
#include "world\World.h"
#include "tools/matrixDefs.h"
#include "support/supportpolygon.h"
using namespace manip_core;
using namespace matrices;


PostureCriteriaToeOnCOM::PostureCriteriaToeOnCOM()
{
	// NOTHING
}

PostureCriteriaToeOnCOM::~PostureCriteriaToeOnCOM()
{
	// NOTHING
}

bool PostureCriteriaToeOnCOM::Evaluate(const manip_core::World& world, const manip_core::Robot& robot, const manip_core::Tree& tree) const
{
	// first let's compute current's com
	/*
	const matrices::Vector3 com = robot.ComputeCom();
	SupportPolygon support(robot);
	if (!support.Contains(com))// && support.WouldContain(com, robot, tree))
	{
		return true;
	}*/
	return false; // TODO
}
