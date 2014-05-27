#ifndef POSTURECRITERIA_ABC_H
#define POSTURECRITERIA_ABC_H

#include "world\World.h"
#include "skeleton\Robot.h"
#include "skeleton\Tree.h"

//class Robot;
//class Tree;
//class World;


class PostureCriteria_ABC
{

public:
	PostureCriteria_ABC();
	~PostureCriteria_ABC();

public:
	virtual bool Evaluate(const manip_core::World& /*world*/, const manip_core::Robot& /*robot*/, const manip_core::Tree& /*tree*/) const = 0;
};

#endif
