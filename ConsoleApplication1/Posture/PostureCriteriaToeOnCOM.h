

#include "PostureCriteria_ABC.h"
#include "world\World.h"
#include "skeleton\Robot.h"
#include "skeleton\Tree.h"


class PostureCriteriaToeOnCOM : public PostureCriteria_ABC
{

public:
	PostureCriteriaToeOnCOM();
	~PostureCriteriaToeOnCOM();

public:
	virtual bool Evaluate(const manip_core::World& /*world*/, const manip_core::Robot& /*robot*/, const manip_core::Tree& /*tree*/) const;
}; //PostureCriteriaToeOffBoundary

