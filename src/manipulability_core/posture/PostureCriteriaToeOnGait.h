
#ifndef _CLASS_POSTURE_CRITERIA_TOE_ON_GAIT
#define _CLASS_POSTURE_CRITERIA_TOE_ON_GAIT

#include "PostureCriteria_ABC.h"
#include "SpiderGaitI.h"

class Robot;
class Tree;
class World;


class PostureCriteriaToeOnGait : public PostureCriteria_ABC
{

public:
	 PostureCriteriaToeOnGait(SpiderGaitI* /*gait*/);
	~PostureCriteriaToeOnGait();

public:
	virtual bool Evaluate(const World& /*world*/, const Robot& /*robot*/, const Tree& /*tree*/) const;

private:
	SpiderGaitI* gait_;
}; //PostureCriteriaToeOffBoundary


#endif //_CLASS_POSTURE_CRITERIA_TOE_ON_GAIT