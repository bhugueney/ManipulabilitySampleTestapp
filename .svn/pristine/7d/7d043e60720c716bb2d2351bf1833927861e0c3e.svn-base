
#ifndef _CLASS_POSTURE_CRITERIA_TOE_OFF_GAIT
#define _CLASS_POSTURE_CRITERIA_TOE_OFF_GAIT

#include "PostureCriteria_ABC.h"
#include "SpiderGaitI.h"

class Robot;
class Tree;
class World;


class PostureCriteriaToeOffGait : public PostureCriteria_ABC
{

public:
	 PostureCriteriaToeOffGait(SpiderGaitI* /*gait*/);
	~PostureCriteriaToeOffGait();

public:
	virtual bool Evaluate(const World& /*world*/, const Robot& /*robot*/, const Tree& /*tree*/) const;

private:
	SpiderGaitI* gait_;
}; //PostureCriteriaToeOffBoundary


#endif //_CLASS_POSTURE_CRITERIA_TOE_OFF_GAIT