
#ifndef _CLASS_POSTURE_CRITERIA_TOE_OFF_BOUNDARY
#define _CLASS_POSTURE_CRITERIA_TOE_OFF_BOUNDARY

#include "PostureCriteria_ABC.h"

class Robot;
class Tree;
class World;


class PostureCriteriaToeOffBoundary : public PostureCriteria_ABC
{

public:
	 PostureCriteriaToeOffBoundary();
	~PostureCriteriaToeOffBoundary();

public:
	virtual bool Evaluate(const World& /*world*/, const Robot& /*robot*/, const Tree& /*tree*/) const;
}; //PostureCriteriaToeOffBoundary


#endif //_CLASS_POSTURE_CRITERIA_TOE_OFF_BOUNDARY