
#ifndef _CLASS_POSTURE_CRITERIA_TOE_ON_COM
#define _CLASS_POSTURE_CRITERIA_TOE_ON_COM

#include "PostureCriteria_ABC.h"

class Robot;
class Tree;
class World;


class PostureCriteriaToeOnCOM : public PostureCriteria_ABC
{

public:
	 PostureCriteriaToeOnCOM();
	~PostureCriteriaToeOnCOM();

public:
	virtual bool Evaluate(const World& /*world*/, const Robot& /*robot*/, const Tree& /*tree*/) const;
}; //PostureCriteriaToeOffBoundary


#endif //_CLASS_POSTURE_CRITERIA_TOE_ON_COM