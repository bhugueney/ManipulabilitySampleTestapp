
#ifndef _CLASS_POSTURE_CRITERIA_ABC
#define _CLASS_POSTURE_CRITERIA_ABC

class Robot;
class Tree;
class World;


class PostureCriteria_ABC
{

public:
	 PostureCriteria_ABC();
	~PostureCriteria_ABC();

public:
	virtual bool Evaluate(const World& /*world*/, const Robot& /*robot*/, const Tree& /*tree*/) const = 0;
};


#endif //_CLASS_POSTURE_CRITERIA_ABC