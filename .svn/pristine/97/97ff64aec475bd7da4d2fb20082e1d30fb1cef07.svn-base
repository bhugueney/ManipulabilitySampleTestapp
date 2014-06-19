
#ifndef _CLASS_FILTER_COLLISION
#define _CLASS_FILTER_COLLISION

#include <memory>

#include "Filter_ABC.h"
#include "MatrixDefs.h"

class Sample;
class Obstacle;
class Robot;
class Tree;
class World;

struct FilterCollisionPImpl;

// checks that end-effector is "around" obstacle
class FilterCollision : public Filter_ABC {

public:
	 EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	 FilterDistance(NUMBER /*treshold*/, const World& /*world*/, const Tree& /*tree*/, const Obstacle& /*obstacle*/, const Robot& /*robot*/); // in robot coordinates
	~FilterDistance();

protected:
	virtual bool ApplyFilter(const Sample& /*sample*/) const;

private:
	std::auto_ptr<FilterCollisionPImpl> pImpl_;
};


#endif //_CLASS_FILTER_COLLISION