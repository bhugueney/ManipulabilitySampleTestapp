
#ifndef _CLASS_OBSTACLECONSTRAINT
#define _CLASS_OBSTACLECONSTRAINT

#include "PartialDerivativeConstraint.h"
#include "world/World.h"

#include "MatrixDefs.h"

class Jacobian;

class ObstacleConstraint : public PartialDerivativeConstraint
{

public:
	 ObstacleConstraint(const World& /*world*/);
	~ObstacleConstraint();

public:
	virtual NUMBER Evaluate(const Robot& /*robot*/, const Tree& /*tree*/, const int joint, Jacobian& /*jacobianMinus*/, Jacobian& /*jacobianPlus*/, float /*epsilon*/, const matrices::Vector3& /*direction*/);

private:
	const World& world_;
};

#endif //_CLASS_OBSTACLECONSTRAINT