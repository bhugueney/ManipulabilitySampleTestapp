
#ifndef _CLASS_PDCONSTRAINT
#define _CLASS_PDCONSTRAINT

#include "MatrixDefs.h"

class Jacobian;
class Tree;
class Robot;

class PartialDerivativeConstraint {

public:
	 PartialDerivativeConstraint();
	~PartialDerivativeConstraint();

public:
	virtual NUMBER Evaluate(const Robot& /*robot*/, const Tree& /*tree*/, const int joint, Jacobian& /*jacobianMinus*/, Jacobian& /*jacobianPlus*/, float /*epsilon*/, const matrices::Vector3& /*direction*/) = 0;
};

#endif //_CLASS_PDCONSTRAINT