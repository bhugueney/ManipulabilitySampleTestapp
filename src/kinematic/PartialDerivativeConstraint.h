
#ifndef _CLASS_PDCONSTRAINT
#define _CLASS_PDCONSTRAINT

#include "MatrixDefs.h"

class Jacobian;

class PartialDerivativeConstraint {

public:
	 PartialDerivativeConstraint();
	~PartialDerivativeConstraint();

public:
	virtual NUMBER Evaluate(Jacobian& /*jacobianMinus*/, Jacobian& /*jacobianPlus*/, float /*epsilon*/, const matrices::Vector3& /*direction*/) = 0;
};

#endif //_CLASS_PDCONSTRAINT