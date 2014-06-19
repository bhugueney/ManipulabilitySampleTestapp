
#ifndef _CLASS_POSTURECONSTRAINT
#define _CLASS_POSTURECONSTRAINT

#include "PartialDerivativeConstraint.h"

#include "MatrixDefs.h"

class Jacobian;

class PostureConstraint : public PartialDerivativeConstraint
{

public:
	 PostureConstraint();
	~PostureConstraint();

public:
	virtual NUMBER Evaluate(const Robot& /*robot*/, const Tree& /*tree*/, const int joint, Jacobian& /*jacobianMinus*/, Jacobian& /*jacobianPlus*/, float /*epsilon*/, const matrices::Vector3& /*direction*/);

};

#endif //_CLASS_POSTURECONSTRAINT