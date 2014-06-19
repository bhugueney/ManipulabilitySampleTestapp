
#ifndef _CLASS_JOINTCONSTRAINT
#define _CLASS_JOINTCONSTRAINT

#include "PartialDerivativeConstraint.h"

#include "MatrixDefs.h"

class Jacobian;

class JointConstraint : public PartialDerivativeConstraint
{

public:
	 JointConstraint();
	~JointConstraint();

public:
	virtual NUMBER Evaluate(const Robot& /*robot*/, const Tree& /*tree*/, const int joint, Jacobian& /*jacobianMinus*/, Jacobian& /*jacobianPlus*/, float /*epsilon*/, const matrices::Vector3& /*direction*/);

};

#endif //_CLASS_JOINTCONSTRAINT