
#ifndef _CLASS_JLCONSTRAINT
#define _CLASS_JLCONSTRAINT

#include "PartialDerivativeConstraint.h"

#include "MatrixDefs.h"

class Jacobian;

class JointLimitConstraint : public PartialDerivativeConstraint
{

public:
	 JointLimitConstraint();
	~JointLimitConstraint();

public:
	virtual NUMBER Evaluate(Jacobian& /*jacobianMinus*/, Jacobian& /*jacobianPlus*/, float /*epsilon*/, const matrices::Vector3& /*direction*/);

private:
};

#endif //_CLASS_JLCONSTRAINT