
#include "ForceManipulabilityConstraint.h"

#include "kinematic/Jacobian.h"

using namespace matrices;
using namespace Eigen;

ForceManipulabilityConstraint::ForceManipulabilityConstraint()
{
	// NOTHING
}

ForceManipulabilityConstraint::~ForceManipulabilityConstraint()
{
	// NOTHING
}

NUMBER ForceManipulabilityConstraint::Evaluate(Jacobian& jacobianMinus, Jacobian& jacobianPlus, float epsilon, const Vector3& direction)
{
 	return NUMBER ((ForceManipulability(jacobianPlus, direction) - ForceManipulability(jacobianMinus, direction)) / (epsilon * 2) * 0.3) ;
}
NUMBER ForceManipulabilityConstraint::ForceManipulability(Jacobian& jacobian, const matrices::Vector3& direction)
{ 
	NUMBER r = ((direction).transpose()*jacobian.GetJacobianProduct()*(direction));
	return 1/sqrt(r); 
}


