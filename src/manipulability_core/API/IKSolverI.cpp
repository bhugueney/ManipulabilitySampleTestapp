#include "IKSolverI.h"
#include "IK/IKSolver.h"
#include "IK/ForceManipulabilityConstraint.h"
using namespace manip_core;

extern "C" MANIPCORE_API IKSolverI* GetIKSolver()
{
	IKSolver* iksolve = new IKSolver;
	//iksolve->Register(new ForceManipulabilityConstraint);
	return iksolve;
}
