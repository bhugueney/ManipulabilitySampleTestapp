
#ifndef _CLASS_IKSOLVER
#define _CLASS_IKSOLVER

#include "MatrixDefs.h"

#include <memory>

class Tree;
class Jacobian;
class PartialDerivativeConstraint;

struct IKPImpl;

class IKSolver {

public:
	 IKSolver(const float espilon = 0.01f, const float treshold = 0.03f);
	~IKSolver();

public:
	bool StepClamping(Tree& /*tree*/, const matrices::Vector3& /*target*/, const matrices::Vector3& /*direction*/ ) const; //true if target reached //target in robot coordinates
	bool QuickStepClamping(Tree& /*tree*/, const matrices::Vector3& /*target*/) const; //true if target reached // target in robot coordinates
	void PartialDerivatives(Tree& /*tree*/, const matrices::Vector3& /*direction*/, matrices::VectorX& /*velocities*/) const;

public:
	void Register(PartialDerivativeConstraint* constraint);

private:
	std::auto_ptr<IKPImpl> pImpl_;

	void PartialDerivative (Tree& /*tree*/, const matrices::Vector3& /*direction*/, matrices::VectorX& /*velocities*/, const int /*joint*/) const;

	const float epsilon_;
	const float treshold_;
};

#endif //_CLASS_IKSOLVER