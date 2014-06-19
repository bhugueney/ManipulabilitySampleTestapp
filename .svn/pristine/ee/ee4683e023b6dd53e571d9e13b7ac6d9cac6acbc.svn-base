
#include "Jacobian.h"
#include "kinematic/Tree.h"
#include "kinematic/Joint.h"


using namespace matrices;
using namespace Eigen;

Jacobian::Jacobian(const Tree& tree)
{
	ComputeJacobian(tree);
}

Jacobian::~Jacobian()
{
	//NOTHING
}

void Jacobian::SetJacobian(const MatrixX& jacobian)
{
	jacobian_ = jacobian;
	Invalidate();
}

void Jacobian::Invalidate()
{
	computeInverse_ = true; computeProduct_ = true; computeProductInverse_ = true;
	computeJacSVD_ = true; computeNullSpace_ = true;
}

void Jacobian::ComputeJacobian(const Tree& tree)
{
	Invalidate();
	jacobian_ = MatrixX(3,tree.GetNumJoint()-1); // à cause de son incrémentation débile
	// Traverse this to find all end effectors
	Vector3 temp;
	Joint* n = tree.GetRoot();
	while (n) {	
		if (n->IsEffector())
		{
			int i = n->GetEffectorNum();
			// Find all ancestors (they will usually all be joints)
			// Set the corresponding entries in the Jacobian J
			Joint* m = tree.GetParent(n);
			while (m) {
				int j = m->GetJointNum();
				assert (0 <=i && i<tree.GetNumEffector() && 0<=j && j<tree.GetNumJoint());
				
				temp = m->GetS();			// joint pos.
				temp -= n->GetS();			// -(end effector pos. - joint pos.)
				Vector3 tmp2 = temp.cross(m->GetW());			// cross product with joint rotation axis
				jacobian_.col(j-1) = tmp2;
				m = tree.GetParent(m);
			}
		}
		n = (n->IsEffector() ? 0 : tree.GetSuccessor(n));
	}
}

void Jacobian::GetEllipsoidAxes(matrices::Vector3& u1, matrices::Vector3& u2, matrices::Vector3& u3)
{
	GetJacobianProduct();
	svdProduct_ = Eigen::JacobiSVD<MatrixX>(jacobianProduct_, Eigen::ComputeThinU | Eigen::ComputeThinV);
	u1 = svdProduct_.matrixU().block(0,0,3,1) / (svdProduct_.singularValues()(0) + 0.000000000000000000000000001);
	u2 = svdProduct_.matrixU().block(0,1,3,1) / (svdProduct_.singularValues()(1) + 0.000000000000000000000000001);
	u3 = svdProduct_.matrixU().block(0,2,3,1) / (svdProduct_.singularValues()(2) + 0.000000000000000000000000001);
	// TODO
}

void Jacobian::GetEllipsoidAxes(matrices::Vector3& u1, matrices::Vector3& u2, matrices::Vector3& u3, NUMBER& sig1, NUMBER& sig2, NUMBER& sig3)
{
	GetJacobianProduct();
	svdProduct_ = Eigen::JacobiSVD<MatrixX>(jacobianProduct_, Eigen::ComputeThinU | Eigen::ComputeThinV);
	u1 = svdProduct_.matrixU().block(0,0,3,1);
	u2 = svdProduct_.matrixU().block(0,1,3,1);
	u3 = svdProduct_.matrixU().block(0,2,3,1);
	sig1 = 1. / (svdProduct_.singularValues()(0) + 0.000000000000000000000000001);
	sig2 = 1. / (svdProduct_.singularValues()(1) + 0.000000000000000000000000001);
	sig3 = 1. / (svdProduct_.singularValues()(2) + 0.000000000000000000000000001);
}


void Jacobian::ComputeAll(const Tree& tree)
{
	ComputeJacobian(tree);
	ComputeAll();
}
void Jacobian::ComputeAll()
{
	ComputeSVD();
	GetJacobianInverse();
	GetJacobianProduct();
	GetJacobianProductInverse();
	GetNullspace();
}

const MatrixX& Jacobian::GetJacobian()
{
	return jacobian_;
}

MatrixX Jacobian::GetJacobianCopy()
{
	return jacobian_;
}
const MatrixX& Jacobian::GetJacobianInverse()
{
	if(computeInverse_)
	{
		computeInverse_ = false;
		jacobianInverse_ = jacobian_;
		PseudoInverseDLS(jacobianInverse_, 1.f); // tmp while figuring out how to chose lambda
	}
	return jacobianInverse_;
}

const MatrixX& Jacobian::GetNullspace()
{
	if(computeNullSpace_)
	{
		computeNullSpace_ = false;
		/*jacobianInverseNoDls_ = jacobian_;
		PseudoInverse(jacobianInverseNoDls_); // tmp while figuring out how to chose lambda*/
		//ComputeSVD();
		MatrixX id = MatrixX::Identity(jacobian_.cols(), jacobian_.cols());
		ComputeSVD();
		//Eigen::JacobiSVD<MatrixX> svd(jacobian_, Eigen::ComputeThinU | Eigen::ComputeThinV);
		MatrixX res = MatrixX::Zero(id.rows(), id.cols());
		for(int i =0; i < svd_.matrixV().cols(); ++ i)
		{
			VectorX v = svd_.matrixV().col(i);
			res += v * v.transpose();
		}
		Identitymin_ = id - res;
		//Identitymin_ = id - (jacobianInverseNoDls_* jacobian_);
	}
	return Identitymin_;
}

void Jacobian::GetNullspace(const MatrixX pseudoId, MatrixX& result)
{
		GetNullspace(); // computing inverse jacobian

		MatrixX id = MatrixX::Identity(Identitymin_.rows(), Identitymin_.cols());
		result = pseudoId - (id + Identitymin_);
		//ComputeSVD();
		//MatrixX id = MatrixX::Identity(svd_.matrixV().cols(), svd_.matrixV().rows());
		/*ComputeSVD();
		MatrixX id = MatrixX::Identity(svd_.matrixV().cols(), svd_.matrixV().rows());
		MatrixX res = MatrixX::Zero(id.rows(), id.cols());
		for(int i =0; i < id.cols(); ++ i)
		{
			VectorX v = svd_.matrixV().col(i);
			res += v * v.transpose();
		}
		//Identitymin_ = id - res;
		result = id  - (GetJacobianInverse()* GetJacobian());*/
}

const Matrix3& Jacobian::GetJacobianProduct()
{
	if(computeProduct_)
	{
		computeProduct_ = false;
		jacobianProduct_ = jacobian_ * jacobian_.transpose();
	}
	return jacobianProduct_;
}

const Matrix3& Jacobian::GetJacobianProductInverse()
{
	if(computeProductInverse_)
	{
		computeProductInverse_ = false;
		Eigen::JacobiSVD<Matrix3> svd = Eigen::JacobiSVD<Matrix3>(jacobianProduct_, Eigen::ComputeFullU | Eigen::ComputeFullV);
		PseudoInverseSVDDLS(jacobianProduct_, svd, jacobianProductInverse_);
	}
	return jacobianProductInverse_;
}

void Jacobian::ComputeSVD()
{
	if(computeJacSVD_)
	{
		computeJacSVD_ = false;
		svd_ = Eigen::JacobiSVD<MatrixX>(jacobian_, Eigen::ComputeThinU | Eigen::ComputeThinV);
	}
}

