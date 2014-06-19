
#include "kinematic/Tree.h"
#include "kinematic/Joint.h"
#include "MatrixDefs.h"
#include "math.h"
#include "sampling/Sample.h"
#include "Jacobian.h"
#include "Pi.h"


using namespace matrices;
using namespace Eigen;
using namespace manip_core::enums;

Tree::Tree(TREE_ID id, eMembers treeType)
: sphereRadius_(0)
, lock_(false)
, jacobian_(0)
, id_(id)
, templateId_(id)
, treeType_(treeType)
, targetReached_(true)
, direction_(1,0,0)
, obsTarget_(0)
, onObstacle_(false)
, targetSample_(0)
{
	directionForce_  = Vector3(0, 1, 0);
	directionVel_ = Vector3(1, 0, 0);
	target_ = Vector3(1, 0, 0);
	root = 0;
	nJoint = nEffector = nJoint = 0;
}

Tree::Tree(TREE_ID id, TREE_ID templateId, eMembers treeType)
: sphereRadius_(0)
, lock_(false)
, jacobian_(0)
, id_(id)
, templateId_(templateId)
, treeType_(treeType)
, obsTarget_(0)
, onObstacle_(false)
, targetSample_(0)
{
	directionForce_  = Vector3(0, 1, 0);
	directionVel_ = Vector3(1, 0, 0);
	target_ = Vector3(1, 0, 0);
	root = 0;
	nJoint = nEffector = nJoint = 0;
}


Tree::~Tree()
{
	Joint* n = this->GetRoot();
	Joint* n1 = n;
	while (n)
	{
		n = n->pChild_;
		delete n1;
		n1 = n;
	}
}

void Tree::LockTarget(const matrices::Vector3& target, const Obstacle* obsTarget)
{ 
	target_ = target; lock_ = true; obsTarget_ = obsTarget; onObstacle_ = true; 
}


const eMembers& Tree::GetTreeType() const
{
	return treeType_;
}


void Tree::ToRest()
{
	Joint* j = GetRoot();
	while(j)
	{
		j->ToRest();
		j = j->pChild_;
	}
	Compute();
}

bool Tree::GetObstacleNormal(double* target) const
{
	if(onObstacle_)
	{
		matrices::vect3ToArray(target, obsTarget_->n_);
		return true;
	}
	else
	{
		return false;
	}
}

double Tree::GetManipulability(const double& x, const double& y, const double& z) const // this is expensive
{
	Jacobian jac(*this);
	Vector3 direction(x, y, z);
	NUMBER r = (direction.transpose()*jac.GetJacobianProduct()*direction);
	return 1/sqrt(r);
}

void Tree::GetEllipsoidAxes(double* u1, double* u2, double* u3) const
{
	Vector3 v1, v2, v3;
	Jacobian jac(*this);
	jac.GetEllipsoidAxes(v1, v2, v3);
	matrices::vect3ToArray(u1,v1);
	matrices::vect3ToArray(u2,v2);
	matrices::vect3ToArray(u3,v3);
}

void Tree::GetEllipsoidAxes(double* u1, double* u2, double* u3, double& sig1, double& sig2, double& sig3) const
{
	Vector3 v1, v2, v3;
	Jacobian jac(*this);
	jac.GetEllipsoidAxes(v1, v2, v3, sig1, sig2, sig3);
	matrices::vect3ToArray(u1,v1);
	matrices::vect3ToArray(u2,v2);
	matrices::vect3ToArray(u3,v3);
}


void Tree::SetSeqNum(Joint* joint)
{
	switch (joint->purpose_) {
	case JOINT:
		joint->seqNumJoint_ = nJoint++;
		joint->seqNumEffector_ = -1;
		break;
	case EFFECTOR:
		joint->seqNumJoint_ = -1;
		joint->seqNumEffector_ = nEffector++;
		break;
	}
}

void Tree::InsertRoot(Joint* root)
{
		assert(nJoint == 0);
	nJoint++;
	Tree::root = root;
	root->r_ = root->attach_;
	assert(!(root->pChild_));
	SetSeqNum(root);
	//sphereRadius_ += root->r_.norm();
}

void Tree::InsertChild(Joint* parent, Joint* child)
{
	assert(parent);
	parent->pChild_ = child;
	child->pRealparent_ = parent;
	child->r_ = child->attach_ - child->pRealparent_->attach_;
	sphereRadius_ += child->r_.norm();
	assert(!(child->pChild_));
	SetSeqNum(child);
}

// Search recursively below "Joint" for the Joint with index value.
Joint* Tree::SearchJoint(Joint* joint, int index) const
{
	Joint* ret(0);
	if (joint != 0)
	{
		if (joint->seqNumJoint_ == index)
		{
			ret = joint;
		}
		else
		{
			ret = SearchJoint(joint->pChild_, index);
		}
	} 
	return ret;
}

const bool Tree::JointLimitBroken() const
{
	Joint* j = GetRoot();
	int i = 0;
	while(j)
	{
		if(i > 1)return false;
		++i;
		if(j->GetRotation() == manip_core::enums::rotation::X &&  (j->GetAngle() > j->GetMaxTheta() || j->GetAngle() < j->GetMinTheta()))
		{
			return true;
		}
		j = j->pChild_;
	}
	return false;
}

// Search recursively below Joint for the end effector with the index value
Joint* Tree::SearchEffector(Joint* joint, int index) const
{
	Joint* ret(0);
	if (joint != 0)
	{
		if (joint->seqNumEffector_ == index)
		{
			ret = joint;
		} 
		else
		{
			ret = SearchEffector(joint->pChild_, index);
		}
	}
	return ret;
}

// Get the joint with the index value
Joint* Tree::GetJoint(int index) const
{
	return SearchJoint(root, index);
}

// Get the end effector for the index value
Joint* Tree::GetEffector(int index) const
{
	return SearchEffector(root, index);
}

// Returns the global position of the effector.
const Vector3& Tree::GetEffectorPosition(int index) const
{
	Joint* effector = GetEffector(index);
	assert(effector);
	return (effector->s_);  
}

void Tree::ComputeTree(Joint* joint)
{
	if (joint != 0) {
		joint->ComputeS();
		joint->ComputeW();
		ComputeTree(joint->pChild_);
	}
}

void Tree::Compute(void)
{ 
	ComputeTree(root); 
}

// Recursively initialize this below the Joint
void Tree::InitTree(Joint* joint)
{
	if (joint != 0)
	{
		joint->InitJoint();
		InitTree(joint->pChild_);
	}
}

// Initialize all Joints in the this
void Tree::Init(void)
{
	InitTree(root);
	ToRest();
	Compute();
	referenceTarget_ = GetEffectorPosition(GetNumEffector()-1);
	jacobian_ = new Jacobian(*this);
}

void Tree::ComputeJacobian() 
{
	jacobian_->ComputeJacobian(*this);
}

const Vector3& Tree::GetPosition() const
{
	Joint* n = this->GetRoot();
	n->ComputeS();
	return n->GetS();
}

Tree* Tree::Clone() const
{
	Tree* res = new Tree(id_, templateId_, treeType_);
	Joint* n1 = this->GetRoot();
	if(n1)
	{
		Joint* n2(n1->pChild_);
		n1 = n1->Clone(); // clones don't have children !
		res->InsertRoot(n1);
		Joint* tmp(0);
		while(n2)
		{
			tmp = n2;
			n2 = n2->Clone();
			res->InsertChild(n1, n2);
			n1 = n2;
			n2 = tmp->pChild_;
		}
	}
	if(IsLocked())
	{
		res->LockTarget(target_);
	}
	res->obsTarget_ = obsTarget_;
	res->onObstacle_ = onObstacle_;
	res->direction_ = direction_;
	res->Compute();
	return res;
}

void Tree::AcceptComVisitor(ComVisitor_ABC* visitor) const
{
	Joint* n = this->GetRoot();
	while(n)
	{
		n->AcceptComVisitor(visitor);
		n = n->pChild_;
	}
}

void Tree::Release()
{
	delete this;
}

void Tree::EndEffectorPosition(double* position) const
{
	matrices::vect3ToArray(position, GetEffectorPosition(GetNumEffector()-1));
}

void Tree::Position(double* position) const
{
	matrices::vect3ToArray(position, GetPosition());
}

const manip_core::JointI* Tree::GetRootJointI() const
{
	return this->GetRoot();
}

bool Tree::IsAnchored() const
{
	return IsLocked();
}

void Tree::GetTarget(double* target) const
{
	matrices::vect3ToArray(target, target_);
}


void Tree::SetTarget(double* target)
{
	matrices::arrayToVect3(target, target_);
}

void Tree::GetReferenceTarget(double* target) const
{
	matrices::vect3ToArray(target, referenceTarget_);
}


