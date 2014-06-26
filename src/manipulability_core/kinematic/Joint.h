
#ifndef _CLASS_JOINT
#define _CLASS_JOINT

#include "MatrixDefs.h"
#include "Com.h"
#include "Exports.h"

#include "API/JointI.h"

class ComVisitor_ABC;

enum Purpose {JOINT, EFFECTOR};

class Joint : public manip_core::JointI {

	friend class Tree;

public:
	Joint(const matrices::Vector3& /*attach*/, const matrices::Vector3& /*iRotAxis*/, Purpose /*purpose*/, Com /*com*/, NUMBER restAngle=0., manip_core::enums::rotation::eRotation rot = manip_core::enums::rotation::Z);
	Joint(const matrices::Vector3& /*attach*/, const matrices::Vector3& /*iRotAxis*/, Purpose /*purpose*/, Com /*com*/, NUMBER minTheta, NUMBER maxTheta, NUMBER restAngle=0., manip_core::enums::rotation::eRotation rot = manip_core::enums::rotation::Z);

	~Joint();

private:
    Joint& operator =(const Joint&);
	Joint(const Joint&);

// inherited
public:
	virtual void Release();
	virtual double GetAngle() const;
	virtual void Offset(double * /*offset*/) const;
	virtual const manip_core::enums::rotation::eRotation GetRotation() const;
	virtual bool IsEffector() const;
	virtual bool IsJoint() const;
	virtual const JointI* GetSon() const;
	virtual const JointI* GetParent() const;
	virtual const bool IsLocked() const ;

public:
	void InitJoint();

	NUMBER GetTheta() const { return theta_; }
	NUMBER AddToTheta(const NUMBER delta);

	const Com& GetCom() const {return com_;};


	const matrices::Vector3& GetS() const { return s_; }
	const matrices::Vector3& GetR() const { return r_; }
	const matrices::Vector3& GetW() const { return w_; }
	const matrices::Vector3& GetRotationAxis() const { return v_; }

	NUMBER GetMinTheta() const { return minTheta_; }
	NUMBER GetMaxTheta() const { return maxTheta_; } 
	NUMBER GetRestAngle() const { return restAngle_; } ;

	void SetTheta(NUMBER newTheta) { theta_ = newTheta; }
	const matrices::Vector3& ComputeS(void);
	void ComputeW(void);

	void AcceptComVisitor(ComVisitor_ABC* /*visitor*/) const;

	int GetEffectorNum() const { return seqNumEffector_; }
	int GetJointNum() const { return seqNumJoint_; }
	void ToRest();

	Joint* Clone() const;

public:
	Joint* pRealparent_;		// pointer to real parent
	Joint* pChild_;				// pointer to child
	
private:
	Purpose purpose_;			// joint / effector / both
	manip_core::enums::rotation::eRotation rot_;				// joint / effector / both
	int seqNumJoint_;			// sequence number if this node is a joint
	int seqNumEffector_;		// sequence number if this node is an effector
	matrices::Vector3 attach_;	// attachment point
	matrices::Vector3 r_;		// relative position vector
	const matrices::Vector3 v_;	// rotation axis
	NUMBER theta_;				// joint angle (radian)
	NUMBER minTheta_;			// lower limit of joint angle
	NUMBER maxTheta_;			// upper limit of joint angle
	NUMBER restAngle_;			// rest position angle
	matrices::Vector3 s_;		// GLobal Position
	matrices::Vector3 w_;		// Global rotation axis
	const Com com_;
};

#endif // _CLASS_JOINT
