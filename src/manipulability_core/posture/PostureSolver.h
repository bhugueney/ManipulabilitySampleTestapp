
#ifndef _CLASS_POSTURESOLVER
#define _CLASS_POSTURESOLVER

#include "API/PostureManagerI.h"

#include "sampling/SampleGeneratorVisitor_ABC.h"
#include "kinematic/Robot.h"
#include "MatrixDefs.h"
#include "Trajectory.h"
#include "sampling/Sample.h"

#include <memory>
#include <vector>

class Sample;
class SampleGenerator;
class World;
class Tree;
class PostureCriteria_ABC;
class SupportPolygon;

struct PosturePImpl;

/* parses sampled configurations in order to find an appropriate posture given a previous posture, 
the current trajectory and the world*/

class PostureSolver
{
public:
    typedef std::vector<std::pair<NUMBER, Robot*> >	T_Robots;
	typedef T_Robots::iterator						T_RobotsIT;
	typedef T_Robots::const_iterator				T_RobotsCIT;

public:
	 PostureSolver(const World& /*world*/); // todo direciton / trajectory
	~PostureSolver();

public:
	void   RegisterPostureListener(		 manip_core::PostureCreatedListenerI& /*listener*/);
	bool UnregisterPostureListener(const manip_core::PostureCreatedListenerI& /*listener*/); // TODO

	void AddToeOffCriteria(PostureCriteria_ABC* /*criteria*/);
	void AddToeOnCriteria (PostureCriteria_ABC* /*criteria*/);
	void SetJumpToTarget(const bool /*jump*/);

public:
	manip_core::T_CubicTrajectory	NextTrajectory(Robot& /*robot*/, const matrices::Vector3& /*direction*/,  bool handleLock = false, bool closestDistance = false);
	int				NextPosture(Robot& /*robot*/, const matrices::Vector3& /*direction*/,  bool handleLock = false);
	const T_Robots& CreatePostures(const Robot& /*previousTransform*/, Trajectory& /*trajectory*/, bool stopAtFirst = false);
	#ifdef PROFILE
	void Log() const;
	#endif

private:
	bool MustLift    (const Robot& /*robot*/, const Tree& /*tree*/) const;
	bool MustLock    (const Robot& /*robot*/, const Tree& /*tree*/) const;
	
	bool LockTree		 (Robot& /*robot*/, Tree& /*tree*/, bool closestDistance = false) const;  // Gets sampled tree configuration that suits the best to constraints
	bool LockTree		 (Robot& /*robot*/, Tree& /*tree*/, Sample& sample, bool closestDistance = false) const;  // Gets sampled tree configuration that suits the best to constraints
	bool HandleLockedTree(Robot& /*robot*/, Tree& /*tree*/);  // Gets sampled tree configuration that suits the best to constraints

private:
	std::auto_ptr<PosturePImpl> pImpl_;
};

#endif //_CLASS_POSTURESOLVER
