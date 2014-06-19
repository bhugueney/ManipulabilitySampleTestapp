
#ifndef _CLASS_POSTUREMANAGER
#define _CLASS_POSTUREMANAGER

#include "API/SampleVisitorI.h"
#include "API/PostureManagerI.h"

#include "spline/exact_cubic.h"

#include "MatrixDefs.h"
#include <vector>

namespace manip_core
{
struct RobotI;
struct TreeI;

class PostureManager
{
public:
	 explicit PostureManager(PostureManagerI* /*pPostureManager*/);
	~PostureManager();

public:
	void ResetTrajectory();

	void SetJumpToTarget(const bool /*jump*/);

	void Update(const unsigned long /*time*/);

	void AddCheckPoint(const float /*time*/, const matrices::Vector3& /*transform*/);
	/**
	Add a criteria either for raising or lowering foot.
	*/
	void AddPostureCriteria(const enums::postureCriteria::ePostureCriteria /*criteria*/) ;
	/**
	Register a PostureCreatedListenerI that will be called whenever a new posture is created
	*/
	void RegisterPostureCreatedListenerI(PostureCreatedListenerI* /*listener*/) ;	
	/**
	Unregister a previously created lister
	*/
	void UnRegisterPostureCreatedListenerI(PostureCreatedListenerI* /*listener*/) ;
	/**
	Compute solution posture for given trajectory and constraints
	*/
	/**
	Compute solution posture for given trajectory and constraints ( climbing)
	*/
	void ComputeOnline(const RobotI* /*robot*/, int /*nbSamples*/) ;

	void InitSamples(const RobotI* /*robot*/, int /*nbSamples*/);

	void VisitSamples(const RobotI* /*robot*/, const TreeI* /*tree*/, SampleVisitorI* /*visitor*/, bool /*collide*/);

	T_CubicTrajectory NextPosture(RobotI* /*robot*/, const matrices::Vector3& /*direction*/);

	T_CubicTrajectory ComputeTrajectoryPostures(RobotI* /*robot*/);

	#ifdef PROFILE
	void Log() const;
	#endif
private:
	PostureManagerI* pPostureManager_;
};

} // namespace manip_core
#endif //_CLASS_POSTUREMANAGER