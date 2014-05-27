#include "sampling\SampleGeneratorVisitor_ABC.h"
#include "tools/MatrixDefs.h"
#include <vector>
#include <xmemory>
#include <list>
#include "world\World.h"

struct RobotI;
struct TreeI;
struct PosturePImpl;

using namespace manip_core;

namespace enums
{
	namespace postureCriteria
	{
		enum ePostureCriteria
		{
			toeOffBoundary = 0, // raise foot when off boundary
			toeOffJointLimit, // raise foot when joint limit reached
			toeOnCOM,           // anchor foot when stability is lost
			toeOffSpiderGait,
			toeOnSpiderGait,
			unknown
		};
	}
}
class PostureManager
{
public:
	typedef std::vector<std::pair<double, Robot*>>	T_Robots;
	typedef T_Robots::iterator						T_RobotsIT;
	typedef T_Robots::const_iterator				T_RobotsCIT;
	typedef std::pair  <float, matrices::Vector3>	P_TimePosition;
	typedef std::list<P_TimePosition, Eigen::aligned_allocator<std::pair<const int, matrices::Matrix4>>> T_TimePositions;
	typedef T_TimePositions::iterator				T_TimePositionsIT;
	typedef T_TimePositions::const_iterator			T_TimePositionsCIT;

public:
	explicit PostureManager(World* /*world*/);
	~PostureManager();

public:
	void ResetTrajectory();

	void SetJumpToTarget(const bool /*jump*/);

	void Update(const unsigned long /*time*/);

	void AddCheckPoint(const float /*time*/, const matrices::Vector3& /*transform*/);
	/**
	Add a criteria either for raising or lowering foot.
	*/
	void AddPostureCriteria(const enums::postureCriteria::ePostureCriteria /*criteria*/);

//	int NextPosture(Robot& robot, const Vector3& direction, bool handleLock);

	PostureManager::T_TimePositions& GetEditableTimePositions();
	/**
	Register a PostureCreatedListenerI that will be called whenever a new posture is created
	*/
	//void RegisterPostureCreatedListenerI(PostureCreatedListenerI* /*listener*/);
	/**
	Unregister a previously created lister
	*/
	//void UnRegisterPostureCreatedListenerI(PostureCreatedListenerI* /*listener*/);
	/**
	Compute solution posture for given trajectory and constraints
	*/
	/**
	Compute solution posture for given trajectory and constraints ( climbing)
	*/
	void ComputeOnline(const Robot& /*robot*/, int /*nbSamples*/);

	void InitSamples(const Robot& /*robot*/, int /*nbSamples*/);

	//void VisitSamples(const RobotI* /*robot*/, const TreeI* /*tree*/, SampleGeneratorVisitor_ABC* /*visitor*/, bool /*collide*/);

	//T_CubicTrajectory NextPosture(RobotI* /*robot*/, const matrices::Vector3& /*direction*/);

	//T_CubicTrajectory ComputeTrajectoryPostures(RobotI* /*robot*/);
private:
	//PostureManagerI* world_;
	World* world_;
	std::auto_ptr<PosturePImpl> pImpl_;
private:
	T_TimePositions timePositions_;
#ifdef PROFILE
		void Log() const;
#endif
	
	};

