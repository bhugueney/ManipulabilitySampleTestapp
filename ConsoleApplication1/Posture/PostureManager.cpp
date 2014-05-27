#include "tools/MatrixDefs.h"
#include "world\World.h"

#include "PostureManager.h"
#include "PostureCriteria_ABC.h"
#include "PostureCriteriaToeOnCOM.h"
#include <list>

using namespace manip_core;
using namespace matrices;

struct PosturePImpl
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	PosturePImpl(const World& world)
	: world_(world)
	, currentDir_(1, 0, 0)
	, oldDir_(1, 0, 0)
	//, ikSolver_()
	//, currentDirWeight_(0)
	//, lastLifted_(-1)
	, jumpToTarget_(false)
	//, trajectoryHandler_(world)
	{
		//NOTHING
#ifdef PROFILE
		totaltime_ = 0;
		timerperf_.restart();
#endif
	}
	~PosturePImpl()
	{
		/*
		Clear();
		for (T_CriteriaIT it = toeOffs_.begin(); it != toeOffs_.end(); ++it)
		{
			delete(*it);
		}
		for (T_CriteriaIT it = toeOns_.begin(); it != toeOns_.end(); ++it)
		{
			delete(*it);
		}*/
	}
	
	/*
	
	typedef std::list<manip_core::PostureCreatedListenerI*>	T_Listener;
	typedef T_Listener::iterator					T_ListenerIT;
	typedef T_Listener::const_iterator				T_ListenerCIT;*/
	typedef std::list<PostureCriteria_ABC*> T_Criteria;
	typedef T_Criteria::iterator			T_CriteriaIT;
	typedef T_Criteria::const_iterator		T_CriteriaCIT;
	/*
	void WarnListeners(NUMBER time, Robot* robot)
	{
		for (T_ListenerIT it = listeners_.begin(); it != listeners_.end(); ++it)
		{
			(*it)->OnPostureCreated(time, robot);
		}
	}
	
	void Clear()
	{
		for (PostureSolver::T_RobotsIT it = postures_.begin(); it != postures_.end(); ++it)
		{
			delete((*it).second);
		}
		postures_.clear();
		currentDir_ = Vector3(1, 0, 0);
		oldDir_ = Vector3(1, 0, 0);
		lastLifted_ = -1;
	}*/

	//IKSolver ikSolver_;
	const World& world_;
	Vector3 currentDir_;
	Vector3 oldDir_;

	bool jumpToTarget_;
	T_Criteria toeOns_;
	PostureManager::T_Robots postures_;
	/*
	NUMBER currentDirWeight_;
	PostureSolver::T_Robots postures_;
	T_Listener listeners_;
	T_Criteria toeOffs_;
	T_Criteria toeOns_;
	Tree::TREE_ID lastLifted_;
	bool jumpToTarget_;
	TrajectoryHandler trajectoryHandler_;*/
#ifdef PROFILE
	TimerPerf timerperf_;
	std::vector<float> times_;
	float totaltime_;
	std::vector<int> hits_;
#endif
};


PostureManager::PostureManager(World* world)
: world_(world)
{
	// NOTHING
}

PostureManager::~PostureManager()
{
	//pPostureManager_->Release();
}

void PostureManager::SetJumpToTarget(const bool jump)
{
//	pImpl_->jumpToTarget_ = jump;
}

PostureManager::T_TimePositions& PostureManager::GetEditableTimePositions()
{
	return timePositions_;
}

void PostureManager::AddCheckPoint(const float time, const matrices::Vector3& transform)
{
	bool inserted = false;
	for (T_TimePositionsIT it = timePositions_.begin(); it != timePositions_.end(); ++it)
	{
		float cTime = (*it).first;
		if (cTime == time)
		{
			//return false;
			return;
		}
		else if (cTime > time)
		{
			timePositions_.insert(it, P_TimePosition(time, transform));
			inserted = true;
			return;
			//return true;
		}
	}
	if (!inserted)
	{
		if (!timePositions_.empty())
		{
			float lastime = timePositions_.back().first;
			// interpolate trajectory as straight line
			// TODO Meilleure échelle
			// compute speed
			Vector3 oldPosition = timePositions_.back().second;
			Vector3 speed = (transform - oldPosition) / (time - lastime);
			/*for (float i = lastime + 0.1f; i < time; i = i + 0.1f)
			{
			Vector3 newVal = oldPosition + speed * ( i);
			timePositions_.push_back(P_TimePosition(i, oldPosition + speed * ( i - lastime) ));
			}*/
		}
		timePositions_.push_back(P_TimePosition(time, transform));
		return;
		//return true;
	}
	return;
	//return false;
}

void PostureManager::ResetTrajectory()
{
	//pPostureManager_->ResetTrajectory();
}

void PostureManager::AddPostureCriteria(const enums::postureCriteria::ePostureCriteria criteria)
{
	switch (criteria){
	case enums::postureCriteria::ePostureCriteria::toeOnCOM:
	{
		PostureCriteriaToeOnCOM * p = new PostureCriteriaToeOnCOM();
//		pImpl_->toeOns_.push_back(p);
	}
	}
}
/*
void PostureManager::RegisterPostureCreatedListenerI(PostureCreatedListenerI* listener)
{
	//pPostureManager_->RegisterPostureCreatedListenerI(listener);
}


void PostureManager::UnRegisterPostureCreatedListenerI(PostureCreatedListenerI* listener)
{
	//pPostureManager_->UnRegisterPostureCreatedListenerI(listener);
}
*/
/*
void PostureManager::ComputeOnline(const Robot& previousTransform, int nbSamples)
{
	// Create Postures
	//pImpl_->Clear(); not working?
	Robot* previousPosture(0);
	T_TimePositions& timepos = GetEditableTimePositions();
	Matrix3 rotation;
	Matrix4 oldTransformation;
	Matrix4 tranformation = previousTransform.ToWorldCoordinates();
	Vector3 oldPosition = tranformation.block(0, 3, 3, 1);
	float currentTimeStep = 0.f;
	previousPosture = previousTransform.Clone();

	//current pos as first index
	T_TimePositionsIT it = timepos.begin();
	++it;
	for (; it != timepos.end() && pImpl_->postures_.size()<80; ++it)
	{
		pImpl_->currentDir_ = ((*it).second - oldPosition);
		if (pImpl_->currentDir_.norm() != 0.)
		{
			// only for vlimbing)
			if (pImpl_->currentDir_(0) <= 0)
			{
				pImpl_->currentDir_(0) = 0.2;
			}
			pImpl_->currentDir_.normalize();
		}
		else
		{
			pImpl_->currentDir_ = pImpl_->oldDir_;
			++it;
			if (it == timepos.end())
				break;
		}
		oldTransformation = tranformation;

		//compute transformation
		matrices::GetRotationMatrix(pImpl_->oldDir_, pImpl_->currentDir_, rotation);
		tranformation.block(0, 0, 3, 3) = tranformation.block(0, 0, 3, 3) * rotation;
		tranformation.block(0, 3, 3, 1) = (*it).second;
		//End compute transformation

	
		Robot* newPosture = previousPosture->Clone();
		newPosture->SetPosOri(tranformation);
		int changes = NextPosture(*newPosture, pImpl_->currentDir_, true);

		if (changes > 1 && trajectory.AddWayPoint(--it)) // it placed on the new waypoint
		{
			it--; // because it ll be increased in loop
			tranformation = oldTransformation;
			pImpl_->currentDir_ = pImpl_->oldDir_;
		}
		else
		{
			if ((changes == 0) &!(pImpl_->postures_.empty()))
			{
				pImpl_->postures_.pop_back();
			}
			/*else if(changes > 1)
			{
			delete newPosture;
			newPosture = CreateStillPosture2(*previousPosture);
			tranformation = oldTransformation;
			pImpl_->currentDir_ = pImpl_->oldDir_;
			}
			currentTimeStep = (*it).first;
			pImpl_->postures_.push_back(std::make_pair(currentTimeStep, newPosture));
			previousPosture = newPosture;
			// robot posture can be changed by optimization
			tranformation = newPosture->ToWorldCoordinates();
			oldPosition = tranformation.block(0, 3, 3, 1);
			pImpl_->oldDir_ = pImpl_->currentDir_;
			if (stopAtFirst)
			{
				return pImpl_->postures_;
			}
		}
	}
	for (PostureManager::T_RobotsIT it = pImpl_->postures_.begin(); it != pImpl_->postures_.end(); ++it)
	{
		pImpl_->WarnListeners((*it).first, (*it).second);
	}

	return pImpl_->postures_;
	
	//pPostureManager_->ComputeOnline(robot, nbSamples);
}

int PostureManager::NextPosture(Robot& robot, const Vector3& direction, bool handleLock)
{
	int changes = 0;
	pImpl_->currentDir_ = direction;
	Robot::T_Tree& trees = robot.GetTrees();
	for (Robot::T_TreeIT it = trees.begin(); it != trees.end(); ++it)
	{
		/*
		Tree* tree = (*it);
		if (tree->IsLocked())
		{
			if (MustLift(robot, *tree))
			{
				tree->UnLockTarget();
				++changes;
			}
			else if (handleLock)
			{
				if (!HandleLockedTree(robot, *tree))
				{
					tree->UnLockTarget();
					++changes;
				}
			}
		}
		else if (!tree->IsLocked())
		{
			if (MustLock(robot, *tree))
			{
				LockTree(robot, *tree);
				changes++;
			}
		}
	}
	return changes;
}
*/
/*
void PostureManager::InitSamples(const RobotI* robot, int nbSamples)
{
	//pPostureManager_->InitSamples(robot, nbSamples);
}

void PostureManager::VisitSamples(const RobotI* robo, const TreeI* tree, SampleGeneratorVisitor_ABC* visitor, bool collide)
{
	//pPostureManager_->AcceptSampleVisitor(robo, tree, visitor, collide);
}

T_CubicTrajectory PostureManager::NextPosture(RobotI* robot, const matrices::Vector3& direction)
{
	double dir[3];
	matrices::vect3ToArray(dir, direction);
	return pPostureManager_->NextPosture(robot, dir, Simulation::GetInstance()->simpParams_.closestDistance_);
}

void PostureManager::Update(const unsigned long time)
{
	pPostureManager_->Update(time);
}*/

#ifdef PROFILE
void PostureManager::Log() const
{
	pPostureManager_->Log();
}
#endif



