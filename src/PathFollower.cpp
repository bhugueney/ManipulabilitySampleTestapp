#include "PathFollower.h"

#include "kinematic/Robot.h"
#include "Trajectory.h"
#include "PostureSolver.h"
#include "Timer.h"


using namespace matrices; 

struct PathPImpl : public PostureCreatedListener_ABC
{
	PathPImpl(Robot& robot, const Trajectory& trajectory, PostureSolver& postureSolver)
		: timePositions_(trajectory.GetTimePositions())
		, robot_(robot)
		, previousTime_(0)
	{
		postureSolver.RegisterPostureListener(*this);
		current_ = robots_.end();
	}

	~PathPImpl()
	{
		// NOTHING
	}

	virtual void OnPostureCreated(NUMBER time, const Robot* pRobot)
	{
		robots_.push_back(std::make_pair(time, pRobot));
		current_ = robots_.begin();
	}

	bool Next()
	{
		if (current_ != robots_.end())
		{
			++current_;
		}
		return current_ != robots_.end();
	}

	void Update()
	{
		// TODO
		//is current posture past
		unsigned long time = timer_.GetTime();
		while((current_ != robots_.end()) && (((*current_).first)*1000 <= time))
		{
			if (Next())
			{
				//assign targets
				const Robot::T_Tree& trees = (*current_).second->GetTrees();
				for(Robot::T_TreeCIT it = trees.begin(); it!= trees.end(); ++it)
				{
					if((*it)->IsLocked())
					{
						robot_.GetTree((*it)->GetId())->LockTarget((*it)->GetTarget());
					}
					else
					{
						robot_.GetTree((*it)->GetId())->UnLockTarget();
					}
				}
			}

		}
		if(current_ != robots_.end())
		{
			const NUMBER reachingTime = (*current_).first*1000;
			const Matrix4& target((*current_).second->ToWorldCoordinates());
			// translation
			Vector3 from, to;
			from = robot_.ToWorldCoordinates().block(0,3,3,1);
			to = target.block(0,3,3,1);
			Vector3 direction = to - from;
			robot_.Translate(direction*((time - previousTime_)/(reachingTime - time)));
			previousTime_ = time;
			//IK
			direction.normalize();
			//robot_.Move(direction);
		}
	}

	Timer timer_;
	unsigned long previousTime_;
	const Trajectory::T_TimePositions timePositions_;
	//const PostureSolver postureSolver_;
	std::vector<std::pair<NUMBER,const Robot*>> robots_;
	std::vector<std::pair<NUMBER,const Robot*>>::const_iterator current_;
	Robot& robot_;

};

PathFollower::PathFollower(Robot& robot, const Trajectory& trajectory, PostureSolver& postureSolver)
	: pImpl_(new PathPImpl(robot, trajectory, postureSolver))
{
	// NOTHING
}

PathFollower::~PathFollower()
{
	// NOTHING
}

void PathFollower::Start()
{
	pImpl_->timer_.Start();
	pImpl_->current_ = pImpl_->robots_.begin();
	pImpl_->previousTime_ = 0;
	Update();
}

void PathFollower::Update()
{
	pImpl_->Update();
}