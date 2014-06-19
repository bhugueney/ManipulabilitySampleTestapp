#include "PostureManager.h"

#include "Simulation.h"
#include "MatrixDefs.h"

#include "API/PostureManagerI.h"

using namespace manip_core;
using namespace matrices;

PostureManager::PostureManager(PostureManagerI* pPostureManager)
	: pPostureManager_(pPostureManager)
{
	// NOTHING
}

PostureManager::~PostureManager()
{
	pPostureManager_->Release();
}

void PostureManager::SetJumpToTarget(const bool jump)
{
	pPostureManager_->SetJumpToTarget(jump);
}


void PostureManager::AddCheckPoint(const float time, const matrices::Vector3& transform)
{
	double transf[3];
	matrices::vect3ToArray(transf, transform);
	pPostureManager_->AddTrajectoryPoint(time, transf);
}

void PostureManager::ResetTrajectory()
{
	pPostureManager_->ResetTrajectory();
}

void PostureManager::AddPostureCriteria(const enums::postureCriteria::ePostureCriteria criteria) 
{
	pPostureManager_->AddPostureCriteria(criteria);
}

void PostureManager::RegisterPostureCreatedListenerI(PostureCreatedListenerI* listener) 	
{
	pPostureManager_->RegisterPostureCreatedListenerI(listener);
}

void PostureManager::UnRegisterPostureCreatedListenerI(PostureCreatedListenerI* listener) 
{
	pPostureManager_->UnRegisterPostureCreatedListenerI(listener);
}

void PostureManager::ComputeOnline(const RobotI* robot, int nbSamples) 
{
	pPostureManager_->ComputeOnline(robot, nbSamples);
}

void PostureManager::InitSamples(const RobotI* robot, int nbSamples)
{
	pPostureManager_->InitSamples(robot, nbSamples);
}

void PostureManager::VisitSamples(const RobotI* robo, const TreeI* tree, SampleVisitorI* visitor, bool collide)
{
	pPostureManager_->AcceptSampleVisitor(robo, tree, visitor, collide);
}


T_CubicTrajectory PostureManager::NextPosture(RobotI* robot, const matrices::Vector3& direction)
{
	double dir [3];
	matrices::vect3ToArray(dir, direction);
	return pPostureManager_->NextPosture(robot, dir, Simulation::GetInstance()->simpParams_.closestDistance_);
}

void PostureManager::Update(const unsigned long time)
{
	pPostureManager_->Update(time);
}

#ifdef PROFILE
void PostureManager::Log() const
{
	pPostureManager_->Log();
}
#endif



