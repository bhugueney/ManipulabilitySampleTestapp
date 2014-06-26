#include "Simulation.h"
#include "AutoRotate.h"
#include "RootTrajectory.h"
//#include "XboxMotion.h"
#include "WorldParser/WorldParser.h"

#include <time.h>

using namespace manip_core;

Simulation* Simulation::instance = 0;

Simulation::Simulation()
	: manager_()
	, simpParams_()
	, pRobot(0)
	, timerHandler_()
	, postureManager_(manager_.GetPostureManager())
	, motionHandler_(manager_)
	, drawManager_(manager_)
{
	srand((unsigned int)(time(0))); //Init Random generation
	// TODO
}

Simulation::~Simulation()
{
	// TODO
	for(std::vector<TimerHandled_ABC*>::const_iterator it =managedTimeListeners_.begin(); it != managedTimeListeners_.end(); ++ it)
	{
		delete *it;
	}
	delete postureManager_;
}

namespace simspace
{
	void start()
	{
		Simulation* sim = Simulation::GetInstance();
		sim->simpParams_.GetCamera()->Reset();
		sim->simpParams_.GetCamera()->Update();
		std::cout << sim->simpParams_.background_ ;
		dsSetBackGround(sim->simpParams_.background_(0), sim->simpParams_.background_(1), sim->simpParams_.background_(2));
		dsSetDrawSky(sim->simpParams_.drawSky_);
		dsSetDrawGround(sim->simpParams_.drawGround_);
		dsSetLight(sim->simpParams_.lightX, sim->simpParams_.lightY);
		dsSetDrawShadow(sim->simpParams_.drawShadows_);
		
		if(sim->simpParams_.planif_)
		{	
			sim->manager_.GetPostureManager()->ComputeOnline(sim->pRobot, sim->simpParams_.nbSamples_);
		}
		sim->simpParams_.pause_ = ! sim->simpParams_.pause_;
		sim->Update();
		sim->simpParams_.pause_ = ! sim->simpParams_.pause_;
		if(sim->simpParams_.pause_)
		{
			sim->pRobot->Rest();
		}
	}

	void simLoop(int pause)
	{
		Simulation* sim = Simulation::GetInstance();
		sim->Update();
		sim->Draw();
	}
}

#include<fstream>
#include <Windows.h>
using namespace std;
void Simulation::Start(int argc, char *argv[])
{
	srand((unsigned int)(time(0))); //Init Random generation
	//init robot
	
	WorldParser parser;
	std::string filename;
	
	if(argc > 1)
	{
		filename = std::string(argv[1]);
		parser.CreateWorld(filename);
	}
	else
	{
		std::string startupFile = std::string("../world/startup.xml");
		// Open startup xml
		if (!std::ifstream("../world/startup.xml")){
			MessageBox(NULL, "Unable to open startup file\n", "Error", MB_OK);
			exit(0);
		} 

		parser.InitWorld(startupFile);
	}
	
	

	//simpParams_.buildWorld();

	if(simpParams_.angleValues_.size() > 0)
	{
        /*if(simpParams_.root)
		{
			pRobot = manager_.CreateRobot(simpParams_.root, simpParams_.robotBasis_, simpParams_.angleValues_);
        }
        else*/
		{
			pRobot = manager_.CreateRobot(simpParams_.robotType_, simpParams_.robotBasis_, simpParams_.angleValues_);
		}
		for(std::vector<int>::const_iterator it = simpParams_.locked_.begin(); it !=simpParams_.locked_.end(); ++it)
		{
			pRobot->LockOnCurrent(*it);
		}
	}
	else
	{
        /*if(simpParams_.root)
		{
			pRobot = manager_.CreateRobot(simpParams_.root, simpParams_.robotBasis_);
		}
        else*/
		{
			pRobot = manager_.CreateRobot(simpParams_.robotType_, simpParams_.robotBasis_);
		}
	}
	postureManager_->SetJumpToTarget(simpParams_.jumpToTarget_);
	postureManager_->InitSamples(pRobot,simpParams_.nbSamples_);
	dRobot = new DrawRobot(pRobot);
	if(simpParams_.gait_)
	{
		postureManager_->AddPostureCriteria(enums::postureCriteria::toeOnSpiderGait);
		postureManager_->AddPostureCriteria(enums::postureCriteria::toeOffSpiderGait);
	}
	if(simpParams_.joint_)
	{
		postureManager_->AddPostureCriteria(enums::postureCriteria::toeOffJointLimit);
	}

    simpParams_.fn_.version = DS_VERSION;
    simpParams_.fn_.start   = &simspace::start;
    simpParams_.fn_.step    = &simspace::simLoop;
	simpParams_.fn_.command = simpParams_.command;
    simpParams_.fn_.stop    = 0;

	RegisterHandlers();
    dsSimulationLoop (argc,argv,1600,900, &simpParams_.fn_);
}

void Simulation::Update()
{
	if(!simpParams_.pause_)
	{
		timerHandler_.Update();
	}
	timerHandler_.GetTimer().Stop();
	if(!simpParams_.pause_)
	{
		simpParams_.GetCamera()->Update();
		if(simpParams_.inputHandler_)
		{
			simpParams_.inputHandler_->Update();
		}
		postureManager_->Update(timerHandler_.GetTimer().GetTime());
		timerHandler_.GetTimer().Start();
	}
}

void Simulation::Draw()
{
	drawManager_.Draw();
	dRobot->Draw();
	simpParams_.GetCamera()->DrawArrow();
}

void Simulation::RegisterHandlers()
{
	if(simpParams_.autorotate_) // before motion handler !
	{
		TimerHandled_ABC* listener = new AutoRotate(matrices::Vector3(0,0,1));
		managedTimeListeners_.push_back(listener);
		timerHandler_.Register(listener);
	}
	else if(simpParams_.autorotateleg_) // before motion handler !
	{
		TimerHandled_ABC* listener = new AutoRotateLeg(matrices::Vector3(0,0,1));
		managedTimeListeners_.push_back(listener);
		timerHandler_.Register(listener);
	}
	if(simpParams_.rootTrajectory_) // before motion handler !
	{
		TimerHandled_ABC* listener = new RootTrajectory(simpParams_.rootSpline, simpParams_.initDir_);
		managedTimeListeners_.push_back(listener);
		timerHandler_.Register(listener);
	}
//	timerHandler_.Register(new XboxMotion());
	timerHandler_.Register(&motionHandler_);
	timerHandler_.Start(simpParams_.initTimer_);
}

void Simulation::Reset()
{
	delete dRobot;
	delete pRobot;
	pRobot = manager_.CreateRobot(simpParams_.robotType_, simpParams_.robotBasis_, simpParams_.angleValues_);
	for(std::vector<int>::const_iterator it = simpParams_.locked_.begin(); it !=simpParams_.locked_.end(); ++it)
	{
		pRobot->LockOnCurrent(*it);
	}
	dRobot = new DrawRobot(pRobot);
	simpParams_.GetCamera()->Reset();
	//simpParams_.rootTrajectory_ = false;
	timerHandler_.Reset(simpParams_.initTimer_);
	#ifdef PROFILE
	postureManager_->Log();
	#endif
}
