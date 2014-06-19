

#include "MatrixDefs.h"
#include "Pi.h"

#include "ManipManager.h"
#include "PostureManager.h"
#include "draw/DrawRobot.h"
#include "draw/DrawManager.h"
#include "API/TreeI.h"
#include "API/RobotI.h"
#include "IKSolver/IKSolver.h"
#include "Timer.h"

#include "XBOXController/Controller.h"

#include <drawstuff/drawstuff.h> // The drawing library for ODE;

#ifdef WIN32
#include <windows.h>
#endif

#include <iostream>
#include <vector>


namespace matrices
{
	const Vector3 unitx = Vector3(1, 0, 0);
	const Vector3 unity = Vector3(0, 1, 0);
	const Vector3 unitz = Vector3(0, 0, 1);
	const Vector3 zero  = Vector3(0, 0, 0);
}

using namespace matrices;
using namespace Eigen;
using namespace std;
using namespace manip_core;

// main object

void Translate(RobotI* robot, const matrices::Vector3 dir)
{
	double tab [3];
	matrices::vect3ToArray(tab, dir);
	robot->Translate(tab);
}


ManipManager manager;
DrawManager drawManager(manager);
PostureManager* postureManager = manager.GetPostureManager();
DrawRobot* dr;
const IKSolver& solver = manager.GetIkSolver();
xbox::Controller controller(1);
matrices::Vector3 oldDir_(1., 0., 0.);

unsigned long lastTime;
Timer timer;


static float xyz[3] = {-15,0.75,1.0};
static float hpr[3] = {70,-0.0,0.0};
RobotI* pRobot;


void BuildWorld()
{
	float decal = 0.75;
	// FORWARD OBSTACLES
	manager.SetNextColor(0,0.63,0.82);

	Vector3 p11(-6, 4, 1.f);
	Vector3 p21( -1, -2 + decal, 1.f);
	manager.SetNextColor(0,0,1);
	manager.GenerateUnevenChess(p11, p21,1.6, 4);
	
	p11 = Vector3(-11, 4, 1.f);
	p21 = Vector3(-7., -2 + decal, 1.f);
	manager.GenerateUnevenChess(p11, p21,1.4, 4);

	
	p11 = Vector3(0, 4, 1.f);
	p21 = Vector3(5., -2 + decal, 1.f);
	manager.GenerateUnevenChess(p11, p21,1.8, 4);


	manager.Initialize(false);
}


void Rotate(RobotI* robot, const matrices::Matrix3& rotation)
{
	matrices::Matrix4 transform;
	double transf[16];
	robot->ToWorldCoordinates(transf);
	matrices::array16ToMatrix4(transf, transform);
	transform.block<3,3>(0,0) = transform.block<3,3>(0,0) * rotation;
	matrices::matrixTo16Array(transf, transform);
	robot->SetTransform(transf);
}

void XboxControl(RobotI * robot)
{
	controller.Update();
	float xL, yL;
	float xR, yR;
	bool transformed = false;
	const float slowDownFactor = 0.01f;
	controller.GetLeftStickMovingVector(xL, yL);
	controller.GetRightStickMovingVector(xR, yR);
	if(xL == 0 && yL == 0 && yR == 0 && xR == 0)
	{
		timer.Stop();
	}
	else if( !timer.IsRunning())
	{
		timer.Start();
		lastTime = timer.GetTime();
	}
	if( xL != 0 || yL != 0)
	{
		
		double trans[3] = {((double)xL * slowDownFactor*2), 0., ((double)yL * slowDownFactor)};
		robot->Translate(trans);
		oldDir_ = Vector3(xL, 0, yL);
		oldDir_.normalize();
		transformed = true;
	}
	//if( yR != 0)
	//{
	//	transformed = true;
	//	double trans[3] = {0., ((double)yR * slowDownFactor), 0. };
	//	robot->Translate(trans);
	//	//Rotate(pRobot, matrices::Roty3(-yR * slowDownFactor*2));

	//}
	if( xR != 0)
	{
		transformed = true;
		Rotate(pRobot, matrices::Rotx3(xR * slowDownFactor*3));

	}
	if(transformed)
	{
		matrices::Matrix4 transform;
		double transf[16];
		robot->ToWorldCoordinates(transf);
		matrices::array16ToMatrix4(transf, transform);
		//postureManager->NextPosture(pRobot,matrices::matrix4TimesVect3(transform, oldDir_));
		xyz[0] = transform(0,3) - 0.f;
		xyz[1] = transform(1,3)- 9.f;
		xyz[2] = transform(2,3) + 2.5f;
		//postureManager->NextPosture(pRobot,matrices::matrix4TimesVect3(transform, oldDir_));
		postureManager->NextPosture(pRobot, oldDir_);
		dsSetViewpoint (xyz,hpr);
	}
	if(controller.AllPressed((int)xbox::Buttons::Y))
	{
		Matrix4 robotBasis(MatrixX::Identity(4,4));
		robotBasis(0,3) = -10;
		robotBasis(1,3) = 1.65;
		robotBasis(2,3) = 1.7;
		enums::robot::eRobots type = pRobot->GetType();
		pRobot->Release();
		pRobot = manager.CreateRobot(type, robotBasis);
		delete dr;
		dr = new DrawRobot(pRobot);
		timer.Stop();
		timer.Reset();
		
		#ifdef PROFILE
		postureManager->Log();
		#endif
	}
}

static void simLoop (int pause)
{
	XboxControl(pRobot);
	if(timer.IsRunning())
	{
		unsigned long time = timer.GetTime() - lastTime;
		lastTime = timer.GetTime();
		postureManager->Update(time);
	}
	bool ok = true; int i = 0;
	do
	{
		++i;
		ok = solver.QuickStepClampingToTargets(pRobot);
	}
	while(!ok && i < 1);
	dr->Draw();
	drawManager.Draw();
}

void start()
{
    dsSetViewpoint (xyz,hpr);
}

void command(int cmd)   /**  key control function; */
{
	const Vector3 trX(0.03, 0, 0);
	const Vector3 trY(0,0.01,  0);
	switch (cmd)
	{	
		case '+' :
			Translate(pRobot, trX);
			postureManager->NextPosture(pRobot,matrices::unitx);
		break;
		case '-' :
			Translate(pRobot, -trX);
			postureManager->NextPosture(pRobot,matrices::unitx);
			//drawManager.PreviousPosture();
		break;
		case 'a' :
			Rotate(pRobot, matrices::Roty3(-0.1));
			postureManager->NextPosture(pRobot,matrices::unity);
			//drawManager.PreviousPosture();
		break;
	}
}


int main(int argc, char *argv[])
{
	postureManager->AddPostureCriteria(enums::postureCriteria::toeOnSpiderGait);
	postureManager->AddPostureCriteria(enums::postureCriteria::toeOffSpiderGait);

	postureManager->SetJumpToTarget(true);


	//init robot
	Matrix4 robotBasis(MatrixX::Identity(4,4));
	robotBasis(0,3) = -10;
	robotBasis(1,3) = 0.75;
	robotBasis(2,3) = 0.6;

	BuildWorld();

	pRobot = manager.CreateRobot(enums::robot::Spider, robotBasis);
	dr = new DrawRobot(pRobot);
	/* TEST */
	//sg->GenerateSamples(*pRobot, 243);

	Vector3 robotBasis2(0,0,0);

	
/* trajectory */
	postureManager->InitSamples(pRobot,10000);
/*TEST */

	/*
	drawstuff stuff*/
	dsFunctions fn;
    fn.version = DS_VERSION;
    fn.start   = &start;
    fn.step    = &simLoop;
	fn.command = &command;
    fn.stop    = 0;
    fn.path_to_textures = "../textures/wall";
    dsSimulationLoop (argc,argv,800,600,&fn);

    return 0;
}