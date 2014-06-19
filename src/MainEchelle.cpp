

#include "MatrixDefs.h"
#include "Pi.h"

#include "ManipManager.h"
#include "PostureManager.h"
#include "draw/DrawRobot.h"
#include "draw/DrawManager.h"
#include "draw/DrawTrajectory.h"
#include "API/TreeI.h"
#include "API/RobotI.h"
#include "IKSolver/IKSolver.h"

#include "XBOXController/Controller.h"


#ifdef WIN32
#include <windows.h>
#endif

#include <drawstuff/drawstuff.h> // The drawing library for ODE;

#include "MouseTrack.h"
#include "MainTools.h"

#include <iostream>
#include <vector>
#include <time.h>

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

bool takeInput = true;

gameState::egameState state = gameState::padDrive;

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
DrawTrajectory drawTrajectory;
const IKSolver& solver = manager.GetIkSolver();
xbox::Controller controller(1);
matrices::Vector3 oldDir_(1., 0., 0.);
matrices::Vector3 screenArrowPosition_(1., 0., 0.);
matrices::Vector2 cursorCoordinates(0., 0.);
//float arrowDist[3] = {1.0, 1.0, 1.0};


//mouse control stuff
float arrowOr[3] = {1.0, 1.0, 1.0};

float idMatrix[12];
float idMat[12];
double tmp[16];

static float xyz[3] = {-10.0,1,3.0};
static float hpr[3] = {0.,0.0,0.0};
RobotI* pRobot;

void PosePrise(const matrices::Vector3& p)
{
	Vector3 p1(p(0), p(1)+0.1, p(2)+0.1);
	Vector3 p2(p(0), p(1)-0.2, p(2)-0.2);
	manager.GenerateVerticalChess(p1, p2, 0);
}


void BuildWorld()
{
	// FORWARD OBSTACLES
	Vector3 p1(-11,6,-0.1f);
	Vector3 p2(10,6,-0.1f);
	Vector3 p3(10,-2,-0.1f);
	Vector3 p4(-11,-2,-0.1f);

	//manager.AddObstacle(p1,p2,p3,p4);
	//manager.AddGround(p1,p2,p3,p4);

	p1 = Vector3(50,200,10);
	p2 = Vector3(50,-200,10.f);
	p3 = Vector3(50,-200,0.f);
	p4 = Vector3(50,200,0.f);

	/*p1 = Vector3(-6,20,10.f);
	p2 = Vector3(-6,-20,10.f);
	p3 = Vector3(-6,-20,0.f);
	p4 = Vector3(-6,20,0.f);
	manager.AddWall(p1,p2,p3,p4);*/

	//random wall ; chance of 10 % to generate prise each time
	manager.SetNextColor(1,0.,0);
	float z = 15.f;
	int val;
	while( z > 0.8f )
	{
		float y = 5.f;
		while( y > -5.f )
		{
			val = rand() % 100;
			if (val < 40. )
			{
				/*if(val<7.)
				{
					manager.SetNextColor(1,0,0,1);
				}
				else if(val<14.)
				{
					manager.SetNextColor(0,0,1,1);
				}
				else
				{
					manager.SetNextColor(1,1,0,1);
				}*/
				float x = -6.7f + ((float)(rand() % 70)) / 100.f; 
				PosePrise( Vector3( -7.f, y, z));
			}
			y -= 0.4f;
		}
		z -= 0.4f;
	}	
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

int nbtraj = 1;
void XboxControlPosture(RobotI * robot)
{
	const RobotI* changedPosition(0);
	controller.Update();
	if(controller.AnyReleased((int)xbox::Buttons::Back))
	{
		postureManager->SetJumpToTarget(false);
		state = gameState::padDrive;
		manager.GetPostureManager()->ResetTrajectory();
		drawManager.Clear();
		drawTrajectory.Clear();
		return;
	}
	float xL, yL;
	const float slowDownFactor = 0.005f;
	if(controller.AnyReleased((int)xbox::Buttons::A))
	{
		input::GetMouseClicked(cursorCoordinates, input::mousePos);
		robot->ToWorldCoordinates(tmp);
		//manager.GetPostureManager()->AddCheckPoint(0.1f, matrices::Vector3(tmp[3], tmp[7], tmp[11]));
		matrices::Vector3 val(tmp[3], input::mousePos[1], input::mousePos[2]);
		drawTrajectory.AddPoint(val);
		manager.GetPostureManager()->AddCheckPoint(nbtraj++, val );
	}
	else if(controller.AnyReleased((int)xbox::Buttons::LeftShoulder))
	{
		/*changedPosition =*/ drawManager.PreviousPosture();
	}
	else if(controller.AnyReleased((int)xbox::Buttons::RightShoulder))
	{
		/*changedPosition =*/ drawManager.NextPosture();
	}
	else if(controller.AnyReleased((int)xbox::Buttons::DPadUp))
	{
		/*changedPosition =*/ drawManager.drawPostures(true);
	}
	else if(controller.AnyReleased((int)xbox::Buttons::DPadDown))
	{
		/*changedPosition =*/ drawManager.drawPostures(false);
	}
	else if(controller.AnyReleased((int)xbox::Buttons::Start))
	{
		drawManager.SetTransparencyForAll(false);
		postureManager->SetJumpToTarget(true);
		state = gameState::mouseDraw;
		robot->ToWorldCoordinates(tmp);
		manager.GetPostureManager()->AddCheckPoint(0.1f, matrices::Vector3(tmp[3], tmp[7], tmp[11]));
		manager.GetPostureManager()->ComputeOnline(robot, 1000000);		
		#ifdef PROFILE
		postureManager->Log();
		#endif
	}
	else if(controller.AnyReleased((int)xbox::Buttons::X))
	{
		drawManager.SetTransparencyForAll(true);
		postureManager->SetJumpToTarget(false);
		state = gameState::mouseDraw;
		robot->ToWorldCoordinates(tmp);
		manager.GetPostureManager()->AddCheckPoint(0.1f, matrices::Vector3(tmp[3], tmp[7], tmp[11]));
		manager.GetPostureManager()->ComputeOnline(robot, 1000000);		
		#ifdef PROFILE
		postureManager->Log();
		#endif
	}
	controller.GetLeftStickMovingVector(xL, yL);
	if( xL != 0 || yL != 0)
	{
		cursorCoordinates(0) += xL * slowDownFactor;
		cursorCoordinates(1) -= yL * slowDownFactor;
		input::GetMouseClicked(cursorCoordinates, input::mousePos);	
	}
	else if(controller.AllPressed((int)xbox::Buttons::Y))
	{
		manager.GetPostureManager()->ResetTrajectory();
		drawManager.Clear();
		drawTrajectory.Clear();
		state = gameState::mousePlan;
		changedPosition = robot;
	}
	/*camera stuff*/
	if(changedPosition)
	{
		matrices::Matrix4 transform;
		double transf[16];
		changedPosition->ToWorldCoordinates(transf);
		matrices::array16ToMatrix4(transf, transform);
		xyz[0] = transform(0,3) - 12.f;
		xyz[1] = transform(1,3); // - 5.f;
		xyz[2] = transform(2,3) + 2.f;
		dsSetViewpoint (xyz,hpr);
	}
}

void XboxControlAnim(RobotI * robot)
{
	//if (! takeInput) return;
	controller.Update();	
	if(controller.AnyReleased((int)xbox::Buttons::Back))
	{
		state = gameState::mousePlan;
		return;
	}
	float xL, yL;
	float xR, yR;
	bool transformed = false;
	const float slowDownFactor = 0.0042f;
	controller.GetLeftStickMovingVector(xL, yL);
	controller.GetRightStickMovingVector(xR, yR);
	if( xL != 0 || yL != 0)
	{
		double trans[3] = {0.,-((double)xL * slowDownFactor*3), ((double)yL * slowDownFactor * 2)};
		robot->Translate(trans);
		oldDir_ = Vector3(0.2, -xL, yL);
		oldDir_.normalize();
		transformed = true;
	}
	if( yR != 0)
	{
		//transformed = true;
		//Rotate(pRobot, matrices::Roty3(-yR * slowDownFactor*2));

	}
	if( xR != 0)
	{
		//transformed = true;
		//Rotate(pRobot, matrices::Rotz3(xR * slowDownFactor*3));

	}
	if(controller.AllPressed((int)xbox::Buttons::A))
	{
		double trans[3] = {slowDownFactor*2, 0., 0.};
		robot->Translate(trans);
		oldDir_(0) = oldDir_(0) + 1.;
		oldDir_.normalize();
		transformed = true;
		drawManager.Clear();
	}
	if(controller.AllPressed((int)xbox::Buttons::B))
	{
		double trans[3] = {-slowDownFactor*2, 0., 0.};
		robot->Translate(trans);
		oldDir_(0) = oldDir_(0) - 1.;
		oldDir_.normalize();
		transformed = true;
	}
	if(transformed)
	{
		postureManager->NextPosture(pRobot, oldDir_);
		matrices::Matrix4 transform;
		double transf[16];
		robot->ToWorldCoordinates(transf);
		matrices::array16ToMatrix4(transf, transform);
		xyz[0] = transform(0,3) - 12.f;
		xyz[1] = transform(1,3); // - 5.f;
		xyz[2] = transform(2,3) + 2.f;/*
		screenArrowPosition_(0) = xyz[0] + 10.f;
		screenArrowPosition_(1) = xyz[1] - 5.f;
		screenArrowPosition_(2) = xyz[2] + 2.f;*/
		screenArrowPosition_(0) = transform(0,3) - 2.f;
		screenArrowPosition_(1) = transform(1,3) + 2.f;
		screenArrowPosition_(2) = transform(2,3) + 1.f;
		//postureManager->NextPosture(pRobot,matrices::matrix4TimesVect3(transform, oldDir_));
		dsSetViewpoint (xyz,hpr);
	}
	if(controller.AllPressed((int)xbox::Buttons::Y))
	{
		Matrix4 robotBasis(MatrixX::Identity(4,4));
		robotBasis(0,3) = -8;
		robotBasis(1,3) = 1.;
		robotBasis(2,3) = 2.;
		enums::robot::eRobots type = pRobot->GetType();
		pRobot->Release();
		pRobot = manager.CreateRobot(type, robotBasis);
		delete dr;
		dr = new DrawRobot(pRobot);
	}
}

void DrawArrow()
{
	matrices::vect3ToArray(arrowOr, screenArrowPosition_ + oldDir_ / 2.);
	dsDrawCapsule(arrowOr, idMatrix, 1, 0.05);
	//glutSolidCone(/*GLdouble base*/0.05, /*GLdouble height*/0.05, /*GLint slices*/10, /*GLint stacks*/10);
}

void Draw(int pause)
{
	if(state != gameState::mouseDraw)
	{
		dr->Draw();
	}
	drawManager.Draw();
	if(state != gameState::padDrive)
	{
		drawTrajectory.Draw();
	}
	// draw direction
	else if(state == gameState::padDrive)
	{
		drawTrajectory.Draw();
		DrawArrow();
	}
	Matrix3 transformCube;
	GetRotationMatrix(unitz, oldDir_, transformCube);
	matrix3ToArray(idMatrix, transformCube);
}

static void simLoop (int pause)
{
	Draw(pause);
	switch(state)
	{
		case gameState::padDrive:
		{
			XboxControlAnim(pRobot);
			break;
		}
		case gameState::mouseDraw:
		case gameState::mousePlan:
		{
			XboxControlPosture(pRobot);
			break;
		}
		default:
			break;
	}
	int i = 0;
	bool ok = true;
	do
	{
		++i;
		takeInput = solver.StepClampingToTargets(pRobot, oldDir_);
	}
	while(i <2 || !ok);
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
	//glutInit(&argc, argv);
//	/glutMouseFunc(input::MouseFunc);

	srand((unsigned int)(time(0))); //Init Random generation
	//init robot
	matrices::matrixID(idMatrix);
	matrices::matrixID(idMat);
	Matrix4 robotBasis(MatrixX::Identity(4,4));
	robotBasis(0,3) = -10;
	robotBasis(1,3) = 1.;
	robotBasis(2,3) = 3.;

	BuildWorld();

	pRobot = manager.CreateRobot(enums::robot::Human, robotBasis);
	dr = new DrawRobot(pRobot);
	/* TEST */
	//sg->GenerateSamples(*pRobot, 243);

	Vector3 robotBasis2(0,0,0);

	
/* trajectory */
	postureManager->SetJumpToTarget(false);
	postureManager->InitSamples(pRobot,250000);
/*TEST */

	/*
	drawstuff stuff*/
	dsFunctions fn;
    fn.version = DS_VERSION;
    fn.start   = &start;
    fn.step    = &simLoop;
	fn.command = &command;
    fn.stop    = 0;
    fn.path_to_textures = "../textures";
	//glutMainLoop();
    dsSimulationLoop (argc,argv,800,600,&fn);

    return 0;
}