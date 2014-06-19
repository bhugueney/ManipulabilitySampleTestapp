

#include "MatrixDefs.h"
#include "Pi.h"

#include "ManipManager.h"
#include "PostureManager.h"
#include "draw/DrawRobot.h"
#include "draw/DrawManager.h"
#include "API/TreeI.h"
#include "API/RobotI.h"
#include "IKSolver/IKSolver.h"

#include "XBOXController/Controller.h"

#include <drawstuff/drawstuff.h> // The drawing library for ODE;

#ifdef WIN32
#include <windows.h>
#endif

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


static float xyz[3] = {-10.0,1,3.0};
//static float hpr[3] = {0.0,0.0,0.0};
static float hpr[3] = {-90.0,0.0,0.0};
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
	Vector3 p1(-11,2,0.4f);
	Vector3 p2(-7,2,0.4f);
	Vector3 p3(-7,-2,0.4f);
	Vector3 p4(-11,-2,0.4f);

	manager.AddObstacle(p1,p2,p3,p4);

	p1 = Vector3(-7 , 1. , 0.7);
	p4 = Vector3(-7., -2, 0.4);
	//manager.GenerateVerticalChess(p1, p4, 0);

	p1 = Vector3(-7 , 2. , 0.7);
	p2 = Vector3(-5., 2, 0.7);
	p3 = Vector3(-5 , -2. , 0.7);
	p4 = Vector3(-7., -2, 0.7);
	manager.AddObstacle(p1,p2,p3,p4);


	p1 = Vector3(-5 , 2. , 1.2);
	p2 = Vector3(-3., 2, 1.2);
	p3 = Vector3(-3 , -2. , 1.2);
	p4 = Vector3(-5., -2, 1.2);
	manager.AddObstacle(p1,p2,p3,p4);

	/*p1 = Vector3(-7 , 2. , 0.9);
	p4 = Vector3(-7., 1.7, 0.6);
	manager.GenerateVerticalChess(p1, p4, 0);

	p1 = Vector3(-7 , 1.7 , 1.5);
	p4 = Vector3(-7., 1.4 , 1.2);
	manager.GenerateVerticalChess(p1, p4, 0);
	
	p1 = Vector3(-7 , 0.  , 1.7);
	p4 = Vector3(-7., -0.3, 1.4 );
	manager.GenerateVerticalChess(p1, p4, 0);
	
	p1 = Vector3(-7 , 1.  , 2.8);
	p4 = Vector3(-7., 0.7, 2.5 );
	manager.GenerateVerticalChess(p1, p4, 0);

	p1 = Vector3(-7 , 1.  , 3.8);
	p4 = Vector3(-7., 0.7, 3.5 );
	manager.GenerateVerticalChess(p1, p4, 0);
	
	p1 = Vector3(-7 , 3.  , 3.8);
	p4 = Vector3(-7., 2.7, 3.5 );
	manager.GenerateVerticalChess(p1, p4, 0);
	
	p1 = Vector3(-7 , 0.5  , 4.4);
	p4 = Vector3(-7., 0.2, 4.1 );
	manager.GenerateVerticalChess(p1, p4, 0);

	p1 = Vector3(-7 , 2.5  , 5.2);
	p4 = Vector3(-7., 2.2, 4.9 );
	manager.GenerateVerticalChess(p1, p4, 0);

	p1 = Vector3(-7 , -1.8  , 5.2);
	p4 = Vector3(-7., -2.1, 4.9 );
	manager.GenerateVerticalChess(p1, p4, 0);
	
	p1 = Vector3(-7 , -1.8  , 6);
	p4 = Vector3(-7., -2.1, 5.7 );
	manager.GenerateVerticalChess(p1, p4, 0);

	p1 = Vector3(-7 , -0.8  , 7.5);
	p4 = Vector3(-7., -1.1, 7.2 );
	manager.GenerateVerticalChess(p1, p4, 0);

	p1 = Vector3(-7 , -0.8  , 9);
	PosePrise(p1);*/


	//random wall ; chance of 10 % to generate prise each time
	/*float z = 15.f;
	int val;
	while( z > 0.8f )
	{
		float y = 5.f;
		while( y > -5.f )
		{
			val = rand() % 100;
			if (val < 20. )
			{
				float x = -6.7f + ((float)(rand() % 60)) / 100.f; 
				PosePrise( Vector3( -7.f, y, z));
			}
			y -= 0.4f;
		}
		z -= 0.4f;
	}*/
	
	//manager.AddObstacle(p1,p2,p3,p4);


	/*for(int i = 0; i < 10; ++ i)
	{
		p1(2) = p1(2) +0.5f;
		p2(2) = p2(2) +0.5f;
		p3(2) = p3(2) +0.5f;
		p4(2) = p4(2) +0.5f;
		manager.AddObstacle(p1,p2,p3,p4);
	}*/

	/*Vector3 p11(-5.2, 2, 0.f);
	Vector3 p21( -1, -1, 0.f);
	manager.GenerateStairChess(p11, p21, 0.4, 1.2,  3, 2);

	Vector3 p13(-0.8, 2, 1.7f);
	Vector3 p23( 3, -1, 1.7f);
	manager.GenerateXInclinedPlank(p13, p23);

	Vector3 p131(3.1, 3, 2.0f);
	Vector3 p231( 6, 1, 1.5f);
	manager.GenerateXInclinedPlank(p131, p231);

	Vector3 p132(3.1, 0, 1.5f);
	Vector3 p232( 6, -2, 2.0f);
	manager.GenerateXInclinedPlank(p132, p232);


	Vector3 p136(6.2, 1, 1.5f);
	Vector3 p236( 18, 0.7, 1.5f);
	manager.GenerateXInclinedPlank(p136, p236);*/
	
	manager.Initialize();
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
	if( xL != 0 || yL != 0)
	{
		double trans[3] = {((double)xL * slowDownFactor*2), 0., ((double)yL * slowDownFactor)};
		robot->Translate(trans);
		oldDir_ = Vector3(xL, 0, yL);
		oldDir_.normalize();
		transformed = true;
	}
	if( yR != 0)
	{
		transformed = true;
		Rotate(pRobot, matrices::Roty3(-yR * slowDownFactor*2));

	}
	if( xR != 0)
	{
		//transformed = true;
		//Rotate(pRobot, matrices::Rotx3(xR * slowDownFactor*3));

	}
	if(transformed)
	{
		matrices::Matrix4 transform;
		double transf[16];
		robot->ToWorldCoordinates(transf);
		matrices::array16ToMatrix4(transf, transform);
		xyz[0] = transform(0,3);// - 5;
		xyz[1] = transform(1,3)+ 5;
		xyz[2] = transform(2,3);
		//postureManager->NextPosture(pRobot,matrices::matrix4TimesVect3(transform, oldDir_));
		postureManager->NextPosture(pRobot, oldDir_);
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

static void simLoop (int pause)
{
	XboxControl(pRobot);
	bool ok = true;
	do
	{
		/*ok =*/ solver.QuickStepClampingToTargets(pRobot);
	}
	while(!ok);
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
	srand((unsigned int)(time(0))); //Init Random generation
	//init robot
	Matrix4 robotBasis(MatrixX::Identity(4,4));
	robotBasis(0,3) = -8;
	robotBasis(1,3) = 1.;
	robotBasis(2,3) = 2.;

	BuildWorld();

	pRobot = manager.CreateRobot(enums::robot::HumanWalk, robotBasis);
	dr = new DrawRobot(pRobot);
	/* TEST */
	//sg->GenerateSamples(*pRobot, 243);

	Vector3 robotBasis2(0,0,0);

	
/* trajectory */
	postureManager->InitSamples(pRobot,1000);
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
    dsSimulationLoop (argc,argv,800,600,&fn);

    return 0;
}
