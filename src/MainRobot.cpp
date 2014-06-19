#include "kinematic/Joint.h"
#include "kinematic/Tree.h"
#include "kinematic/RobotFactory.h"
#include "kinematic/Robot.h"
#include "kinematic/SupportPolygon.h"

#include "world/Obstacle.h"
#include "ObstacleGenerator.h"
#include "world/World.h"
#include "sampling/SampleGenerator.h"
#include "PostureSolver.h"

#include "MatrixDefs.h"
#include "Pi.h"
#include "PostureCriteriaToeOffBoundary.h"
#include "PostureCriteriaToeOnCOM.h"


#include "DrawPostures.h"
#include "DrawRobot.h"
#include "DrawWorld.h"
#include "DrawSupportPolygon.h"

#include "Trajectory.h"
#include "PathFollower.h"

#include <drawstuff/drawstuff.h> // The drawing library for ODE;

#ifdef WIN32
#include <windows.h>
#endif

#include <iostream>
#include <vector>


using namespace matrices;
using namespace Eigen;
using namespace std;

// main object
DrawPostures* pDrawPosturesTEST;
PathFollower* pPathFollower;

Robot* pRobot;
DrawRobot* pDrawRobot;
World* pWorld;
DrawWorld* pDrawWorld;

//Direction of robot movement
Vector3* pDirection;

//basis for arrow drawing
const Vector3 from(0.5, 0, 0.5);

//camera
static float xyz[3] = {-10.0,1,3.0};
static float hpr[3] = {0.0,0.0,0.0};

static bool handleDirection(int cmd)
{
	bool found = false;
	switch (cmd)
		{
		case 'o':
			found = true;
			pDirection = new Vector3(0,0,1);
			break;
		case 'l':
			found = true;
			pDirection = new Vector3(0,0,-1);
			break;
		case 'm':
			found = true;
			pDirection = new Vector3(1,0,0);
			break;
		case 'k':
			found = true;
			pDirection = new Vector3(-1,0,0);
			break;
		case 'i':
			found = true;
			pDirection = new Vector3(0,1,0);
			break;
		case 'p':
			found = true;
			pDirection = new Vector3(0,-1,0);
			break;
		}
	return found;
}

void command(int cmd)   /**  key control function; */
{
	if (!handleDirection(cmd))
	{
		Vector3 trX(0.01, 0, 0);
		Vector3 trY(0,0.01,  0);
		switch (cmd)
		{	
		case '+' :
			pDrawPosturesTEST->Next();
		break;
		case '-' :
			pDrawPosturesTEST->Previous();
		break;
		case 't' :
			pRobot->Reset();
		break;
		case 'r' :
			pRobot->Rest();
		break;
		case 'z' :
			xyz[0] += 0.01f;
			pRobot->Translate(trX);
		dsSetViewpoint (xyz,hpr);
		break;
		case 's' :
			xyz[0] -= 0.01f;
			pRobot->Translate(-trX);
		dsSetViewpoint (xyz,hpr);
		break;
		case 'q' :
			xyz[1] += 0.01f;
			pRobot->Translate(trY);
		dsSetViewpoint (xyz,hpr);
		break;
		case 'd' :
			xyz[1] -= 0.01f;
			pRobot->Translate(-trY);
		dsSetViewpoint (xyz,hpr);
		break;
		}
	}
}


void BuildWorld(World& world)
{
	// FORWARD OBSTACLES
	Vector3 p1(-11,2,0.4f);
	Vector3 p2(-5.5,2,0.4f);
	Vector3 p3(-5.5,-1,0.4f);
	Vector3 p4(-11,-1,0.4f);

	world.AddObstacle(new Obstacle(p1,p2,p3,p4));

	Vector3 p11(-5.2, 2, 0.f);
	Vector3 p21( -1, -1, 0.f);
	//Vector3 p121(pose+2,posey+2,0.2f);
	//Vector3 p141(pose,posey,0.2f);

	ObstacleGenerator generator;
	//generator.GenerateChess(world, p11, p21, 0.4,  3);
	//generator.GenerateStair(world, p11, p21, 0.4, 1.2,  3);
	//generator.GenerateStairChess(world, p11, p21, 0.4, 1.2,  3, 2);

	
	/*Vector3 p133(6.2, 1.5f, 3.0f);
	Vector3 p233(  8, 1.5f, 1.f);
	generator.GenerateXInclinedPlank(world, p133, p233);*/

	//Vector3 p135(6.2,1.5, 1.7f);
	//Vector3 p235( 8, 1.2, 1.7f);
	//generator.GenerateXInclinedPlank(world, p135, p235);

	//Vector3 p137(6.2,-0.5, 1.7f);
	//Vector3 p237( 8, -0.8, 1.7f);
	//generator.GenerateXInclinedPlank(world, p137, p237);

	Vector3 p136(-5.2, 2, 0.f);
	Vector3 p236( -4, -1, 6.f);
	generator.GenerateYInclinedPlank(world, p136, p236);

	//Vector3 p134(6.2, -0.5f, 3.0f);
	//Vector3 p234(  8, -0.5f, 1.f);
	//generator.GenerateXInclinedPlank(world, p134, p234);





}

/**
Drawstuff stuff :
*/
static void DrawArrow(const Vector3& from, const Vector3& dir)
{
	float Identity [12] = { 1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f };
	
	Vector3 destdir = from+dir;
	float fr[3];
	float t[3];
	vect3ToArray(fr, from); 
	vect3ToArray(t, destdir); 
	dsDrawLine  (fr, t);
	dsDrawSphere(t, Identity, 0.02f);
}


static void simLoop (int pause)
{
	//pPathFollower->Update();
	//pRobot->Move(*pDirection, *pWorld);
	//pDrawRobot->Draw();
	pDrawWorld->Draw();
	DrawArrow(from, *pDirection);
/*TEST*/
	Vector3 com = matrices::matrix4TimesVect3( pRobot->ToWorldCoordinates(), pRobot->ComputeCom());
	//Vector3 com = pRobot->ComputeCom();
float Identity [12] = { 1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f };
float t[3];
t[0] = (float)(com(0));
t[1] = (float)(com(1));
t[2] = (float)(com(2));
//dsDrawSphere(t, Identity, 0.1f);
pDrawPosturesTEST->DrawOne();
//SupportPolygon sup(*pRobot);
//DrawSupportPolygon dsup(*pRobot, sup);
//dsup.Draw();
/*TEST*/
}

void start()
{
    dsSetViewpoint (xyz,hpr);
}

int main(int argc, char *argv[])
{
	//init robot
	Matrix4 robotBasis(MatrixX::Identity(4,4));
	robotBasis(0,3) = -8;

	factories::RobotFactory robotfact_;
	//pRobot = robotfact_.CreateRobot(factories::Human, robotBasis);
	pRobot = robotfact_.CreateRobot(factories::Quadruped, robotBasis);

	//init DrawRobot
	DrawRobot dRobot(*pRobot);
	pDrawRobot = &dRobot;

	// for now mostly contains obstacles
	World world;
	BuildWorld(world);
	pWorld = &world;

	//init DrawWorld
	DrawWorld dWorld(world);
	pDrawWorld = &dWorld;

	//movement direction
	Vector3 direction(1,0,0);
	pDirection = &direction;


/* TEST */
SampleGenerator* sg = SampleGenerator::GetInstance();
sg->GenerateSamples(*pRobot,10000);
//sg->GenerateSamples(*pRobot, 243);
PostureSolver ps(world);
ps.AddToeOffCriteria(new PostureCriteriaToeOffBoundary());
ps.AddToeOnCriteria (new PostureCriteriaToeOnCOM());

Vector3 robotBasis2(0,0,0);
Trajectory traj;

/* trajectory */
//robotBasis2(0) = -10;
//traj.AddCheckPoint( 2, robotBasis2);
//robotBasis2(0) = -4;
//robotBasis2(2) = 0.2;
//traj.AddCheckPoint( 5, robotBasis2);


robotBasis2(0) = -7.8;
robotBasis2(2) = 0.1;
traj.AddCheckPoint( 1, robotBasis2);

robotBasis2(0) = -7.8;
robotBasis2(2) = 0.2;
traj.AddCheckPoint( 2, robotBasis2);

robotBasis2(0) = -7.8;
robotBasis2(2) = 0.3;
traj.AddCheckPoint( 3, robotBasis2);

//robotBasis2(0) = -1;
//robotBasis2(2) = 0.2;
//traj.AddCheckPoint( 22, robotBasis2);
//
//robotBasis2(0) = -0.7;
//robotBasis2(2) = 0.3;
//traj.AddCheckPoint( 22, robotBasis2);
//
//robotBasis2(0) = -0.5;
//robotBasis2(2) = 0.3;
//traj.AddCheckPoint( 23, robotBasis2);
//
//robotBasis2(0) = 0;
//robotBasis2(2) = 0.4;
//traj.AddCheckPoint(24, robotBasis2);
//
//robotBasis2(0) = 1;
//robotBasis2(2) = 0.4;
//traj.AddCheckPoint(25, robotBasis2);


/* trajectory */

// temp test project
Vector3 from(1,1,0);
Vector3 to(0,0,1);
Matrix3 res;
GetRotationMatrix(from, to, res);
Vector3 result = res*from;
// 

pDrawPosturesTEST = new DrawPostures(ps);
ps.CreatePostures(*pRobot, traj);

pPathFollower = new PathFollower(*pRobot, traj, ps);
//pPathFollower->Start();
/*TEST */

	/*
	drawstuff stuff*/
	dsFunctions fn;
    fn.version = DS_VERSION;
    fn.start   = &start;
    fn.step    = &simLoop;
	fn.command = &command;
    fn.stop    = 0;
    fn.path_to_textures = "./textures";
    dsSimulationLoop (argc,argv,800,600,&fn);

    return 0;
}
