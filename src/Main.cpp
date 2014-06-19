

#include "MatrixDefs.h"
#include "Pi.h"

#include "ManipManager.h"
#include "PostureManager.h"
#include "draw/DrawManager.h"

#include <drawstuff/drawstuff.h> // The drawing library for ODE;

#ifdef WIN32
#include <windows.h>
#endif

#include <iostream>
#include <vector>


using namespace matrices;
using namespace Eigen;
using namespace std;
using namespace manip_core;

#include<windows.h>
#include<stdio.h>   
#include<tchar.h>

// Use to convert bytes to MB
#define DIV 1048576

// Use to convert bytes to MB
//#define DIV 1024

// Specify the width of the field in which to print the numbers. 
// The asterisk in the format specifier "%*I64d" takes an integer 
// argument and uses it to pad and right justify the number.

#define WIDTH 7

void printmem()
{
  MEMORYSTATUSEX statex;

  statex.dwLength = sizeof (statex);

  GlobalMemoryStatusEx (&statex);


  _tprintf (TEXT("There is  %*ld percent of memory in use.\n"),WIDTH, statex.dwMemoryLoad);
  _tprintf (TEXT("There are %*I64d total Mbytes of physical memory.\n"),WIDTH,statex.ullTotalPhys/DIV);
  _tprintf (TEXT("There are %*I64d free Mbytes of physical memory.\n"),WIDTH, statex.ullAvailPhys/DIV);
  _tprintf (TEXT("There are %*I64d total Mbytes of paging file.\n"),WIDTH, statex.ullTotalPageFile/DIV);
  _tprintf (TEXT("There are %*I64d free Mbytes of paging file.\n"),WIDTH, statex.ullAvailPageFile/DIV);
  _tprintf (TEXT("There are %*I64d total Mbytes of virtual memory.\n"),WIDTH, statex.ullTotalVirtual/DIV);
  _tprintf (TEXT("There are %*I64d free Mbytes of virtual memory.\n"),WIDTH, statex.ullAvailVirtual/DIV);
  _tprintf (TEXT("There are %*I64d free Mbytes of extended memory.\n"),WIDTH, statex.ullAvailExtendedVirtual/DIV);


}

// main object

ManipManager manager;
DrawManager drawManager(manager);
static float xyz[3] = {-10.0,1,6.0};
static float hpr[3] = {0.0,0.0,0.0};

void BuildWorld()
{

	float decal = -0.75;
	// FORWARD OBSTACLES
	Vector3 p1(-11,2 + decal,0.4f);
	Vector3 p2(-5.5,2 + decal,0.4f);
	Vector3 p3(-5.5,-1 + decal,0.4f);
	Vector3 p4(-11,-1 + decal,0.4f);

	manager.SetNextColor(0.7,0.7,0.7);
	manager.AddObstacle(p1,p2,p3,p4);

	Vector3 p11(-5.2, 2 + decal, 0.f);
	Vector3 p21( -1, -1 + decal, 0.f);
	//manager.SetNextColor(0,0,1);
	manager.GenerateStairChess(p11, p21, 0.4, 1.2,  3, 2);

	Vector3 p13(-0.8, 2 + decal, 1.7f);
	Vector3 p23( 3, -1 + decal, 1.7f);
	//manager.SetNextColor(1,0,0);
	manager.GenerateXInclinedPlank(p13, p23);

	p13 = Vector3(-0.8, 2 + decal, 0.1f);
	p23 = Vector3( 3, -1 + decal, 0.1f);
	//manager.SetNextColor(1,0,0);
	manager.GenerateXInclinedPlank(p13, p23);

	Vector3 p131(3.1, 3 + decal, 2.0f);
	Vector3 p231( 6, 1 + decal, 1.5f);
	manager.GenerateXInclinedPlank(p131, p231);

	Vector3 p132(3.1, 0 + decal, 1.5f);
	Vector3 p232( 6, -2 + decal, 2.0f);
	//manager.SetNextColor(0,0,1);
	manager.GenerateXInclinedPlank(p132, p232);


	Vector3 p136(6.2, 1 + decal, 1.5f);
	Vector3 p236( 18, 0.7 + decal, 1.5f);
	manager.GenerateXInclinedPlank(p136, p236);
	
	manager.Initialize(false);
}

static void simLoop (int pause)
{
	drawManager.Draw();
}

void start()
{
    dsSetViewpoint (xyz,hpr);
}

void command(int cmd)   /**  key control function; */
{
	Vector3 trX(0.01, 0, 0);
	Vector3 trY(0,0.01,  0);
	switch (cmd)
	{	
		case '+' :
			drawManager.NextPosture();
		break;
		case '-' :
			drawManager.PreviousPosture();
		break;
		case 't' :
			drawManager.drawPostures(true);
		break;
		case 'u' :
			drawManager.drawPostures(false);
		break;
	}
}


int main(int argc, char *argv[])
{
	//init robot
	Matrix4 robotBasis(MatrixX::Identity(4,4));
	robotBasis(0,3) = -10;
	robotBasis(1,3) = 0.;
	robotBasis(2,3) = 1.8;
	BuildWorld();

	const RobotI* pRobot = manager.CreateRobot(enums::robot::Quadruped, robotBasis);
	
	PostureManager* postureManager = manager.GetPostureManager();
	postureManager->AddPostureCriteria(enums::postureCriteria::toeOnCOM);
	//postureManager->Compute(pRobot, 10000);
	postureManager->SetJumpToTarget(true);
	/* TEST */
	//sg->GenerateSamples(*pRobot, 243);

	Vector3 robotBasis2(0,0,1.8);

	/* trajectory */
	robotBasis2(0) = -9;
	robotBasis2(2) = 1.8;
	postureManager->AddCheckPoint( 1.f, robotBasis2);

	robotBasis2(0) = -8;
	postureManager->AddCheckPoint( 2, robotBasis2);

	robotBasis2(0) = -7;
	postureManager->AddCheckPoint( 3, robotBasis2);

	robotBasis2(0) = -5;
	//robotBasis2(2) = 0.2;
	postureManager->AddCheckPoint( 4, robotBasis2);

	robotBasis2(0) = -2;
	robotBasis2(2) = 0.4 + 1.8;
	postureManager->AddCheckPoint( 5, robotBasis2);

	robotBasis2(0) = -1;
	robotBasis2(2) = 0.8 + 1.8;
	postureManager->AddCheckPoint( 6, robotBasis2);

	robotBasis2(0) = 1.3;
	robotBasis2(2) = 1.3 + 1.8;
	postureManager->AddCheckPoint( 7, robotBasis2);


	robotBasis2(0) = 6;
	robotBasis2(2) = 1.0 + 1.8;
	postureManager->AddCheckPoint( 8, robotBasis2);

	robotBasis2(0) = 12;
	robotBasis2(2) = 1.2 + 1.8;
	postureManager->AddCheckPoint( 9, robotBasis2);

	robotBasis2(0) = 18;
	robotBasis2(2) = 1.7 + 1.8;
	postureManager->AddCheckPoint( 10, robotBasis2);

	/*printmem();
	postureManager->InitSamples(pRobot,1000000);
	printmem();*/

/* trajectory */
	postureManager->Compute(pRobot,1000);
/*TEST */
	#ifdef PROFILE
	postureManager->Log();
	#endif

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