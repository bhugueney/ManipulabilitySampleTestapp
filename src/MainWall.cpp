

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



void DrawEllipse( float rx =1.f,float ry=1.f, float rz = 0.5f)
{
	float sina;
	float a, b, x, y, z;

	float pas = 0.01f;
	glColor3f(1.0f, 1.0f, 0.0f);


	glBegin(GL_POINTS);

	for (a=0.0f; a <= Pi; a+=pas)
	{
		z = rz*cosf(a);
		sina = sinf (a);
		for (b=-Pi; b <= Pi; b+=pas)
		{			
			  	  
			x =rx*sina * cosf(b);
		  
			y =ry*sina * sinf(b);
	  
			//normal(x, y, z);
			glVertex3f(x,y,z) ;

		}
	}
	glEnd();
}


double robotCoord[16];
Matrix4 currentTransform;
Vector3 pos, v1, v2, v3;
double from[3];
double u1[3];
double u2[3];
double u3[3];
double sig1, sig2, sig3;
static void setTransform (const float pos[3], const float R[12])
{
  GLfloat matrix[16];
  matrix[0]=R[0];
  matrix[1]=R[4];
  matrix[2]=R[8];
  matrix[3]=0;
  matrix[4]=R[1];
  matrix[5]=R[5];
  matrix[6]=R[9];
  matrix[7]=0;
  matrix[8]=R[2];
  matrix[9]=R[6];
  matrix[10]=R[10];
  matrix[11]=0;
  matrix[12]=pos[0];
  matrix[13]=pos[1];
  matrix[14]=pos[2];
  matrix[15]=1;
  glPushMatrix();
  glMultMatrixf (matrix);
}

void DrawEllipse(RobotI* rob)
{

	TreeI* tree = rob->GetTreeI(3);
	pRobot->ToWorldCoordinates(robotCoord);
	matrices::array16ToMatrix4(robotCoord, currentTransform);
	tree->EndEffectorPosition(from);
	matrices::arrayToVect3(from, pos);
	pos = matrices::matrix4TimesVect3(currentTransform, pos);
	matrices::vect3ToArray(from, pos);
	tree->GetEllipsoidAxes(u1, u2, u3, sig1, sig2, sig3);

	matrices::arrayToVect3(u1, v1);
	matrices::arrayToVect3(u2, v2); 
	matrices::arrayToVect3(u3, v3);

	//Getting svd
	/*Matrix3d u = tree->svd_.matrixU();
	VectorXd sing = tree->svd_.singularValues();
	DrawEllipse( sing(0), sing(1), sing(2));*/
	/*const Vector3 from( -1, 0, 2 );
	/*DrawArrow( from, u.col(0) );
	DrawArrow( from, u.col(1) );
	DrawArrow( from, u.col(2) );*/

	float rx, ry, rz, xf, yf, zf;
	rx = sig1;
	ry = sig2;
	rz = sig3;

	float sina;
	float a, b, x, y, z;

	float pas = 0.01f;
	glColor3f(0.0f, 0.0f, 1.0f);
	
	glBegin(GL_POINTS);


	//directing vectors values
	float ox, oy, oz, x0,y0,z0,x1,y1,z1,x2,y2,z2;
	x0 = v1(0); x1 = v2(0); x2 = v3(0);
	y0 = v1(1); y1 = v2(1); y2 = v3(1);
	z0 = v1(2); z1 = v2(2); z2 = v3(2);
	ox = pos(0); oy = pos(1); oz = pos(2);

	for (a=0.0f; a <= Pi; a+=pas)
	{
		z = rz*cosf(a);
		sina = sinf (a);
		for (b=-Pi; b <= Pi; b+=pas)
		{			
			  	  
			x =rx*sina * cosf(b);
		  
			y =ry*sina * sinf(b);
	  
			//normal(x, y, z);
			//rotation with correct angles
			xf = ox + x * x0 + y * x1 + z * x2;
			yf = oy + x * y0 + y * y1 + z * y2;
			zf = oz + x * z0 + y * z1 + z * z2;

			//glVertex3f(x,y,z) ;
			glVertex3f(xf,yf,zf) ;

		}
	}
	glEnd();
	
}

void DrawEllipsoid(RobotI* rob)
{
	TreeI* tree = rob->GetTreeI(3);
	pRobot->ToWorldCoordinates(robotCoord);
	matrices::array16ToMatrix4(robotCoord, currentTransform);
	tree->EndEffectorPosition(from);
	matrices::arrayToVect3(from, pos);
	pos = matrices::matrix4TimesVect3(currentTransform, pos);
	matrices::vect3ToArray(from, pos);
	tree->GetEllipsoidAxes(u1, u2, u3);

	matrices::arrayToVect3(u1, v1);
	matrices::arrayToVect3(u2, v2); 
	matrices::arrayToVect3(u3, v3);
	dsSetColor(0,0,1);
	//dsSetTexture(0);
	//dsSetColorAlpha(0,0,1,0.3);
	//glPushMatrix();
	//// translating to point center
	//glTranslatef(pos.x(), pos.y(), pos.z());
	////glRotate(angle, axis.x, axis.y, axis.z);
	//// Scaling to ellipsoid values
	//glScalef(v1.norm(), v2.norm(), v3.norm());
	//gluSphere(quad, 1, 100, 100);
	//glPopMatrix();
	v1 = pos + v1; matrices::vect3ToArray(u1, v1);
	v2 = pos + v2; matrices::vect3ToArray(u2, v2);
	v3 = pos + v3; matrices::vect3ToArray(u3, v3);
	dsDrawLineD(from, u1);
	dsDrawLineD(from, u2);
	dsDrawLineD(from, u3);

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
		state = gameState::padDrive;
		manager.GetPostureManager()->ResetTrajectory();
		drawManager.Clear();
		drawTrajectory.Clear();
		return;
	}
	float xL, yL;
	const float slowDownFactor = 1.f ; // = 0.005f;
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
	else if(controller.AnyReleased((int)xbox::Buttons::Start))
	{
		state = gameState::mouseDraw;
		robot->ToWorldCoordinates(tmp);
		manager.GetPostureManager()->AddCheckPoint(0.1f, matrices::Vector3(tmp[3], tmp[7], tmp[11]));
		manager.GetPostureManager()->Compute(robot, 10000);
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
	if (! takeInput) return;
	controller.Update();	
	if(controller.AnyReleased((int)xbox::Buttons::Back))
	{
		state = gameState::mousePlan;
		return;
	}
	float xL, yL;
	float xR, yR;
	bool transformed = false;
	const float slowDownFactor = 0.005f;
	controller.GetLeftStickMovingVector(xL, yL);
	controller.GetRightStickMovingVector(xR, yR);
	if( xL != 0 || yL != 0)
	{
		double trans[3] = {0.,-((double)xL * slowDownFactor*2), ((double)yL * slowDownFactor * 2)};
		robot->Translate(trans);
		oldDir_ = Vector3(0, -xL, yL);
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
		xyz[2] = transform(2,3) + 2.f;
		screenArrowPosition_(0) = xyz[0] + 10.f;
		screenArrowPosition_(1) = xyz[1] - 5.f;
		screenArrowPosition_(2) = xyz[2] + 2.f;
		//postureManager->NextPosture(pRobot,matrices::matrix4TimesVect3(transform, oldDir_));
		dsSetViewpoint (xyz,hpr);
	}
	if(controller.AllPressed((int)xbox::Buttons::Y))
	{
		Matrix4 robotBasis(MatrixX::Identity(4,4));
		robotBasis(0,3) = -10;
		robotBasis(1,3) = 0.75;
		robotBasis(2,3) = 1.4;
		enums::robot::eRobots type = pRobot->GetType();
		pRobot->Release();
		pRobot = manager.CreateRobot(type, robotBasis);
		delete dr;
		dr = new DrawRobot(pRobot);
	}
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
		matrices::vect3ToArray(arrowOr, screenArrowPosition_ + oldDir_ / 2.);
		dsDrawCapsule(arrowOr, idMatrix, 1, 0.05);
	}
	Matrix3 transformCube;
	GetRotationMatrix(unitz, oldDir_, transformCube);
	matrix3ToArray(idMatrix, transformCube);
}


void XboxControlWall(RobotI * robot)
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
		transformed = true;
		Rotate(pRobot, matrices::Rotx3(xR * slowDownFactor*3));

	}
	if(transformed)
	{
		matrices::Matrix4 transform;
		double transf[16];
		robot->ToRobotCoordinates(transf);
		matrices::array16ToMatrix4(transf, transform);
		//postureManager->NextPosture(pRobot,matrices::matrix4TimesVect3(transform, oldDir_));
		postureManager->NextPosture(pRobot, oldDir_);
	}
	if(controller.AllPressed((int)xbox::Buttons::Y))
	{
		
		Matrix4 robotBasis(MatrixX::Identity(4,4));
		robotBasis(0,3) = -10;
		robotBasis(1,3) = 0.75;
		robotBasis(2,3) = 1.4;
		pRobot->Release();
		pRobot = manager.CreateRobot(enums::robot::Quadruped, robotBasis);
		delete dr;
		dr = new DrawRobot(pRobot);
	}
}

static void simLoop (int pause)
{
	Draw(pause);
	XboxControlWall(pRobot);
	/*switch(state)
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
	}*/
	bool ok = true;
	do
	{
		takeInput = solver.StepClampingToTargets(pRobot, oldDir_);
	}
	while(!ok);
	if(pRobot)
	{
		/*DrawEllipsoid(pRobot);
		DrawEllipse(pRobot);*/
	}
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

//// WAL SPECIFIC
//namespace matrices
//{
//	const Vector3 unitx = Vector3(1, 0, 0);
//	const Vector3 unity = Vector3(0, 1, 0);
//	const Vector3 unitz = Vector3(0, 0, 1);
//	const Vector3 zero  = Vector3(0, 0, 0);
//}
//
//using namespace matrices;
//using namespace Eigen;
//using namespace std;
//using namespace manip_core;
//
//// main object
//
//void Translate(RobotI* robot, const matrices::Vector3 dir)
//{
//	double tab [3];
//	matrices::vect3ToArray(tab, dir);
//	robot->Translate(tab);
//}
//
//
//ManipManager manager;
//DrawManager drawManager(manager);
//PostureManager* postureManager = manager.GetPostureManager();
//DrawRobot* dr;
//const IKSolver& solver = manager.GetIkSolver();
//xbox::Controller controller(1);
//matrices::Vector3 oldDir_(1., 0., 0.);
//
//
//static float xyz[3] = {-10.0,1,3.0};
//static float hpr[3] = {0.0,0.0,0.0};
//RobotI* pRobot;


void BuildWorld()
{
	// FORWARD OBSTACLES
	Vector3 p1(-11,6,0.1f);
	Vector3 p2(10,6,0.1f);
	Vector3 p3(10,-2,0.1f);
	Vector3 p4(-11,-2,0.1f);
	manager.SetNextColor(1,0,0);
	manager.AddObstacle(p1,p2,p3,p4);

	p1 = Vector3(-7 , 3. , 4);
	p4 = Vector3(-7., -3, 0.4);
	manager.SetNextColor(1,1,0);
	manager.GenerateVerticalChess(p1, p4, 0);
	
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
//
//void Rotate(RobotI* robot, const matrices::Matrix3& rotation)
//{
//	matrices::Matrix4 transform;
//	double transf[16];
//	robot->ToWorldCoordinates(transf);
//	matrices::array16ToMatrix4(transf, transform);
//	transform.block<3,3>(0,0) = transform.block<3,3>(0,0) * rotation;
//	matrices::matrixTo16Array(transf, transform);
//	robot->SetTransform(transf);
//}

int main(int argc, char *argv[])
{
	//init robot
	Matrix4 robotBasis(MatrixX::Identity(4,4));
	robotBasis(0,3) = -10;
	robotBasis(1,3) = 0.75;
	robotBasis(2,3) = 1.4;

	BuildWorld();

	pRobot = manager.CreateRobot(enums::robot::Quadruped, robotBasis);
	dr = new DrawRobot(pRobot);
	/* TEST */
	//sg->GenerateSamples(*pRobot, 243);

	Vector3 robotBasis2(0,0,0);

	
/* trajectory */
	postureManager->SetJumpToTarget(true);
	postureManager->InitSamples(pRobot,100000);
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
