

#include "MatrixDefs.h"
#include "Pi.h"

#include "ExactCubic.h"
#include "draw/DrawSpline.h"

#include "Timer.h"

#include "ManipManager.h"
#include "PostureManager.h"
#include "draw/DrawRobot.h"
#include "draw/DrawManager.h"
#include "draw/DrawTrajectory.h"
#include "API/TreeI.h"
#include "API/RobotI.h"
#include "IKSolver/IKSolver.h"
#include "API/IkConstraintHandlerI.h"

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

Timer timer;

unsigned long lastTime = 0;

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

spline::ExactCubic* sspline(0);
DrawSpline* drSpline(0);

//mouse control stuff
float arrowOr[3] = {1.0, 1.0, 1.0};

float idMatrix[12];
float idMat[12];
double tmp[16];

static float xyz[3] = {-10.0,1,3.50};
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
	Vector3 p1(-7.5,6,3.f);
	Vector3 p2(10,6,3.f);
	Vector3 p3(10,-2,3.f);
	Vector3 p4(-7.5,-2,3.f);

	manager.AddObstacle(p1,p2,p3,p4);

	p1 = Vector3(50,200,10);
	p2 = Vector3(50,-200,10.f);
	p3 = Vector3(50,-200,0.f);
	p4 = Vector3(50,200,0.f);

	manager.SetNextColor(1,0.,0);

	PosePrise( Vector3( -7.f, 0 , 4));
	PosePrise( Vector3( -7.1f, 0. , 3.8));
	PosePrise( Vector3( -7.1f, 0.5, 3.5));
	PosePrise( Vector3( -7.1f, -0.5, 3.5));
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
		state = gameState::mouseDraw;
		robot->ToWorldCoordinates(tmp);
		manager.GetPostureManager()->AddCheckPoint(0.1f, matrices::Vector3(tmp[3], tmp[7], tmp[11]));
		manager.GetPostureManager()->ComputeOnline(robot, 10000);		
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
		xyz[2] = transform(2,3) + 2.f;/*
		screenArrowPosition_(0) = xyz[0] + 10.f;
		screenArrowPosition_(1) = xyz[1] - 5.f;
		screenArrowPosition_(2) = xyz[2] + 2.f;*/
		screenArrowPosition_(0) = transform(0,3) - 2.f;
		screenArrowPosition_(1) = transform(1,3) + 2.f;
		screenArrowPosition_(2) = transform(2,3) + 1.f;
		//postureManager->NextPosture(pRobot,matrices::matrix4TimesVect3(transform, oldDir_));
		//dsSetViewpoint (xyz,hpr);
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
	Matrix3 transformCube;
	GetRotationMatrix(unitz, oldDir_, transformCube);
	matrix3ToArray(idMatrix, transformCube);
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

	TreeI* tree = rob->GetTreeI(0);
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

	float pas = 0.05f;
	glColor3f(0.3f, 0.4f, 0.7f);
	
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
	TreeI* tree = rob->GetTreeI(0);
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
	//glColor3f(0.3f, 0.4f, 0.7f);
	dsSetColor(	0.3f, 0.4f, 0.7f);
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


static void simLoop (int pause)
{
	lastTime = timer.GetTime();
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
	bool ok = true;
	do
	{
		takeInput = solver.StepClampingToTargets(pRobot, oldDir_);
	}
	while(!ok);
	DrawEllipsoid(pRobot);
		DrawEllipse(pRobot);
	if(drSpline)
	{
		drSpline->Draw();
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
	const Vector3 trZ(0,0.0,  0.03);
	const Vector3 myZ(0.3, 0.0,  0.07);
	spline::ExactCubic* nspline;
	switch (cmd)
	{	
		case '+' :
			oldDir_(0) = oldDir_(0) + 1.;
			oldDir_.normalize();
			Translate(pRobot, trZ);
			nspline = postureManager->NextPosture(pRobot, myZ)[0].second;
			if(nspline)
			{
				delete (sspline);
				delete (drSpline);
				sspline = nspline;
				drSpline = new DrawSpline(*sspline);
			}
		break;
		case '-' :
			Translate(pRobot, trX);
			nspline = postureManager->NextPosture(pRobot,matrices::unitx)[0].second;
			//drawManager.PreviousPosture();
		break;
		case 'c' :
			solver.constraints_->AddConstraint(manip_core::enums::IKConstraints::ObstacleManip);
		break;
		case 'v' :
			solver.constraints_->RemoveConstraint(manip_core::enums::IKConstraints::ObstacleManip);
		break;
		case 'a' :
			//Rotate(pRobot, matrices::Roty3(-0.1));
			//postureManager->NextPosture(pRobot,matrices::unity);
			////drawManager.PreviousPosture();
			Matrix4 robotBasis(MatrixX::Identity(4,4));
			robotBasis(0,3) = -8;
			robotBasis(1,3) = 1.;
			robotBasis(2,3) = 2.;
			enums::robot::eRobots type = pRobot->GetType();
			pRobot->Release();
			pRobot = manager.CreateRobot(type, robotBasis);
			delete dr;
			dr = new DrawRobot(pRobot);
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
	robotBasis(0,3) = -8;
	robotBasis(1,3) = 1.;
	robotBasis(2,3) = 3.;

	BuildWorld();

	pRobot = manager.CreateRobot(enums::robot::HumanEllipse, robotBasis);
	dr = new DrawRobot(pRobot);
	/* TEST */
	//sg->GenerateSamples(*pRobot, 243);

	Vector3 robotBasis2(0,0,0);

	
/* trajectory */
	postureManager->SetJumpToTarget(false);
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
    fn.path_to_textures = "../textures";
	//glutMainLoop();
	timer.Start();

    dsSimulationLoop (argc,argv,800,600,&fn);

    return 0;
}