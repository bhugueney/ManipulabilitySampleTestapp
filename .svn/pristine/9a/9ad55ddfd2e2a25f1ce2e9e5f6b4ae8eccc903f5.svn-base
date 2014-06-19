

#include "MatrixDefs.h"
#include "Pi.h"

#include "ManipManager.h"
#include "PostureManager.h"
#include "draw/DrawManager.h"
#include "API/RobotI.h"
#include "API/TreeI.h"

#include <drawstuff/drawstuff.h> // The drawing library for ODE;

#ifdef WIN32
#include <windows.h>
#endif

#include <GL/gl.h>
#include <GL/glu.h>

#include <iostream>
#include <vector>


using namespace matrices;
using namespace Eigen;
using namespace std;
using namespace manip_core;

// main object

ManipManager manager;
DrawManager drawManager(manager);
static float xyz[3] = {-10.0,1,3.0};
static float hpr[3] = {0.0,0.0,0.0};

GLUquadric* quad = gluNewQuadric();

RobotI* pRobot(0);
double robotCoord[16];
Matrix4 currentTransform;
Vector3 pos, v1, v2, v3;
double from[3];
double u1[3];
double u2[3];
double u3[3];
double sig1, sig2, sig3;

void BuildWorld()
{

	float decal = -0.75;
	// FORWARD OBSTACLES
	Vector3 p1(-11,2 + decal,0.4f);
	Vector3 p2(-5.5,2 + decal,0.4f);
	Vector3 p3(-5.5,-1 + decal,0.4f);
	Vector3 p4(-11,-1 + decal,0.4f);

	manager.SetNextColor(0,0,0);
	manager.AddObstacle(p1,p2,p3,p4);
	
	manager.Initialize(false);
}
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

static void simLoop (int pause)
{
	drawManager.Draw();
	if(pRobot)
	{
		DrawEllipsoid(pRobot);
		DrawEllipse(pRobot);
	}
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

	pRobot = manager.CreateRobot(enums::robot::Quadruped, robotBasis);
	
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


/* trajectory */
	postureManager->Compute(pRobot,100);
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
