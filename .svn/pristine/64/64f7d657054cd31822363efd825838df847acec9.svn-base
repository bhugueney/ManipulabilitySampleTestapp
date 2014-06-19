#include "kinematic/Tree.h"
#include "sampling/Sample.h"
#include "sampling/SampleGenerator.h"
#include "kinematic/IKSolver.h"
#include "kinematic/Jacobian.h"

#include "SimpleSampleVisitor.h"

#include "world/Obstacle.h"
#include "world/Intersection.h"
#include "DrawObstacle.h"



#include "MatrixDefs.h"
#include "Pi.h"

#include <drawstuff/drawstuff.h> // The drawing library for ODE;

#ifdef WIN32
#include <windows.h>
#endif

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <GL/glui.h>

#include <iostream>
#include <vector>

using namespace std;

#define RADIAN(X)	((X)*DegreesToRadians)

using namespace matrices;
using namespace Eigen;

Tree* test; // TO F... REMOVE

Vector3 Target(0,0,0);

SimpleSampleVisitor visitor;
SampleGenerator* generator = SampleGenerator::GetInstance();

Intersection intersection;
IKSolver iKSolver;

bool activateIK = false;

Obstacle* obstacles[4]; // stupid visual bug
DrawObstacle::T_DrawObstacle drawObstacles;

//static void ViewOrtho(int x, int y)							// Set Up An Ortho View
//{
//	glMatrixMode(GL_PROJECTION);					// Select Projection
//	glPushMatrix();							// Push The Matrix
//	glLoadIdentity();						// Reset The Matrix
//	glOrtho(0, x , y , 0, -40, 40);				// Select Ortho Mode
//	glMatrixMode(GL_MODELVIEW);					// Select Modelview Matrix
//	glPushMatrix();							// Push The Matrix
//	glLoadIdentity();						// Reset The Matrix
//}
//
//static void ViewPerspective(void)							// Set Up A Perspective View
//{
//	glMatrixMode(GL_PROJECTION);					// Select Projection
//	glPopMatrix();							// Pop The Matrix
//	glMatrixMode(GL_MODELVIEW);					// Select Modelview
//	glPopMatrix();							// Pop The Matrix
//}

void GetOGLPos(int x, int y, Vector3& res)
{
    GLint viewport[4];
    GLdouble modelview[16];
    GLdouble projection[16];
    GLfloat winX, winY, winZ;
    GLdouble posX, posY, posZ;
 
    glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
    glGetDoublev(GL_PROJECTION_MATRIX, projection);
    glGetIntegerv(GL_VIEWPORT, viewport);
 
    winX = (float)x;
    winY = (float)viewport[3] - (float)y;
    glReadPixels(x, int(winY), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ);
 
    gluUnProject(winX, winY, winZ, modelview, projection, viewport, &posX, &posY, &posZ);
 
	res.x() = posX;
	res.y() = posY;
	res.z() = posZ;
	//return CVector3(posX, posY, posZ);
}

static bool handleDirection(int cmd)
{
	bool found = false;
	double delta = 0.05;
	Vector3* dir = test->directionForce_;
	switch (cmd)
		{
		case 'o':
			found = true;
			test->directionForce_->z() = test->directionForce_->z() + delta;
			break;
		case 'l':
			test->directionForce_->z() = test->directionForce_->z() - delta;
			found = true;
			break;
		case 'm':
			test->directionForce_->x() = test->directionForce_->x() + delta;
			found = true;
			break;
		case 'k':
			test->directionForce_->x() = test->directionForce_->x() - delta;
			found = true;
			break;
		}
		if(found) 
		{
			test->directionForce_->normalize();
		}
	return found;
}

void command(int cmd)   /**  key control function; */
{
	Sample * res;
	VectorX vect(5);
	vect << 1, 1, 1, 1 , 1;
	VectorX res1 = test->Identitymin * (vect * -1) * 0.01;
	VectorX res2 = test->Identitymin * vect  * 0.01;
	VectorX res3 = VectorX(test->Identitymin.cols());
	if (! handleDirection(cmd))
	{
		Joint* joint(0);
		int sens = 1;
		switch (cmd)
		{
		case 'i' :
			activateIK = false;
			test->GetJoint(1)->AddToTheta(res1(0));
			test->GetJoint(2)->AddToTheta(res1(1));
			test->GetJoint(3)->AddToTheta(res1(2));
			test->GetJoint(4)->AddToTheta(res1(3));
			test->GetJoint(5)->AddToTheta(res1(4));
		break;
		case 'p' :
			/*test->GetJoint(1)->AddToTheta(res2.x()*0.01);
			test->GetJoint(2)->AddToTheta(res2.y()*0.01);
			test->GetJoint(3)->AddToTheta(res2.z()*0.01);*/		
			for(int i = 0; i < 1 ; ++i)
			{
			iKSolver.PartialForceManDerivatives(*test, *test->directionForce_, res3);
			res3 = test->Identitymin * res3;
			res3 *= 100;
			test->GetJoint(1)->AddToTheta(res3(0));
			test->GetJoint(2)->AddToTheta(res3(1));
			test->GetJoint(3)->AddToTheta(res3(2));
			test->GetJoint(4)->AddToTheta(res3(3));
			test->GetJoint(5)->AddToTheta(res3(4));
			test->Compute();
			test->ComputeJacobian();
			}
		break;
		case 'w' :
			test->ToRest();
			break;
		case 'x':
			{	//force
				visitor.Configure(1.f, 0.f);
				intersection.Intersect(*test, *(obstacles[0]), Target);
				// SAMPLE APPROACH
				/*res = visitor.Run(*generator, *(test->directionForce_), *(test->directionVel_), Target);
				if(res)
				{
					res->LoadIntoTree(*test);
				}*/
				//generator->Accept(visitor);
				// IK APPROACH
				activateIK = true;
				break;
			}
		case 'c':
			test->OptimizeCurrentConfiguration(); 
			break;
		case 'v':
			//vitesse
			{
				visitor.Configure(0.f, 1.f);
				intersection.Intersect(*test, *(obstacles[2]), Target);
				// SAMPLE APPROACH
				/*res = visitor.Run(*generator, *(test->directionForce_), *(test->directionVel_), Target);
				if(res)
				{
					res->LoadIntoTree(*test);
				}*/
				//generator->Accept(visitor);
				// IK APPROACH
				activateIK = true;
				break;
			}
			case 'n':
			//controle force, grande vitesse
			{
				visitor.Configure(-1.f, 1.f);
				res = visitor.Run(*generator, *(test->directionForce_), *(test->directionVel_), Target);
				if(res)
				{
					res->LoadIntoTree(*test);
				}
				//generator->Accept(visitor);
				break;
			}
		case 'b':
			test->ApproximateVelocityManipulabilityBruteForce(); 
			break;
		case 'a':
			joint = test->GetJoint(1); // increases THETA[1] when a key is pressed
			break;
		case 'z':
			joint = test->GetJoint(2);
			break;
		case 'e':
			joint = test->GetJoint(3);
			break;
		case 'r':
			joint = test->GetJoint(4);
			break;
		case 't':
			joint = test->GetJoint(5);
			break;
		case 'q':
			sens = -1;
			joint = test->GetJoint(1); // increases THETA[1] when a key is pressed
			break;
		case 's':
			sens = -1;
			joint = test->GetJoint(2);
			break;
			sens = -1;
		case 'd':
			sens = -1;
			joint = test->GetJoint(3);
			break;
		case 'f':
			sens = -1;
			joint = test->GetJoint(4);
			break;
		case 'g':
			sens = -1;
			joint = test->GetJoint(5);
			break;
		}
		if (joint)
		{
			joint->AddToTheta(0.05 * sens);
		}
	}
}

void BuildObstacles()
{
	Vector3 p1(-1,1,0.1f);
	Vector3 p2(1,1,0.1f);
	Vector3 p3(1,0,0.1f);
	Vector3 p4(-1,0,0.1f);

	Obstacle* obs = new Obstacle(p1,p2,p3,p4); 
	obstacles[0] = obs;
	DrawObstacle drawObs(*obs);
	drawObstacles.push_back(drawObs);

	float pose = -4.f;
	float posey = 1.f;
	Vector3 p11(pose,posey+ 1,0.5f);
	Vector3 p12(pose+1,posey+ 1,0.5f);
	Vector3 p13(pose+1,posey,0.1f);
	Vector3 p14(pose,posey,0.1f);

	Obstacle* obs1 = new Obstacle(p11,p12,p13,p14); 
	obstacles[1] = obs1;
	DrawObstacle drawObs1(*obs1);
	drawObstacles.push_back(drawObs1);

	pose = 0.2f;
	posey = 1.f;
	Vector3 p21(pose,posey+ 1,0.4f);
	Vector3 p22(pose+1,posey+ 1,0.4f);
	Vector3 p23(pose+1,posey,1.f);
	Vector3 p24(pose,posey,1.f);

	Obstacle* obs2 = new Obstacle(p21,p22,p23,p24); 
	obstacles[2] = obs2;
	DrawObstacle drawObs2(*obs2);
	drawObstacles.push_back(drawObs2);
}

void BuildTree(Joint *node[], Tree &tree)
{
	const Vector3 unitx(1, 0, 0);
	const Vector3 unity(0, 1, 0);
	const Vector3 unitz(0, 0, 1);
	const Vector3 unit1(sqrt(14.0)/8.0, 1.0/8.0, 7.0/8.0);
	const Vector3 zero(0,0,0);

	/*VERTICAL ARM*/
	
	node[0] = new Joint(Vector3(0.0, 0.0, 1.7), unity,JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(0), Y);
	tree.InsertRoot(node[0]);

	node[1] = new Joint(Vector3(0.0, 0.0, 1.7), unitx,JOINT, RADIAN(-135), RADIAN(135), RADIAN(0), X);
	tree.InsertChild(node[0], node[1]);

	node[2] = new Joint(Vector3(0.0, 0.0, 1.7), unitz,JOINT, RADIAN(-45), RADIAN(45), RADIAN(0), Z);
	tree.InsertChild(node[1], node[2]);

	node[3] = new Joint(Vector3(0.0, 0.0, 0.9), unity,JOINT, RADIAN(-90), RADIAN(45), RADIAN(-30), Y);
	tree.InsertChild(node[2], node[3]);

	node[4] = new Joint(Vector3(0.0, 0.0, 0.9), unitx,JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(0.), X);
	tree.InsertChild(node[3], node[4]);

	node[5] = new Joint(Vector3(0.0, 0.0, 0.0), zero,EFFECTOR);
	tree.InsertChild(node[4], node[5]);

	/*HORIZONTAL ARM*/
	/*
	node[0] = new Joint(Vector3(0.0, -0.5, 0.0), unity,JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.), Y);
	tree.InsertRoot(node[0]);

	node[1] = new Joint(Vector3(0.0, -0.5, 0.0), unitx,JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.), X);
	tree.InsertChild(node[0], node[1]);

	node[2] = new Joint(Vector3(0.0, -0.5, 0.0), unitz,JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.), Z);
	tree.InsertChild(node[1], node[2]);

	node[3] = new Joint(Vector3(0.0, 0.4, 0.0), unitz,JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.));
	tree.InsertChild(node[2], node[3]);

	node[4] = new Joint(Vector3(0.0, 0.4, 0.0), unitx,JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.), X);
	tree.InsertChild(node[3], node[4]);

	node[5] = new Joint(Vector3(0.0, 1.2, 0.0), zero,EFFECTOR);
	tree.InsertChild(node[4], node[5]);*/
	
	
	test = &tree; // to remove
}

void BuildTree2D(Joint *node[], Tree &tree)
{
	const Vector3 unitx(1, 0, 0);
	const Vector3 unity(0, 1, 0);
	const Vector3 unitz(0, 0, 1);
	const Vector3 unit1(sqrt(14.0)/8.0, 1.0/8.0, 7.0/8.0);
	const Vector3 zero(0,0,0);

	/*VERTICAL ARM*/
	
	node[0] = new Joint(Vector3(0.0, 0.0, 2.0), unity,JOINT, RADIAN(-45.), RADIAN(225.), RADIAN(0.), Y);
	tree.InsertRoot(node[0]);

	node[1] = new Joint(Vector3(0.0, 0.0, 3.0), unity,JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(0.), Y);
	tree.InsertChild(node[0], node[1]);

	node[2] = new Joint(Vector3(0.0, 0.0, 4.0), unity,JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(0.), Y);
	tree.InsertChild(node[1], node[2]);

	node[3] = new Joint(Vector3(0, 0.0, 4.5), zero,EFFECTOR);
	tree.InsertChild(node[2], node[3]);

	/*HORIZONTAL ARM*/
	/*
	node[0] = new Joint(Vector3(0.0, -0.5, 0.0), unity,JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.), Y);
	tree.InsertRoot(node[0]);

	node[1] = new Joint(Vector3(0.0, -0.5, 0.0), unitx,JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.), X);
	tree.InsertChild(node[0], node[1]);

	node[2] = new Joint(Vector3(0.0, -0.5, 0.0), unitz,JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.), Z);
	tree.InsertChild(node[1], node[2]);

	node[3] = new Joint(Vector3(0.0, 0.4, 0.0), unitz,JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.));
	tree.InsertChild(node[2], node[3]);

	node[4] = new Joint(Vector3(0.0, 0.4, 0.0), unitx,JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.), X);
	tree.InsertChild(node[3], node[4]);

	node[5] = new Joint(Vector3(0.0, 1.2, 0.0), zero,EFFECTOR);
	tree.InsertChild(node[4], node[5]);*/
	
	
	test = &tree; // to remove
}

/**
Drawstuff stuff : 
*/

static void DrawJoint(Joint* joint, Matrix4& currentTransform)
{
	float length = 1.6f;
	//joint->AddToTheta(Pi / 240.);
	
if (joint->IsEffector())
{
	//joint->AddToTheta(Pi / 2400.);
}

	//joint->AddToTheta(Pi / 24000.);

	
	
	//draw line between positions
	float from[3];
	vect4ToArray(from, currentTransform.col(3));

	Matrix4 jointTransform = Translate(joint->GetR()); // translation of current joint
	switch(joint->GetRotation())
	{
		case X:
			jointTransform = jointTransform + Rotx4(joint->GetTheta());
			break;
		case Y:
			jointTransform = jointTransform + Roty4(joint->GetTheta());
			break;
		case Z:
			jointTransform = jointTransform + Rotz4(joint->GetTheta());
			break;
	}
	//currentTransform = currentTransform * (Rotz4(joint->GetTheta()) + Translate(joint->GetR()));
	currentTransform = currentTransform * jointTransform;
	float R[12];
	matrixToArray(R, currentTransform);
    dsSetColor(1.0,0.0,0.0);
	float ps[3];


	vect4ToArray(ps, currentTransform.col(3));
	//= {currentTransform(0,3), currentTransform(1,3), currentTransform(2,3)};
	//double ps[3] = {0,0,0};
	if(joint->pChild_)
	{
		dsDrawSphere (ps, R, 0.05);
	}
	else
	{
		float sides[3] = {0.05, 0.05, 0.05};
		dsDrawBox(ps, R, sides);
	}

	//draw line between positions
	if(joint->pRealparent_)
		dsDrawLine(from, ps);

 //   dsSetColor(0.0,1.0,0.0);
	//Vector3 pos = joint->GetS();
	//for(int j =0; j< 3; ++j)
	//{
	//	ps[ j ] = float(pos(j));
	//}
	//ps[ 2 ] = ps[ 2 ] + 0.5f;
	//dsDrawSphere (ps, R, 0.05);

	////draw line between positions
	//from[ 2 ] = from[ 2 ] + 0.5f;
	//if(joint->pRealparent_)
	//	dsDrawLine(from, ps);
}

static void DrawArrow(const Vector3& from, const Vector3& dir)
{
	/*Vector3 dest; GetOGLPos(from.x(), from.y(), dest);
	Vector3 destdir = dest + dir;*/
	
	//glDisable(GL_LIGHTING);
	//glColor3f(1.0f, 1.0f, 0.0f);
	//glLineWidth(4.0);
	//glBegin(GL_LINES);
	//glVertex3f(dest.x(), dest.y(), 1); //dest.z());
	//glVertex3f(destdir.x(), destdir.y(), destdir.z());
	//glEnd();
	//glLineWidth(1.0);
	//glEnable(GL_LIGHTING);

	float Identity [12] = { 1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f };
	
	Vector3 destdir = from+dir;
	float fr[3];
	float t[3];
	vect3ToArray(fr, from); 
	vect3ToArray(t, destdir); 
	dsDrawLine  (fr, t);
	dsDrawSphere(t, Identity, 0.02);


	//ViewOrtho(200, 200);
	//glDisable(GL_LIGHTING);
	//glColor3f(1.0f, 1.0f, 0.0f);
	//glLineWidth(2.0);
	//glBegin(GL_LINES);
	//glVertex3f(from.x(), from.y(), from.z());
	//glVertex3f(to.x(), to.y(), to.z());
	//glEnd();
	//glLineWidth(1.0);
	//glEnable(GL_LIGHTING);
	//
	//ViewPerspective();
/*	glDisable(GL_LIGHTING);
	glColor3f(1.0f, 1.0f, 0.0f);
	glLineWidth(2.0);
	glBegin(GL_LINES);
	glVertex3f(from.x(), from.y(), from.z());
	glVertex3f(to.x(), to.y(), to.z());
	glEnd();
	glLineWidth(1.0);
	glEnable(GL_LIGHTING);*/
}

static void DrawObstacles()
{
	float pos_[3];
	float R_[12];
	pos_[0] = 0.f;
	pos_[1] = 0.f;
	pos_[2] = 1.7f;
	matrixID(R_);
	Vector3 intersectionPoint(0,0,0);
	//dsDrawSphere(pos_, R_, test->GetBoundaryRadius());
	for(int i = 0; i < (int)drawObstacles.size(); ++i)
	{
		if(intersection.Intersect(*test, *(obstacles[i]), intersectionPoint))
		{
			pos_[0] = intersectionPoint.x();
			pos_[1] = intersectionPoint.y();
			pos_[2] = intersectionPoint.z();
			dsDrawSphere(pos_, R_, 0.04f);
			drawObstacles[i].DrawRed();
		}
		else
		{
			drawObstacles[i].Draw();
		}
	}
}

static void DrawTree(Tree* tree)
{
	//joint->AddToTheta(Pi / 240.);
	Matrix4 m =  Matrix4::Identity();
	//cout << m << endl;
	Joint* joint = tree->GetRoot();
	//joint->AddToTheta(Pi / 2400.);
	while (joint) {
		DrawJoint(joint, m);
		joint = (joint->IsEffector() ? 0 : tree->GetSuccessor(joint));
		/*if (joint)
		{
			joint->AddToTheta(Pi / 240.);
		}*/
	}
	
	//tree->ComputeJacobian();
	Vector3 haut (0.0, 0.0, -1.0);
	Vector3 droite (1.0, 0.0, 0.0);
	std::cout << " FORCE MANIPULABILITY "	 << tree->ComputeForceManipulability(*(tree->directionForce_)) << std::endl << std::endl ;
	std::cout << " VELOCITY MANIPULABILITY " << tree->ComputeVelocityManipulability(droite) << std::endl << std::endl << std::endl << std::endl;
	std::cout << " MANIPULABILITY " << tree->ComputeManipulability() << std::endl << std::endl << std::endl << std::endl;
}

void CheckJacobian()
{
	MatrixX m(2,3);
	double l1, l2, l3, a1, a2, a3, c1, c12, c123, s1, s12, s123;

	l1 = (test->GetJoint(2)->GetR()).norm();
	l2 = (test->GetJoint(3)->GetR()).norm();
	Joint * j = test->GetJoint(3)->pChild_;;
	l3 = (j->GetR()).norm();

	a1 = (test->GetJoint(1)->GetTheta());
	a2 = (test->GetJoint(2)->GetTheta());
	a3 = (test->GetJoint(3)->GetTheta());
	//a3 = (j->GetTheta());

	c1 = cos(a1); s1 = sin(a1);
	c12 = cos(a1+ a2); s12 = sin(a1+a2);
	c123 = cos(a1+ a2 + a3); s123 = sin(a1+a2+a3);

	m(0,0)= l1*c1+l2*c12+l3*c123 ; m(0,1)= l2*c12+l3*c123 ; m(0,2)= l3 * c123;
	m(1,0)= -l1*s1 - l2*s12 -l3 * s123; m(1,1)=-l2*s12 - l3 * s123; m(1,2)= - l3*s123;
}

static void simLoop (int pause)
{
	const Vector3 arrowFrom(0.5, 0, 0.5);
	DrawTree(test);
	dsSetColor(1.0, 1.0, 0.0);
	DrawArrow(arrowFrom, *(test->directionForce_));
    dsSetColor(1.0, 1.0, 1.0);
	DrawArrow(arrowFrom, *(test->directionVel_));
	//CheckJacobian();
	DrawObstacles();

	/*test obstacles */
	Intersection intersection;
	Vector3 endeffPos = test->GetEffectorPosition(test->GetNumEffector()-1);
	if(activateIK)
	{
		activateIK = !(iKSolver.StepForceManipulability(*test, Target, *(test->directionForce_)));
		//activateIK = !(iKSolver.StepForceAndAvoidManipulability(*test, Target, *(test->directionForce_)));
		//activateIK = !(iKSolver.Step(*test, Target));
		VectorX velocities = VectorX::Zero(5);
		//Jacobian j(*test);
		//activateIK = !(iKSolver.StepClamping(*test, j, Target, velocities));
		//test->Compute();
		//test->ComputeJacobian();
		/*if((endeffPos - test->GetEffectorPosition(test->GetNumEffector()-1)).norm() < 0.05)
		{
			activateIK = false;
		}
		activateIK = true;*/
	}
	//while(activateIK)
	//{
 //	//	activateIK = !(iKSolver.StepForceManipulability(*test, Target, *(test->directionForce_)));
	//	activateIK = !(iKSolver.Step(*test, Target));
	//	
	//	if(activateIK)
	//	{
	//		test->Compute();
	//		test->ComputeJacobian();
	//		if((endeffPos - test->GetEffectorPosition(test->GetNumEffector()-1)).norm() < 0.01)
	//		{
	//			activateIK = false;
	//		}
	//		else
	//		{
	//			endeffPos = test->GetEffectorPosition(test->GetNumEffector()-1);
	//		}
	//	}
	//}
		test->Compute();
		test->ComputeJacobian();

}

void start()
{
    static float xyz[3] = {0.0,-3.0,1.0};
    static float hpr[3] = {90.0,0.0,0.0};
    dsSetViewpoint (xyz,hpr);
}



int main(int argc, char *argv[])
{
	Tree treeY;
	//Jacobian *jacobY;
	Joint* joints[4];
	BuildTree(joints,treeY);
	BuildObstacles();
	treeY.Init();
	treeY.Compute();
	treeY.directionForce_ = new Vector3(0,0,1);
	treeY.directionVel_ = new Vector3(1,0,0);

	treeY.GetJoint(1)->SetTheta(RADIAN(160));
	treeY.GetJoint(2)->SetTheta(RADIAN(-90));
	treeY.GetJoint(3)->SetTheta(RADIAN(90));

	/*treeY.GetJoint(1)->SetTheta(RADIAN(100));
	treeY.GetJoint(2)->SetTheta(RADIAN(25));
	treeY.GetJoint(3)->SetTheta(RADIAN(105));*/

	/*treeY.GetJoint(1)->SetTheta(RADIAN(180));
	treeY.GetJoint(2)->SetTheta(RADIAN(-90));
	treeY.GetJoint(3)->SetTheta(RADIAN(10));*/


	visitor.Configure(0.f, 1.f);
				
	//SAMPLE TESTING
	//generator->GenerateSamples(*test, 10000);

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
