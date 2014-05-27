#include <iostream>
#include "sampling\SampleGenerator.h"
#include "sampling\SampleGeneratorVisitor_ABC.h"
#include "skeleton\Robot.h"
#include "skeleton\Joint.h"
#include "WorldManager.h"
#include "RobotManager.h"
#include "tools\MatrixDefs.h"
#include "world\Obstacle.h"
#include "Posture\PostureManager.h"
#include "sampling\filters\FilterDistance.h"

// Include standard headers
#include <stdio.h>
#include <stdlib.h>
#include <GL/glut.h>
#include <vector>

using namespace std;
using namespace manip_core;
using namespace matrices;

float zoom = 1.0f;
float rot_x = 0;
float rot_y = 0.001f;
float rot_z = 0.001f;
float move_x = 0;
float move_y = 0;
int last_x = 0;
int last_y = 0;

bool bMouse_Left, bMouse_Right, bMouse_Middle;

Quads * quads_;
Robot * robot_;
PostureManager * postureManager_;

// SamplegeneratorTestApp.cpp : Entry point for the console application.
//

// Setup variables
static class Painter{
public:
	Painter();
	~Painter();

public:
	static void DrawRobot(Robot * robot){
		const Tree * tree = robot->GetTorso();

		Matrix4 m4 = robot->ToWorldCoordinates();

		// Torso
		Joint * joint = tree->GetRoot();

		while (joint){
			Painter::DrawJoint(joint, m4, 0.15);
			joint = joint->pChild_;
		}

		// Rest of the body
		std::vector<Tree*> trees = robot->GetTrees();
		for (int i = 0; i < 2; i++){
			Joint * joint = trees[i]->GetRoot();
			while (joint){
				Painter::DrawJoint2(joint, m4);
				joint = joint->pChild_;
			}
		}

		for (int i = 2; i < 4; i++){
			Joint * joint = trees[i]->GetRoot();
			while (joint){
				Painter::DrawJoint(joint, m4);
				joint = joint->pChild_;
			}

		}
	}
	static void DrawJoint(Joint *joint){
		const Vector3 local = joint->ComputeS();
		Eigen::Vector4d v4d = Eigen::Vector4d(local.x(), local.y(), local.z(), 1.0);

		Painter::DrawSphere(v4d, 0.1);
	}

	static void DrawJoint(Joint *joint, Matrix4 &m4){
		const Vector3 local = joint->GetS();
		Eigen::Vector4d v4d = Eigen::Vector4d(local.x(), local.y(), local.z(), 1.0);
		v4d = m4 * v4d;

		Painter::DrawOrangeSphere(v4d, 0.1);
	}

	static void DrawJoint2(Joint *joint, Matrix4 &m4){
		const Vector3 local = joint->GetS();
		Eigen::Vector4d v4d = Eigen::Vector4d(local.x(), local.y(), local.z(), 1.0);
		v4d = m4 * v4d;

		Painter::DrawGreenSphere(v4d, 0.1);
	}


	static void DrawJoint(Joint *joint, Matrix4 &m4, float radius){
		const Vector3 local = joint->GetS();
		Eigen::Vector4d v4d = Eigen::Vector4d(local.x(), local.y(), local.z(), 1.0);
		v4d = m4 * v4d;

		Painter::DrawSphere(v4d, radius);
	}

	static void DrawSphere(Eigen::Vector4d &v, float radius){
		glColor3d(0.5647, 0.6549, 0.8314);
		glPushMatrix();
			glTranslated(v.x(), v.y(), v.z());
			glutSolidSphere(radius, 50, 50);
		glPopMatrix();
	}

	static void DrawOrangeSphere(Eigen::Vector4d &v, float radius){
		glColor3d(0.9412, 0.3882, 0.0471);
		glPushMatrix();
		glTranslated(v.x(), v.y(), v.z());
		glutSolidSphere(radius, 50, 50);
		glPopMatrix();
	}

	static void DrawGreenSphere(Eigen::Vector4d &v, float radius){
		glColor3d(0.0784, 0.9412, 0.0471);
		glPushMatrix();
		glTranslated(v.x(), v.y(), v.z());
		glutSolidSphere(radius, 50, 50);
		glPopMatrix();
	}
	static void DrawQuads(Quads *quads){
		Quads q = *quads;
		for (int i = 0; i < q.size(); i++){
			Painter::DrawQuad(q[i]);
		}
	}

	static void DrawQuad(Obstacle *obstacle){
		DrawQuad(obstacle->GetP1(), obstacle->GetP2(), obstacle->GetP3(), obstacle->GetP4());
	}

	static void DrawQuad(Vector3 p1, Vector3 p2, Vector3 p3, Vector3 p4){

		GLfloat vertices[] = { p1.x(), p1.y(), p1.z(), //bottom left corner
							   p2.x(), p2.y(), p2.z(), //top left corner
							   p3.x(), p3.y(), p3.z(), //top right corner
							   p4.x(), p4.y(), p4.z() }; // bottom right rocner

		GLubyte colors[] =
		{
			247, 206, 109,
			247, 206, 109,
			247, 206, 109,
			247, 206, 109
		};

		GLubyte indices[] = { 0, 1, 2, // first triangle (bottom left - top left - top right)
			0, 2, 3 }; // second triangle (bottom left - top right - bottom right)

		glEnableClientState(GL_COLOR_ARRAY);
		glEnableClientState(GL_VERTEX_ARRAY);
		glColorPointer(3, GL_UNSIGNED_BYTE, 0, colors);
		glVertexPointer(3, GL_FLOAT, 0, vertices);
		glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_BYTE, indices);
	}
};


/* GLUT callback Handlers */
static void resize(int width, int height)
{
	const float ar = (float)width / (float)height;

	glViewport(0, 0, width, height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glFrustum(-ar, ar, -1.0, 1.0, 2.0, 100.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void MouseMove(int x, int y)
{
	int diff_x = x - last_x;
	int diff_y = y - last_y;
	last_x = x;
	last_y = y;

	if (bMouse_Left)
	{
		rot_x += (float)0.5f * diff_y;
		rot_z += (float)0.5f * diff_x;
	}
	else if (bMouse_Right)
	{
		move_x += (float)0.05f * diff_x;
		move_y -= (float)0.05f * diff_y;
	}
	else if (bMouse_Middle)
	{
		zoom -= (float)0.05f * diff_x;
	}

	glutPostRedisplay();
}

void MouseButton(int button, int state, int x, int y)
{
	last_x = x;
	last_y = y;

	switch (button)
	{
	case GLUT_LEFT_BUTTON:
		if (state == GLUT_DOWN)
		{
			bMouse_Left = true;
		}
		else
		{
			bMouse_Left = false;
		}
		break;

	case GLUT_RIGHT_BUTTON:
		if (state == GLUT_DOWN)
		{
			bMouse_Right = true;
		}
		else
		{
			bMouse_Right = false;
		}
		break;

	case GLUT_MIDDLE_BUTTON:
		if (state == GLUT_DOWN)
		{
			bMouse_Middle = true;
		}
		else
		{
			bMouse_Middle = false;
		}
		break;
	}

	glutPostRedisplay();
}

static void display(void)
{
	glClearColor(1.0, 1.0, 1.0, 0.0);
	glClear(GL_COLOR_BUFFER_BIT);
	glColor3f(1.0, 1.0, 1.0);
	glLoadIdentity();

	gluLookAt(0.0, 0.0, 5.0,
		0.0, 0.0, 0.0,
		0.0, 1.0, 0.0);

	glPushMatrix();
	glTranslatef(0, 0, -zoom);
	glTranslatef(move_x, move_y, 0);
	glRotatef(rot_x, 1, 0, 0);
	glRotatef(rot_y, 0, 1, 0);
	glRotatef(rot_z, 0, 0, 1);

	Painter::DrawQuads(quads_);
	Painter::DrawRobot(robot_);
	
	glPopMatrix();
	glutSwapBuffers();
}


static void key(unsigned char key, int x, int y)
{
	switch (key)
	{
	case 27:
	case 'q':
		exit(0);
		break;
	}
	glutPostRedisplay();
}

static void idle(void)
{

	glutPostRedisplay();
}

int main(int argc, char *argv[])
{
	cout << "Initializing...";
	// Builds All elements in the world including obstacles and robot
	Manager::WorldManager* wManager = new Manager::WorldManager();
	World * world = wManager->initWorld();
	Manager::RobotManager* rManager = new Manager::RobotManager();
	Robot * robot = rManager->generateRobotInstance();

	Quads quads = wManager->getQuads();

	// Transfer to buffer for drawing
	quads_ = &quads;
	robot_ = robot;

	// Start Offline sampling
	cout << "Generating Samples...";
	SampleGenerator * sGenerator = SampleGenerator::GetInstance();
	sGenerator->GenerateSamples(*robot, 1);

	// Start Online sampling
	postureManager_ = new PostureManager(world);
	//postureManager_->AddPostureCriteria(enums::postureCriteria::toeOnCOM);
	postureManager_->SetJumpToTarget(true);

	Vector3 robotBasis2(0, 0, 1.8);

	/* trajectory */
	robotBasis2(0) = -9;
	robotBasis2(2) = 1.8;
	postureManager_->AddCheckPoint(1.f, robotBasis2);

	//FilterDistance * filterDistance = new FilterDistance(1,);
	//sGenerator->Request(robot, t)
	//postureManager_->Compute(robot_, 1000);
	// Start glut
	glutInit(&argc, argv);
	glutInitWindowSize(640, 480);
	glutInitWindowPosition(10, 10);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);

	glutCreateWindow("Sample Generator Test App");

	glutReshapeFunc(resize);
	glutDisplayFunc(display);
	glutMouseFunc(MouseButton);
	glutMotionFunc(MouseMove);

	glutKeyboardFunc(key);
	glutIdleFunc(idle);

	glClearColor(0, 0, 0, 0);

	glutMainLoop();
	return 0;
}

