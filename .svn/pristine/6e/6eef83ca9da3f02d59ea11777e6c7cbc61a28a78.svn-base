
#include "DrawSupportPolygon.h"
#include <drawstuff/drawstuff.h> // The drawing library for ODE;
#include "kinematic/SupportPolygon.h"
#include "kinematic/Robot.h"

#ifdef WIN32
#include <windows.h>
#endif

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <GL/glui.h>

using namespace matrices;

DrawSupportPolygon::DrawSupportPolygon(const Robot& robot, const SupportPolygon& supportPolygon)
{
	transform_ = robot.ToWorldCoordinates();
	supportPolygon.Accept(this);
}

DrawSupportPolygon::~DrawSupportPolygon()
{
	// TODO
}

void DrawSupportPolygon::Visit(const matrices::Vector3& polygonPoint)
{
	points_.push_back(matrix4TimesVect3(transform_, polygonPoint));
}


void DrawSupportPolygon::Draw() const
{
	dsSetColor(1.0f, 1.0f, 1.0f);
	float Identity [12] = { 1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f };
	float pos1_[3];
	float pos2_[3];
	if( points_.size() == 1 )
	{
		vect3ToArray(pos1_, points_[0]);
		dsDrawSphere(pos1_, Identity, 0.1f);
	}
	else
	{
		int n = points_.size() -1;
		for (int i=0; i<n; i++) 
		{ 
			vect3ToArray(pos1_, points_[i]);
			vect3ToArray(pos2_, points_[i+1]);	
			dsDrawLine(pos1_, pos2_);
		}
	}
}

