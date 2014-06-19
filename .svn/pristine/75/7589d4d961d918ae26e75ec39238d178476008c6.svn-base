#include "DrawSpline.h"
#include <drawstuff/drawstuff.h> // The drawing library for ODE;

using namespace matrices;

DrawSpline::DrawSpline(const spline::curve_abc<>& spline)
{
	matrices::Vector3 val;
	for(NUMBER t = 0; t < spline.max(); t = t +0.01)
	{
		points_.push_back(spline(t));
	}
}

DrawSpline::~DrawSpline()
{
	// NOTHING
}

void DrawSpline::Draw() const
{
	float id[12];
	float pos[3];
	matrices::matrixID(id);
	dsSetColor(1.0f, 0.0f, 0.0f);
	for(CIT_Vector3 it = points_.begin(); it!=points_.end(); ++it)
	{
		matrices::vect3ToArray(pos,*it);
		dsDrawSphere(pos,id,0.05f);
	}
}

