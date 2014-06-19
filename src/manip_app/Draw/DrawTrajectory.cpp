
#include "DrawTrajectory.h"
#include <drawstuff/drawstuff.h> // The drawing library for ODE;

using namespace std;
using namespace matrices;


DrawTrajectory::DrawTrajectory()
{
	// NOTHING
}


DrawTrajectory::~DrawTrajectory()
{
	// NOTHING
}

void DrawTrajectory::AddPoint(const matrices::Vector3& point)
{
	points_.push_back(point);
}

void DrawTrajectory::AddPoint(const float* point)
{
	matrices::Vector3 mPoint;
	matrices::arrayToVect3(point, mPoint);
	points_.push_back(mPoint);
}

void DrawTrajectory::Draw() const
{
	float id[12]; matrices::matrixID(id);
	dsSetColor(0.0,0.0,1.0);
	for(vector<Vector3>::const_iterator it = points_.begin(); it != points_.end(); ++it)
	{
		float draw[3];
		matrices::vect3ToArray(draw, *it);
		dsDrawSphere(draw, id, 0.1f);
	}
	//drawPostures_.Draw();
}

void DrawTrajectory::Clear()
{
	points_.clear();
}


