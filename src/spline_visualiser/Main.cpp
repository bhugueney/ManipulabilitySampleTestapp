
#include <drawstuff/drawstuff.h> // The drawing library for ODE;

#ifdef WIN32
#include <windows.h>
#endif

#include "spline/cubic_function.h"
#include "spline/exact_cubic.h"
#include "spline/bezier_curve.h"
#include "MatrixDefs.h"

#include <string>
#include <iostream>
#include <cmath>

/*Cubic Visitor tests*/
#include <vector>

#include<Eigen/StdVector>

namespace spline
{
	typedef std::vector<matrices::Vector3,Eigen::aligned_allocator<matrices:Vector3>> T_Vector;
	typedef T_Vector::const_iterator CIT_Vector;
	//struct SplineVisitorTest : public SplineVisitor
	//{
	//	SplineVisitorTest()
	//		: previousTime_(-1)
	//	{
	//		// NOTHING
	//	}

	//	~SplineVisitorTest()
	//	{
	//		// NOTHING
	//	}

	//	virtual void Visit(const Real time, const Vector3& value)
	//	{
	//		if(previousTime_ >= time)
	//		{
	//			std::cout << "Error : Visit method called with non sequential time values" << std::endl;
	//		}
	//		previousTime_ = time;
	//		values_.push_back(value);
	//	}

	//	Real previousTime_;
	//	T_Vector values_;
	//};
}


using namespace spline;

//SplineVisitorTest visitor;
//
//static float xyz[3] = {-10.0,1,5.0};
//static float hpr[3] = {0.0,0.0,0.0};
//static float O[3] = {0.0,0.0,0.0};
//static float X[3] = {5.0,0.0,0.0};
//static float Y[3] = {0.0,5.0,0.0};
//static float Z[3] = {0.0,0.0,5.0};
//
//
//static void simLoop (int pause)
//{
//	float id[12];
//	float pos[3];
//	matrices::matrixID(id);
//	dsSetColor(1.0f, 0.0f, 0.0f);
//	for(CIT_Vector it = visitor.values_.begin(); it!=visitor.values_.end(); ++it)
//	{
//		matrices::vect3ToArray(pos,*it);
//		dsDrawSphere(pos,id,0.1);
//	}
//	// rouge
//	dsDrawLine(O,X);
//
//	float pos1[3] = {-7.5,1.39,2.73};
//	dsDrawSphere(pos1,id,0.3);
//	// vert
//	dsSetColor(0.0f, 1.0f, 0.0f);
//
//
//	float pos2[3] = {-7.5,1.1,3.04};
//	dsDrawSphere(pos2,id,0.3);
//
//	dsDrawLine(O,Y);
//	//bleu
//	dsSetColor(0.0f, 0.0f, 1.0f);
//	dsDrawLine(O,Z);
//
//	float pos3[3] = {-7.4,0.52,3};
//	dsDrawSphere(pos3,id,0.3);
//
//}
//
//void start()
//{
//    dsSetViewpoint (xyz,hpr);
//	spline::T_Waypoint waypoints;
//	/*waypoints.push_back(std::make_pair(0,Vector3(-7.5,1.39,2.73)));
//	waypoints.push_back(std::make_pair(0.43,Vector3(-7.5,1.1,3.04)));
//	waypoints.push_back(std::make_pair(1.2,Vector3(-7.4,0.52,3)));
//	waypoints.push_back(std::make_pair(1.022,Vector3(-7.4,0.52,3)));
//	ExactCubic cubic(waypoints);
//	cubic.Accept(visitor, 0.01);*/
//
//	T_Vector  points;
//	points.push_back(Vector3(-7.5,1.39,2.73));
//	points.push_back(Vector3(-7.5,1.1,3.04));
//	points.push_back(Vector3(-7.5,0.6,3.04));
//	points.push_back(Vector3(-7.4,0.52,3));
//	BezierCurve cubic(points);
//	cubic.Accept(visitor, 0.01);
//	Vector3 wtf;
//	cubic.Evaluate(0, wtf);
//	std::cout << wtf << std::endl << std::endl;
//	cubic.Evaluate(10, wtf);
//	std::cout << wtf << std::endl << std::endl;
//	bool tg = cubic.Evaluate(15, wtf);
//	std::cout << wtf << std::endl << std::endl;
//	cubic.Evaluate(20, wtf);
//	std::cout << wtf << std::endl;
//}
//
//void command(int cmd)   /**  key control function; */
//{
//	switch (cmd)
//	{	
//		case '+' :
//		break;
//	}
//}


int main(int argc, char *argv[])
{
		/*
	drawstuff stuff*/
	/*dsFunctions fn;
    fn.version = DS_VERSION;
    fn.start   = &start;
    fn.step    = &simLoop;
	fn.command = &command;
    fn.stop    = 0;
    fn.path_to_textures = "../textures";
    dsSimulationLoop (argc,argv,800,600,&fn);*/

    return 0;
}
