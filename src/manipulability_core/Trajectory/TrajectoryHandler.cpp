
#include "TrajectoryHandler.h"

#include "kinematic/Robot.h"
#include "kinematic/Tree.h"
#include "world/Obstacle.h"
#include "world/ObstacleVisitor_ABC.h"
#include "world/Intersection.h"

#include "spline/exact_cubic.h"
#include "spline/exact_bezier.h"
#include "spline/line_curve.h"


using namespace matrices;

struct ReachableObstacles : public ObstacleVisitor_ABC
{
	ReachableObstacles(const World& world, const Tree& tree, const Robot& robot)
		: ObstacleVisitor_ABC()
		, world_(world)
		, tree_ (tree)
		, robot_(robot)
	{
		world.Accept(*this);
	}

	~ReachableObstacles()
	{
		// NOTHING
	}

	virtual void Visit(const Obstacle& obstacle)
	{
		if(world_.IsReachable(robot_, tree_, obstacle))
			obstacles_.push_back(&obstacle);
	}

	typedef std::vector<const Obstacle*>	T_Obstacles;
	typedef T_Obstacles::const_iterator		T_ObstaclesCIT;
	typedef T_Obstacles::iterator			T_ObstaclesIT;

	T_Obstacles obstacles_;
	const World& world_;
	const Tree&  tree_ ;
	const Robot& robot_;
};

TrajectoryHandler::TrajectoryHandler(const World& world)
	: world_(world)
{
	// TODO
}

TrajectoryHandler::~TrajectoryHandler()
{
	// NOTHING
}

//template<typename In>
//bool Log(In wayPointsBegin, In wayPointsEnd, In newwayPointsBegin, In newwayPointsEnd)
//{
//	if(std::distance(wayPointsBegin, wayPointsEnd) > 2)
//	{
//		std::stringstream f;
//		In it(wayPointsBegin);
//		f << "original waypoints : \n";
//		while(it != wayPointsEnd )
//		{
//			f << " \n waypoint: \n";
//			f << it->second << "\n";
//			++it;
//		}
//		In it2(newwayPointsBegin);
//		f << "optimized waypoints : \n";
//		while(it2 != newwayPointsEnd )
//		{
//			f << " \n waypoint: \n";
//			f << it2->second << "\n";
//			++it2;
//		}
//
//		std::ofstream myfile;
//		myfile.open ("log.txt", std::ofstream::out | std::ofstream::app);
//		if (myfile.is_open())
//		{
//			myfile << f.rdbuf();
//			myfile.close();
//			return true;
//		}
//	}
//	return false;
//}


#include <list>


namespace
{

	typedef std::list<Vector3, Eigen::aligned_allocator<Vector3>>	T_Waypoint;
	typedef T_Waypoint::const_iterator								CIT_Waypoint;
	typedef T_Waypoint::iterator									IT_Waypoint;


	// given a segment we are sure to be intersecting one of the obstacle's borders, compute the intersection point
	bool IntersectObstacleSegment(const Intersection& intersection, const Obstacle& obstacle, const Vector3& a, const Vector3& b, const Vector3& from, const Vector3& to, Vector3& midPoint)
	{
		Vector3 res(0,0,0);
		Vector3 aproj, bproj, fromproj, toproj;
		Vector2 aproj2, bproj2, fromproj2, toproj2;;
		matrices::Matrix4 obsBasis = obstacle.BasisInv();
		aproj = matrices::matrix4TimesVect3(obsBasis, a);
		bproj = matrices::matrix4TimesVect3(obsBasis, b);
		fromproj = matrices::matrix4TimesVect3(obsBasis, from);
		toproj = matrices::matrix4TimesVect3(obsBasis, to);
		aproj2 = Vector2(aproj(0), aproj(1));
		bproj2 = Vector2(bproj(0), bproj(1));
		fromproj2 = Vector2(fromproj(0), fromproj(1));
		toproj2 = Vector2(toproj(0), toproj(1));

		double h = obstacle.GetH();
		double w = obstacle.GetW();
		Vector2 A(0,h);
		Vector2 B(w,h);
		Vector2 C(w,0);
		Vector2 D(0,0);

		Vector2 goodBorderA = A;
		Vector2 goodBorderB = B;
		if(!intersection.IntersectSegments(aproj2, bproj2, A, B, res))
		{
			goodBorderB = D;
			if(!intersection.IntersectSegments(aproj2, bproj2, A, D, res))
			{
				goodBorderA = B;
				goodBorderB = C;
				if(!intersection.IntersectSegments(aproj2, bproj2, B, C, res))
				{
					goodBorderA = C;
					goodBorderB = D;
					intersection.IntersectSegments(aproj2, bproj2, C, D, res);
				}
			}
		}
		intersection.IntersectSegments(fromproj2, toproj2, goodBorderA, goodBorderB, res);
		if(res == Vector3(0,0,0))
		{
			res = to - from / 2;
			return false;
		}
		midPoint = matrices::matrix4TimesVect3(obstacle.Basis(), res);
		return true;
	}

	 bool ComputeNewWayPoint(const matrices::Matrix4& mat, const Intersection& intersection, const Tree& current, const Tree& estimated,
		 const Obstacle& obstacle, const Obstacle& plane, const Vector3& treePos, const Vector3& from, const Vector3& to, const Vector3& intersectionPoint, Vector3& midPoint)
	{
		// If there is a collision, it necesseraly means that one of the trees intersects with the plan.
		// Either the current or the targeted.
		// Assuming that, we consider this point to compute the closest point in the obstacle
		// and this will be our next waypoint ( with a small offset of course )
		if(!intersection.IntersectPlane(current, plane, midPoint))
		{
			if(!intersection.IntersectPlane(estimated, plane, midPoint))
			{
				assert (false);
			}
		}
		//TODO
		// now intersect the segment formed by the tree intersection and the trajectory intersection, with the obstacles
		// this gives us a point on the border. Move it "up" by a small delta along the obstacle normal.
		if (IntersectObstacleSegment(intersection, obstacle, intersectionPoint, matrices::matrix4TimesVect3(mat, midPoint), from, to, midPoint))
		{
			Vector3 norm = obstacle.n_;
			norm.normalize();
			midPoint = midPoint + norm * 0.6;
			return true;
		}
		// temp
		return false;
	}

	void ComputeCollisionFreePath(
		const matrices::Matrix4& mat, const Tree& current, const Tree& estimated,
		T_Waypoint& waypoints, IT_Waypoint it1, IT_Waypoint it2,
		const Obstacle& obstacle, const Obstacle& plane, const Vector3& treePos)
	{
		Intersection intersection;
		Vector3 intersectionPoint;
		if(intersection.Intersect(*it1, *it2, obstacle, intersectionPoint))
		{
			Vector3 midPoint;
			if(ComputeNewWayPoint(mat, intersection, current, estimated, obstacle, plane, treePos, *it1, *it2, intersectionPoint, midPoint))
			{
				IT_Waypoint itMid = waypoints.insert(it2, midPoint);
			}
			/*ComputeCollisionFreePath(mat, current, estimated, waypoints, it1, itMid, obstacle, plane, treePos);
			  ComputeCollisionFreePath(mat, current, estimated, waypoints, itMid, it2, obstacle, plane, treePos);*/
		}
	}

	spline::curve_abc<>* MakeSpline(const spline::SplineOptimizer<NUMBER>& optimizer, T_Waypoint& waypoints)
	{
// CUBIC SPLINE
		// timeline based on length(1)  
		/*std::vector<std::pair<double, Vector3> > splinePoints;
		Vector3& previous(*(waypoints.begin()));
		float t = 0;
		previous = *(waypoints.begin());
		for(CIT_Waypoint it = waypoints.begin(); it!= waypoints.end(); ++it)
		{
			t += (float)(*it - previous).norm();
			previous = *it;
			splinePoints.push_back(std::make_pair(t,*it));
		}
		return new spline::exact_cubic<>(splinePoints.begin(), splinePoints.end());*/
//
		
// Line curve
		// timeline based on length(1)  
		/*std::vector<std::pair<double, Vector3> > splinePoints;
		Vector3& previous(*(waypoints.begin()));
		float t = 0;
		previous = *(waypoints.begin());
		for(CIT_Waypoint it = waypoints.begin(); it!= waypoints.end(); ++it)
		{
			t += (float)(*it - previous).norm();
			previous = *it;
			splinePoints.push_back(std::make_pair(t,*it));
		}
		return new spline::line_curve<>(splinePoints.begin(), splinePoints.end());*/
//

// BEZIER CURVE
		// timeline based on length(1)  
		std::vector<Vector3> splinePoints;
		Vector3& previous(*(waypoints.begin()));
		NUMBER t = 0;
		for(CIT_Waypoint it = waypoints.begin(); it!= waypoints.end(); ++it)
		{
			t += (*it - previous).norm();
			previous = *it;
			splinePoints.push_back(*it);
		}
		return spline::exact_bezier<double, double, 3, false, Eigen::Matrix<double, 3, 1> >(splinePoints.begin(), splinePoints.end(), 0, t);
//
// OPTIMIZED STUFF
		// timeline based on length(1)  
		/*std::vector<std::pair<double, Vector3> > splinePoints;
		Vector3& previous(*(waypoints.begin()));
		float t = 0;
		previous = *(waypoints.begin());
		for(CIT_Waypoint it = waypoints.begin(); it!= waypoints.end(); ++it)
		{
			t += (float)(*it - previous).norm();
			previous = *it;
			splinePoints.push_back(std::make_pair(t,*it));
		}
		return optimizer.GenerateOptimizedCurve(splinePoints.begin(), splinePoints.end());*/
	}
}

spline::curve_abc<>* TrajectoryHandler::ComputeTrajectory(const Robot& robot, const Tree& current, const Tree& estimated, const Vector3& target)
{
	// Real world target Position
	Vector3 from = matrices::matrix4TimesVect3(robot.ToWorldCoordinates(), current.GetEffectorPosition(current.GetNumEffector()-1));
	// Real world tree Position
	Vector3 treePos = matrices::matrix4TimesVect3(robot.ToWorldCoordinates(), current.GetPosition());
	ReachableObstacles reachableObstacles(world_, current, robot);
	T_Waypoint waypoints;
	IT_Waypoint it1 = waypoints.insert(waypoints.end(), from);
	Vector3 midPoint = from + (target - from) / 2;
	midPoint(2) += 0.2;
	waypoints.insert(waypoints.end(), midPoint);
	IT_Waypoint it2 = waypoints.insert(waypoints.end(), target);
	//for(ReachableObstacles::T_ObstaclesCIT it = reachableObstacles.obstacles_.begin(); it != reachableObstacles.obstacles_.end(); ++it)
	//{
	//	matrices::Matrix4 mat = robot.ToRobotCoordinates();
	//	// create obstacle with good coordinates
	//	Obstacle obs(	matrices::matrix4TimesVect3(mat, (*it)->GetP1()),
	//					matrices::matrix4TimesVect3(mat, (*it)->GetP2()),
	//					matrices::matrix4TimesVect3(mat, (*it)->GetP3()),
	//					matrices::matrix4TimesVect3(mat, (*it)->GetP4()));
	//	ComputeCollisionFreePath(robot.ToWorldCoordinates(), current, estimated, waypoints, it1, it2, **it, obs, treePos);
	//}
	return MakeSpline(optimizer_, waypoints);
}
