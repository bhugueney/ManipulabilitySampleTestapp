
#include "SupportPolygon.h"
#include "SupportPolygonVisitor_ABC.h"
#include "Robot.h"
#include "kinematic/Tree.h"
#include "world/Intersection.h"

#include "world/World.h"

#include <math.h>

#include<Eigen/StdVector>

using namespace matrices;
using namespace Eigen;

using namespace std;

namespace
{
	const NUMBER Epsilon = 0.1;
}

struct SupportPolygonPImpl
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	SupportPolygonPImpl(const Robot& robot)
	{
		const Robot::T_Tree& trees = robot.GetTrees();
		for(Robot::T_TreeCIT it = trees.begin(); it!= trees.end(); ++it)
		{
			if((*it)->IsLocked())
			{
				//points_.push_back((*it)->GetEffectorPosition(0));
				VectorX result(4);
				result.block(0,0,3,1) = matrix4TimesVect3(robot.ToRobotCoordinates(), (*it)->GetTarget());
				result(3) = (*it)->GetId();
				points_.push_back(result); // assumed position ahem
				avgZ += result(2);
			}
		}
		if(points_.size() > 2) // add last point only if already have a polygon
		{
			points_ = ConvexHull(points_);
			//points_.push_back(points_[0]);
			//compute ConvexHull
			//points_ = ConvexHull(points_);
		}
		if(points_.size() != 0) avgZ = avgZ / points_.size();
	}

	~SupportPolygonPImpl()
	{
		// NOTHING
	}

	// we're using VectorX; 4th value is TreeID !!
    typedef std::vector<matrices::VectorX,Eigen::aligned_allocator<matrices::VectorX> > T_Point;
	
	// isLeft(): tests if a point is Left|On|Right of an infinite line.
	//    Input:  three points P0, P1, and P2
	//    Return: >0 for P2 left of the line through P0 and P1
	//            =0 for P2 on the line
	//            <0 for P2 right of the line
	//    See: the January 2001 Algorithm "Area of 2D and 3D Triangles and Polygons"
	NUMBER isLeft( const VectorX& P0, const VectorX& P1, const VectorX& P2 )
	{
		return ( (P1.x() - P0.x()) * (P2.y() - P0.y())
				- (P2.x() - P0.x()) * (P1.y() - P0.y()) );
	}

	NUMBER isLeft( const VectorX& P0, const VectorX& P1, const Vector3& P2 )
	{
		return ( (P1.x() - P0.x()) * (P2.y() - P0.y())
				- (P2.x() - P0.x()) * (P1.y() - P0.y()) );
	}

	// source http://softsurfer.com/Archive/algorithm_0103/algorithm_0103.htm#wn_PinPolygon()
	// wn_PnPoly(): winding number test for a Vector3 in a polygon
	//      Input:   P = a Vector3,
	//               points_ = Point vector of size n+1 with points_[n]=points_[0]
	//      Return:  wn = the winding number (=0 only if P is outside points_[])
	bool InPolygon ( const T_Point& points, const Vector3& P )
	{
		int    wn = 0;    // the winding number counter
		int n = (int)(points.size()) - 1;
		if(n < 0)
		{
			return false;
		}
		else if(n == 0)
		{
			Vector2 p (P.x(), P.y());
			Vector2 p0 (points[0].x(), points[0].y());
			return ((p-p0).norm() < Epsilon);
		}
		else if(n == 1)
		{
			Vector2 p0 (points[0].x(), points[0].y());
			Vector2 p1 (points[1].x(), points[1].y());
			return (DistancePointSegment(P, p0, p1) < Epsilon);
		} 

		// loop through all edges of the polygon
		for (int i=0; i<n; i++) 
		{   // edge from points_[i] to points_[i+1]
			if (points[i].y() <= P.y()) 
			{   // start y <= P.y()
				if (points[i+1].y() > P.y())      // an upward crossing
				{
					if (isLeft( points[i], points[i+1], P) > 0)  // P left of edge
						++wn;            // have a valid up intersect
				}
			}
			else 
			{   // start y > P.y() (no test needed)
				if (points[i+1].y() <= P.y())     // a downward crossing
				{
					if (isLeft( points[i], points[i+1], P) < 0)  // P right of edge
						--wn;            // have a valid down intersect
				}
			}
		}
		return !( wn == 0 );
	}

	const VectorX& LeftMost(const T_Point& points)
	{
		unsigned int i = 0;
		for(unsigned int j = 1; j < points.size(); ++j)
		{
			if(points[j].x() < points[i].x())
			{
				i = j;
			}
		}
		return points[i];
	}

#include <iostream>
	//http://en.wikipedia.org/wiki/Gift_wrapping_algorithm
	T_Point ConvexHull(const T_Point& points)
	{
		T_Point res;
		VectorX pointOnHull = LeftMost(points);	
		VectorX endPoint;
		int i = 0;
		do
		{
			++i;
			VectorX pi = pointOnHull;
			endPoint = points[0];
			for(unsigned int j = 1; j < points.size(); ++j)
			{
				if((endPoint == pointOnHull) || (isLeft(pi, endPoint, points[j]) > 0))
				{
					endPoint = points[j];
				}
				
				if( i > 10000 )
				{
					std::cout << " WTF " << std::endl << points[j] << std::endl;
					bool gtd = false;
				}
			}
			res.push_back(pi);
			pointOnHull = endPoint;
		} while(endPoint != res[0]);
		res.push_back(endPoint);
		return res;
	}

	Vector2 Centroid(const T_Point& points) const
	{
		unsigned int n = (unsigned int)(points.size());
		Vector2 res(0,0);
		if(n>0)
		{
			if(n == 1)
			{
				res =  points[0].block(0,0,2,1);
			}
			else if(n == 2)
			{
				res = (points[0].block(0,0,2,1) + points[1].block(0,0,2,1)) / 2;
			}
			else
			{
				for(unsigned int i = 0; i < (n-1); ++i)
				{
					res = res + points[i].block(0,0,2,1);
				}
				res = res / (NUMBER)(n-1);
			}
		}
		return res;
	}

	const Tree::TREE_ID GetTreeToLift(const Vector2& externalCom, const T_Point& points, const Vector3& direction)
	{
		Vector2 centroid = Centroid(points);
		unsigned int i = 0;
		Vector2 A, B;
		Vector3 interPoint;
		Vector2 dir(direction.x(),direction.y());
		dir.normalize();
		for(unsigned int j = 1; j < points.size(); ++j)
		{
			A = points[i].block(0,0,2,1);
			B = points[j].block(0,0,2,1);
			if( points.size() == 2 || intersection_.IntersectSegments(centroid, externalCom, A, B, interPoint))
			{
				// use dot product to determine angle between direction and ACom and Bcom vectors, if one is positive
				// it s ine the direction we want to take, so we'll choose the other.
				//if both are negative.
				// return furthest point from com

				Vector2 comA(A - externalCom);
				comA.normalize();

				Vector2 comB(B - externalCom);
				comB.normalize();

				if(comA.dot(dir) > 0 || dir.dot(comA) > 0)
				{
					// return b
					return (Tree::TREE_ID) (points[j](3));
				}
				else if(comB.dot(dir) > 0 || dir.dot(comB) > 0)
				{
					return (Tree::TREE_ID) (points[i](3));
				}
				else
				{
					return (Tree::TREE_ID) (((externalCom - A).norm() > (externalCom - B).norm()) ? (points[i](3)) : (points[j](3)));
				}
			}
			++i;
		}
		return (Tree::TREE_ID) (-1);
	}

	Tree::TREE_ID GetTreeToLift(const matrices::Vector2& externalCom, const T_Point& points, const matrices::Vector3& direction, const std::vector<Tree::TREE_ID>& candidates, Tree::TREE_ID lastLifted)
	{
		Tree::TREE_ID res = GetTreeToLift(externalCom, points, direction);
		// com is not out, just grab the further away from the direction
		if(res<0)
		{
			NUMBER minCos = 1;
			Vector2 dir(direction.x(),direction.y());
			dir.normalize();
			Vector2 A;
			Tree::TREE_ID id = -1;
			for(unsigned int i = 0; i < points.size(); ++i)
			{
				A = points[i].block(0,0,2,1);
				id = (Tree::TREE_ID) (points[i](3));
				if( id != lastLifted)
				{
					if(std::find(candidates.begin(), candidates.end(), id) != candidates.end())
					{
						Vector2 comA(A - externalCom);
						comA.normalize();
						NUMBER minc = comA.dot(dir);
						if(minc <= minCos)
						{
							minCos = minc;
							res = id;
						}

					}
				}
			}
		}
		return res;
	}

	NUMBER DistancePointSegment(const Vector3& pt, const Vector2& A, const Vector2& B)
	{
		Vector2 point(pt.x(), pt.y());
		//first coefficient director
		NUMBER a = (A.y() - B.y()) / (A.x() - B.x());
		NUMBER b = B.y() - ( a * B.x());
		
		//line that goes through and orthogonal to segment
		NUMBER c = -(1/a);
		NUMBER d = point.y() - (point.x() * c);

		//now computing intersection point
		NUMBER Xint = (d - b) / (c - a);
		NUMBER Yint = a * Xint + b;

		// Xint on the segment if Aint and AB colinear and norm(Aint) < norm(AB)
		Vector2 inter(Xint, Yint);
		if( (inter-A).dot(B-A) > 0 && (inter-A).norm() < (B-A).norm() )
		{
			Vector2 newPosition(Xint, Yint);
			return (point - newPosition).norm();
		}
		else // take closest extremity
		{
			return min((point - A).norm(), (point - B).norm());
		}
	}

	void ComputeMinimalCorrection(const Vector2& externalCom, const T_Point& points, Vector3& newPosition)
	{
		Vector2 centroid = Centroid(points);
		unsigned int i = 0;
		Vector2 A, B;
		Vector3 interPoint;
		newPosition(0) = 0; newPosition(1) = 0; newPosition(2) = 0;
		if(points.size() == 1)
		{
			newPosition.block(0,0,2,1) =  points[i].block(0,0,2,1) - externalCom;
		}
		else
		{
			for(unsigned int j = 1; j < points.size(); ++j)
			{
				A = points[i].block(0,0,2,1);
				B = points[j].block(0,0,2,1);
				if(points.size() == 2 || intersection_.IntersectSegments(centroid, externalCom, A, B, interPoint))
				{
					// ok, compute directing vector of normal projection

					//first coefficient director
					NUMBER a = (A.y() - B.y()) / (A.x() - B.x());
					NUMBER b = B.y() - ( a * B.x());
					
					//line that goes through and orthogonal to segment
					NUMBER c = -(1/a);
					NUMBER d = externalCom.y() - (externalCom.x() * c);

					//now computing intersection point
					NUMBER Xint = (d - b) / (c - a);
					NUMBER Yint = a * Xint + b;

					// Xint on the segment if Aint and AB colinear and norm(Aint) < norm(AB)
					Vector2 inter(Xint, Yint);
					if( (inter-A).dot(B-A) > 0 && (inter-A).norm() < (B-A).norm() )
					{
						//newPosition(0) = Xint - externalCom(0); tmp
						newPosition(1) = Yint - externalCom(1);
					}
					else // take closest extremity
					{
						newPosition.block(0,0,2,1) = (((externalCom - A).norm() > (externalCom - B).norm()) ? (B - externalCom) : (A - externalCom));
						newPosition(0)=0;//temp
					}
				}
				++i;
			}
		}
	}

	T_Point points_;
	Intersection intersection_;
	NUMBER avgZ;
};

SupportPolygon::SupportPolygon(const Robot& robot)
	: pImpl_(new SupportPolygonPImpl(robot))
{
	//TODO
}

SupportPolygon::~SupportPolygon()
{
	// NOTHING
}
Vector3 SupportPolygon::Centroid() const
{
	Vector3 res(0,0,pImpl_->avgZ);
	Vector2 center = pImpl_->Centroid(pImpl_->points_);
	res(0) = center(0);
	res(1) = center(1);
	return res;
}


// TODO handle z in reorientation !
Vector3 SupportPolygon::ReplaceCom(const matrices::Vector3& com) const
{
	Vector2 centroid = pImpl_->Centroid(pImpl_->points_);
	return Vector3 (centroid.x() - com.x(), centroid.y() - com.y(), 0);
}


void SupportPolygon::Accept(SupportPolygonVisitor_ABC* visitor) const
{
	for(SupportPolygonPImpl::T_Point::const_iterator it=pImpl_->points_.begin(); it != pImpl_->points_.end(); ++it)
	{
		Vector3 point = (*it).block(0,0,3,1);
		visitor->Visit(point);
	}
}


bool SupportPolygon::Contains(const matrices::Vector3& aPoint) const
{
	return pImpl_->InPolygon(pImpl_->points_, aPoint);
}

bool SupportPolygon::Contains(const matrices::Vector3& aPoint, matrices::Vector3& correction) const
{
	if(pImpl_->InPolygon(pImpl_->points_, aPoint))
	{
		return true;
	}
	else
	{
		matrices::Vector2 comOut = aPoint.block(0,0,2,1);
		pImpl_->ComputeMinimalCorrection(comOut, pImpl_->points_, correction);
		// TEMP JUST computing vector to centroid...
		// TODO find minimal distance
		/*Vector3 centroid(0,0,0);
		centroid.block(0,0,2,1) = pImpl_->Centroid(pImpl_->points_);
		correction = centroid - aPoint;
		correction(2) = 0;*/
		return false;
	}
}

bool SupportPolygon::WouldContain(const matrices::Vector3& aPoint, const matrices::Vector3& polygonPoint)
{
	/*SupportPolygonPImpl::T_Point points = pImpl_->points_;
	if (points.size() > 0 )
	{
		if (points.size() > 2)
		{
			points.pop_back();
		}
		points.push_back(polygonPoint);
		points = pImpl_->ConvexHull(points);
	}
	return pImpl_->InPolygon(points, aPoint);*/
	return true;
}


bool SupportPolygon::WouldContain(const matrices::Vector3& aPoint, const Robot& robot, const Tree& tree)
{
	// TODO
	/*SupportPolygonPImpl::T_Point points = pImpl_->points_;
	if (points.size() > 0 )
	{
		if (points.size() > 2)
		{
			points.pop_back();
		}
		points.push_back(tree.GetPosition());
		points = pImpl_->ConvexHull(points);
	}
	return pImpl_->InPolygon(points, aPoint);*/
	return true;
}

Tree::TREE_ID SupportPolygon::ComputeTreeToLift(const matrices::Vector3& com, const Vector3& direction)
{
	matrices::Vector2 comOut = com.block(0,0,2,1);
	return pImpl_->GetTreeToLift(comOut, pImpl_->points_, direction);
}

Tree::TREE_ID SupportPolygon::ComputeTreeToLift(const matrices::Vector3& com, const matrices::Vector3& direction, const std::vector<Tree::TREE_ID>& candidates, Tree::TREE_ID lastLifted)
{
	matrices::Vector2 comOut = com.block(0,0,2,1);
	return pImpl_->GetTreeToLift(comOut, pImpl_->points_, direction, candidates, lastLifted);
}
