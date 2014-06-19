
#ifndef _CLASS_DRAWSUPPORTPOLYGON
#define _CLASS_DRAWSUPPORTPOLYGON

#include "MatrixDefs.h"
#include <Eigen/StdVector>
#include "SupportPolygonVisitor_ABC.h"

class SupportPolygon;
class Robot;

class DrawSupportPolygon : public  SupportPolygonVisitor_ABC 
{
public:
	typedef std::vector<matrices::Vector3,Eigen::aligned_allocator<matrices::Vector3>> T_Point;
	typedef T_Point::const_iterator T_PointIT;

public:
	 DrawSupportPolygon(const Robot& /*robot*/, const SupportPolygon& /*supportPolygon*/);
	~DrawSupportPolygon();

	 virtual void Visit(const matrices::Vector3& /*polygonPoint*/);	

	 void Draw()const;

private:
	T_Point points_;
	matrices::Matrix4 transform_;

}; // class DrawSupportPolygon

#endif //_CLASS_DRAWSUPPORTPOLYGON