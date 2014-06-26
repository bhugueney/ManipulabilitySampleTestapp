
#ifndef _CLASS_DRAWSPLINE
#define _CLASS_DRAWSPLINE

#include "spline/exact_cubic.h"

#include "MatrixDefs.h"

class DrawSpline
{
public:
    typedef std::vector<matrices::Vector3,Eigen::aligned_allocator<matrices::Vector3> > T_Vector3;
	typedef T_Vector3::const_iterator CIT_Vector3;

public:
	 DrawSpline(const spline::curve_abc<>& spline);
	~DrawSpline();

public:
	 void Draw() const;

public: 


private:
	T_Vector3 points_;

}; // class DrawSpline

#endif //_CLASS_DRAWSPLINE
