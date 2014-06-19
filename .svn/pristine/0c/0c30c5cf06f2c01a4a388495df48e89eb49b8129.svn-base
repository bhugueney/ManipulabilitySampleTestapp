/**
* \file exact_bezier.h
* \brief class allowing to create an exact Bezier curve of dimension 1 <= n <= 3.
* By exact we mean that the curve goes through the given indicated waypoints
* \author Steve T.
* \version 0.1
* \date 06/17/2013
*/


#ifndef _CLASS_EXACTBEZIERCURVE
#define _CLASS_EXACTBEZIERCURVE

#include "bezier_curve.h"

namespace spline
{

template<typename In, typename Point, typename Numeric, int Dim>
inline std::vector<Point> interpolate(In PointsBegin, In PointsEnd, size_t size)
{
	typedef Eigen::Matrix<Numeric, 2, 2> Matrix2;
	typedef Eigen::Matrix<Numeric, 2, Eigen::Dynamic> Matrix23;

	std::vector<Point> res;
	In it(PointsBegin);
	Point p0, p1, p2;
	p0 = *it; ++it;
	p1 = *it; ++it;
	p2 = *it; ++it;

	res.push_back(p0);
	if(size == 3) // TODO ABOVE
	{
		const double u = 0.5;
		res.push_back((p1 - p2 * u*u - p0 * (1-u)*(1-u))/(2 * u * (1 - u)));
		res.push_back(p2);
	}
	else if(size >= 4) // TODO ABOVE
	{
		Point p3 = *it;
		const double u = 0.33; const double v = 0.67;

		double a=0.0, b=0.0, c=0.0, d=0.0, det=0.0;
		Point q1, q2;

		a = 3*(1-u)*(1-u)*u; b = 3*(1-u)*u*u;
		c = 3*(1-v)*(1-v)*v; d = 3*(1-v)*v*v;
		det = a*d - b*c;
		/* unnecessary, but just in case... */
		if (det == 0.0) throw; /* failure */
	
		q1 = p1 - ((1-u)*(1-u)*(1-u)*p0 + u*u*u*p3);
		q2 = p2 - ((1-v)*(1-v)*(1-v)*p0 + v*v*v*p3);

		res.push_back((d*q1 - b*q2)/det);
		res.push_back(((-c)*q1 + a*q2)/det);
		res.push_back(p3);
	}
	return res;
}

	template<typename Time, typename Numeric, int Dim, bool Safe, typename Point, typename In >
	inline bezier_curve<Time, Numeric, Dim, Safe, Point>* exact_bezier(In PointsBegin, In PointsEnd, const Time minBound=0, const Time maxBound=1)
	{
		std::vector<Point> res;
		size_t size = std::distance(PointsBegin, PointsEnd);
		switch(size)
		{
			case 0:
			case 1:
				throw;
			case 2:
				return new bezier_curve<Time, Numeric, Dim, Safe, Point>(PointsBegin, PointsEnd, minBound, maxBound);
			default:
				res = interpolate<In, Point, Numeric, Dim>(PointsBegin, PointsEnd, size);
				return new bezier_curve<Time, Numeric, Dim, Safe, Point>(res.begin(), res.end(), minBound, maxBound);
		}
	}

}
#endif //_CLASS_EXACTBEZIERCURVE
