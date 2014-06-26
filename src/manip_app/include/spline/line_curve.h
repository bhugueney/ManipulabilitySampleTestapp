/**
* \file line_curve.h
* \brief class allowing to create a trajectory made of straight lines
* \author Steve T.
* \version 0.1
* \date 06/17/2013
*/


#ifndef _CLASS_LINECURVE
#define _CLASS_LINECURVE

#include "curve_abc.h"
#include "cubic_function.h"

#include "MathDefs.h"

#include <functional>
#include <vector>

namespace spline
{
/// \class line_curve
/// \brief Represents a set of cubic splines defining a continuous function 
/// crossing each of the waypoint given in its initialization
///
template<typename Time= double, typename Numeric=Time, int Dim=3, bool Safe=false
, typename Point= Eigen::Matrix<Numeric, Dim, 1> >
struct line_curve : public curve_abc<Time, Numeric, Dim, Safe, Point>
{
	typedef Point 	point_t;
	typedef Time 	time_t;
	typedef Numeric	num_t;
	typedef cubic_function<time_t, Numeric, Dim, Safe, Point> cubic_function_t;
	typedef typename std::vector<cubic_function_t*> T_cubic;
	typedef typename T_cubic::iterator IT_cubic;
	typedef typename T_cubic::const_iterator CIT_cubic;

	/* Constructors - destructors */
	public:
	///\brief Constructor
	///\param wayPointsBegin : an iterator pointing to the first element of a waypoint container
	///\param wayPointsEns   : an iterator pointing to the end           of a waypoint container
	template<typename In>
	line_curve(In wayPointsBegin, In wayPointsEnd)
	{
		std::size_t const size(std::distance(wayPointsBegin, wayPointsEnd));
		if(Safe && size < 1)
		{
			throw; // TODO
		}

		point_t zero = Point::Zero();
		In it(wayPointsBegin), next(wayPointsBegin);
		++next;
		it= wayPointsBegin, next=wayPointsBegin; ++ next;
		for(int i=0; next != wayPointsEnd; ++i, ++it, ++next)
		{
			//x(t) = a + b(t - t_min_) + c(t - t_min_)^2 + d(t - t_min_)^3
			point_t a,b;

			subSplines_.push_back(new cubic_function_t((*it).second, -((*it).second - (*next).second)/((*next).first - (*it).first), zero, zero, (*it).first, (*next).first));
		}
		subSplines_.push_back(new cubic_function_t((*it).second, zero, zero, zero, (*it).first, (*it).first));
	}

	///\brief Destructor
	~line_curve()
	{
		for(IT_cubic it = subSplines_.begin(); it != subSplines_.end(); ++ it)
		{
			delete(*it);
		}
	}

	private:
	line_curve(const line_curve&);
	line_curve& operator=(const line_curve&);
	/* Constructors - destructors */

	/*Operations*/
	public:
	///  \brief Evaluation of the cubic spline at time t.
	///  \param t : the time when to evaluate the spine
	///  \param return : the value x(t)
	virtual point_t operator()(time_t t) const
	{
		point_t p;
    	if(Safe && (t < subSplines_.front()->t_min_ || t > subSplines_.back()->t_max_)){throw std::out_of_range("TODO");}
		for(CIT_cubic it = subSplines_.begin(); it != subSplines_.end(); ++ it)
		{
			if(t >= ((*it)->t_min_) && t <= ((*it)->t_max_))
			{
				return (*it)->operator()(t);
			}
		}
		return p;
	}
	/*Operations*/

	/*Helpers*/
	public:
	num_t virtual min() const{return subSplines_.front()->t_min_;}
	num_t virtual max() const{return subSplines_.back()->t_max_;}
	/*Helpers*/

	/*Attributes*/
	private:
	T_cubic subSplines_;
	/*Attributes*/
};
}
#endif //_CLASS_LINECURVE

