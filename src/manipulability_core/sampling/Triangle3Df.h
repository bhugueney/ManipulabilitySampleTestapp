#ifndef _CLASS_TRIANGLE3DF
#define _CLASS_TRIANGLE3DF

#include "MatrixDefs.h"

using namespace matrices;

namespace manip_core
{

	class Triangle3Df
	{
	public:
		Triangle3Df(const Vector3& a, const Vector3& b, const Vector3& c)
		: a_(a), b_(b), c_(c){}

		~Triangle3Df(){}

		const Vector3 a_;
		const Vector3 b_;
		const Vector3 c_;
	};
}
#endif