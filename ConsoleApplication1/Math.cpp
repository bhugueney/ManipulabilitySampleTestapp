#include "Math.h"
#include "tools\MatrixDefs.h"
using namespace matrices;

void localmath::vect3ToArray(double f[3], const Vector3 v){
	for (int i = 0; i< 3; ++i)
	{
		f[i] = (double)v(i);
	}
}