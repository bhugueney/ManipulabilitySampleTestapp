
#include "MatrixDefs.h"

#ifndef _PI
#define _PI

const NUMBER Pi = NUMBER(std::acos(-1.0));

const NUMBER RadiansToDegrees = NUMBER(180.0/Pi);
const NUMBER DegreesToRadians = NUMBER(Pi/180);

#define RADIAN(X)	((X)*DegreesToRadians)

static NUMBER GetAngle(NUMBER cosinus, NUMBER sinus)
{	
	int sign = (sinus > 0) ? 1 : -1;
	return sign * acos(cosinus);
}

#endif // _PI
