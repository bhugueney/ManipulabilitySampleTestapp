#ifndef PERFORMANCETIMER_H_
#define PERFORMANCETIMER_H_

#ifdef _WIN32
#include <windows.h>
#else
#include <sys/time.h>
#endif

//#include "CoreDLL.h"

class PerformanceTimer
{
#ifdef _WIN32
	LARGE_INTEGER _tstart, _tend;
	static LARGE_INTEGER freq;
#else
	struct timeval _tstart, _tend;
	struct timezone tz;
#endif

public:
	PerformanceTimer();
	~PerformanceTimer();
	
	void tstart(void);
	void tend(void);
	double tval(); // tiempo transcurrido en segundos desde tstart a tend
	double tcurrent(); // tiempo transcurrido en segundos desde tstart
	double getTicks();
};

#endif 
