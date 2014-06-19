#include "PerformanceTimer.h"

#ifdef _WIN32
LARGE_INTEGER PerformanceTimer::freq;
#endif

PerformanceTimer::PerformanceTimer()
{
#ifdef _WIN32
	QueryPerformanceFrequency(&freq);
#endif
}

PerformanceTimer::~PerformanceTimer()
{
}

double PerformanceTimer::getTicks()
{
#ifdef _WIN32
	LARGE_INTEGER _tickValue;
	QueryPerformanceCounter(&_tickValue);
	return ((double)_tickValue.QuadPart)/((double)freq.QuadPart);
#else
	struct timeval _tickValue;
	struct timezone tz;
	gettimeofday(&_tickValue, &tz);
        return (double)_tickValue.tv_sec + (double)_tickValue.tv_usec/(1000*1000);
#endif
}

double PerformanceTimer::tcurrent()
{
#ifdef _WIN32
	LARGE_INTEGER _tickValue;
	QueryPerformanceCounter(&_tickValue);

	return ((double)_tickValue.QuadPart -
		(double)_tstart.QuadPart)/((double)freq.QuadPart);
#else
	struct timeval _tickValue;
        struct timezone tz;
        gettimeofday(&_tickValue, &tz);

	return (double)_tickValue.tv_sec + (double)_tickValue.tv_usec/(1000*1000)
		 - (double)_tstart.tv_sec - (double)_tstart.tv_usec/(1000*1000);
#endif
}

void PerformanceTimer::tstart(void)
{
#ifdef _WIN32
	QueryPerformanceCounter(&_tstart);
#else
	gettimeofday(&_tstart, &tz);
#endif
}

void PerformanceTimer::tend(void)
{
#ifdef _WIN32
	QueryPerformanceCounter(&_tend);
#else
	gettimeofday(&_tend,&tz);
#endif
}

double PerformanceTimer::tval()
{
#ifdef _WIN32
	return ((double)_tend.QuadPart -
		(double)_tstart.QuadPart)/((double)freq.QuadPart);
#else
	double t1, t2;

	t1 =  (double)_tstart.tv_sec + (double)_tstart.tv_usec/(1000*1000);
	t2 =  (double)_tend.tv_sec + (double)_tend.tv_usec/(1000*1000);
	return t2-t1;
#endif
}

