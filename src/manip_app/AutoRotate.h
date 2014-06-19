
#ifndef _CLASS_AUTOROTATE
#define _CLASS_AUTOROTATE

#include "TimerHandler.h"
#include "MatrixDefs.h"


class AutoRotate : public TimerHandled_ABC
{
public:
	explicit AutoRotate(const matrices::Vector3& /*angleSpeeds*/);
	~AutoRotate();

private:
	AutoRotate(const AutoRotate&);
	AutoRotate& operator=(const AutoRotate&);

public:
	virtual void Update(const Timer::t_time /*t*/, const Timer::t_time /*dt*/);

private:
	const matrices::Vector3 angleSpeeds_;
	double comDir_[2];
};

class AutoRotateLeg : public TimerHandled_ABC
{
public:
	explicit AutoRotateLeg(const matrices::Vector3& /*angleSpeeds*/);
	~AutoRotateLeg();

private:
	AutoRotateLeg(const AutoRotateLeg&);
	AutoRotateLeg& operator=(const AutoRotateLeg&);

public:
	virtual void Update(const Timer::t_time /*t*/, const Timer::t_time /*dt*/);

private:
	const matrices::Vector3 angleSpeeds_;
	double legBuffer_[3];
};

#endif //_CLASS_AUTOROTATE
