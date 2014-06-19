
#ifndef _CLASS_AUTOROTATE
#define _CLASS_AUTOROTATE

#include "TimerHandler.h"
#include "MatrixDefs.h"


class AutoIncline : public TimerHandled_ABC
{
public:
	explicit AutoIncline(const matrices::Vector3& /*angleSpeeds*/);
	~AutoIncline();

private:
	AutoIncline(const AutoIncline&);
	AutoIncline& operator=(const AutoIncline&);

public:
	virtual void Update(const Timer::t_time /*t*/, const Timer::t_time /*dt*/);

private:
	const matrices::Vector3 angleSpeeds_;
	double comDir_[2];
};

#endif //_CLASS_AUTOROTATE
