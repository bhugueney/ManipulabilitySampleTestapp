
#ifndef _CLASS_XBOXMOTION
#define _CLASS_XBOXMOTION

#include "MotionHandler.h"
#include "XBOXController/Controller.h"


class XboxMotion : public TimerHandled_ABC
{
public:
	explicit XboxMotion();
	~XboxMotion();

private:
	XboxMotion(const XboxMotion&);
	XboxMotion& operator=(const XboxMotion&);

public:
	virtual void Update(const Timer::t_time /*t*/, const Timer::t_time /*dt*/);
	virtual void Reset();

private:
	xbox::Controller controller_;
};

#endif //_CLASS_XBOXMOTION
