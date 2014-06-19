
#ifndef _CLASS_SPIDER_GAIT
#define _CLASS_SPIDER_GAIT

#include "SpiderGaitI.h"
#include "Timer.h"

class SpiderGait : public SpiderGaitI
{

public:
	 SpiderGait();
	~SpiderGait();

public:
	// in ms
	virtual void Update(unsigned long time);
	virtual int Phase() const;
	virtual bool MidPhase() const;

private:
	int phase_;
	unsigned long elapsed_;
	unsigned long totalelapsed_;
	unsigned long phaseLength_;
};

#endif //_CLASS_SPIDER_GAIT

