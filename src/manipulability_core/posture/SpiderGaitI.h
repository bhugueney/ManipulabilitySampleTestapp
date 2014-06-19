
#ifndef _CLASS_SPIDER_GAITI
#define _CLASS_SPIDER_GAITI

class SpiderGaitI
{
public:
	virtual void Update(unsigned long time) = 0;
	virtual int Phase() const = 0;
	virtual bool MidPhase() const = 0;
};


#endif //_CLASS_SPIDER_GAITI
