
#ifndef _CLASS_SPLINETIMEMANAGER
#define _CLASS_SPLINETIMEMANAGER

#include "spline/exact_cubic.h"
#include "Timer.h"
#include "MatrixDefs.h"

namespace spline
{

class SplineTimeManager
{
public:
	explicit SplineTimeManager(const Timer::t_time /*zeroTime*/, curve_abc<>* /*cubic*/, float speed = 3.f);
	~SplineTimeManager();

private:
	//SplineTimeManager(const SplineTimeManager&);
	//SplineTimeManager& operator=(const SplineTimeManager&);

public:
	void Update(const Timer::t_time /*t*/, const Timer::t_time /*dt*/);
	matrices::Vector3 GetTarget() const;

public:
	bool IsObsolete();
	const curve_abc<>& GetCubic() {return *cubic_;}

private:
	const Timer::t_time zeroTime_;
	Timer::t_time currentTime_;
	const float speed_;
	curve_abc<>* cubic_;
	NUMBER distance_;
};

} // namespace spline
#endif //_CLASS_SPLINETIMEMANAGER