#include "SplineTimeManager.h"

#include "MatrixDefs.h"


using namespace matrices;
using namespace spline;

SplineTimeManager::SplineTimeManager(const Timer::t_time zeroTime, spline::curve_abc<>* cubic, float speed)
	: cubic_(cubic)
	, zeroTime_(zeroTime)
	, speed_(speed)
	, distance_(0.)
{
	// NOTHING
}

SplineTimeManager::~SplineTimeManager()
{
	delete cubic_;
}

void SplineTimeManager::Update(const Timer::t_time t, const Timer::t_time dt)
{
	Timer::t_time elapsed = t - zeroTime_; // total time elapsed since creation
	distance_ = (double)elapsed * speed_;   //distance crossed
}

matrices::Vector3 SplineTimeManager::GetTarget() const
{
	matrices::Vector3 res = cubic_->operator()(std::min(cubic_->max(), distance_));
	return res;
}

bool SplineTimeManager::IsObsolete()
{
	return distance_ > cubic_->max();
}
