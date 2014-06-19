
#ifndef _CLASS_PATHFOLLOWER
#define _CLASS_PATHFOLLOWER

#include <memory>

class Trajectory;
class PostureSolver;
class Robot;

struct PathPImpl;

class PathFollower {

public:
	 PathFollower(Robot& /*robot*/, const Trajectory& /*trajectory*/, PostureSolver& /*postureSolver*/);
	~PathFollower();

public:
	void Update();
	void Start();

private:
	std::auto_ptr<PathPImpl> pImpl_;
};

#endif //_CLASS_PATHFOLLOWER