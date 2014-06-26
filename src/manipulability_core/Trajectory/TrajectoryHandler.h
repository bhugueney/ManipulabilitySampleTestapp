
#ifndef _CLASS_TRAJECTORYHANDLER
#define _CLASS_TRAJECTORYHANDLER

#include "spline/curve_abc.h"
#include "MatrixDefs.h"

#include "world/World.h"

class World;
class Robot;
class Tree;

struct TrajectoryHandlerPimpl;

// Everything in world coordinates
class TrajectoryHandler {

public:
	 TrajectoryHandler(const World& /*world*/); // in robot coordinates
	~TrajectoryHandler();

private:
	TrajectoryHandler(const TrajectoryHandler&);
	TrajectoryHandler& operator=(const TrajectoryHandler&);

public:
	spline::curve_abc<>* ComputeTrajectory(const Robot& /*robot*/, const Tree& /*current*/, const Tree& /*estimated*/, const matrices::Vector3& /*target*/);
	
private:
    const World& world_;
};


#endif //_CLASS_TRAJECTORYHANDLER
