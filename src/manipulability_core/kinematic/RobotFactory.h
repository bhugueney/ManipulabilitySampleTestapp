
#ifndef _CLASS_ROBOT_FACTORY
#define _CLASS_ROBOT_FACTORY

#include "MatrixDefs.h"
#include "kinematic/Robot.h"
#include "kinematic/Enums.h"
//#include "kinematics/joint.h"

namespace factories{

struct RobotFactoryPimpl;

//typedef kinematics::joint<float, float, 3, 5, false> joint_def_t;

class RobotFactory {
public:
	  RobotFactory();
	 ~RobotFactory();
	 
public:
	Robot* CreateRobot(const manip_core::enums::robot::eRobots /*robot*/, const matrices::Matrix4& /*robotBasis*/) const;
    //Robot* CreateRobot(const joint_def_t& /*joint*/, const matrices::Matrix4& /*robotBasis*/) const;

private:
	std::auto_ptr<RobotFactoryPimpl> pImpl_;
	 
}; // RobotFactory

} // namespace factories

#endif // _CLASS_ROBOT_FACTORY
