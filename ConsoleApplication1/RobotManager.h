#include "skeleton\Robot.h"
#include "world\Obstacle.h"

using namespace manip_core;

namespace Manager{
	struct PImpl_;
	class RobotManager{
	public:
		RobotManager();
		~RobotManager();
	public:
		Robot * generateRobotInstance();
	
	private:
		matrices::Matrix4 generateRobotBasis();
		Robot * generateHumanRobot();
		Robot * generateQuadrupledRobot();
		void createHumanTorso(Tree& tree);
		void createHumanRightLeg(Tree& tree);
		void createHumanLeftLeg(Tree& tree);
		void createHumanRightArm(Tree& tree);
		void createHumanLeftArm(Tree& tree);
		
	};
}