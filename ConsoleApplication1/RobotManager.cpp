#include "RobotManager.h"
#include "skeleton\Robot.h"
#include "Math.h"
#include <iostream>
#include "skeleton\Joint.h"

using namespace localmath;
using namespace matrices;

#if (USEFLOAT)
typedef float NUMBER;
#else
typedef double NUMBER;
#endif

typedef Eigen::MatrixXf Matrix44;

Manager::RobotManager::RobotManager (){

}

Manager::RobotManager::~RobotManager(){

}

Robot * Manager::RobotManager::generateRobotInstance(){
	// Generate robot
	//Robot * robot = generateHumanRobot();
	Robot * robot = generateQuadrupledRobot();
	return robot;
}

// local methods
matrices::Matrix4 Manager::RobotManager::generateRobotBasis(){
	Matrix4 robotBasis= Matrix4::Identity(4, 4);

	robotBasis(0, 3) = -10;
	robotBasis(1, 3) = 0.;
	robotBasis(2, 3) = 1.8;
	return robotBasis; 
}

Robot * Manager::RobotManager::generateQuadrupledRobot(){
	// Generate robot basis
	Matrix4 robotBasis = generateRobotBasis();

	// Create Quadrupled 
	const Tree::TREE_ID QUADRUPLED_TORSO = 4;
	const Tree::TREE_ID QUADRUPLED_RIGHT_LEG = 0;
	const Tree::TREE_ID QUADRUPLED_LEFT_LEG = 1;
	const Tree::TREE_ID QUADRUPLED_RIGHT_ARM = 2;
	const Tree::TREE_ID QUADRUPLED_LEFT_ARM = 3;

	const Vector3 TORSO_ROOT = Vector3(0.0, 0.0, 0);
	const Vector3 RIGHT_LEG_ROOT = Vector3(-1.2, -0.25, -0.1);
	const Vector3 LEFT_LEG_ROOT = Vector3(-1.2, 0.25, -0.1);
	const Vector3 LEFT_ARM_ROOT = Vector3(0.3, 0.25, -0.1);
	const Vector3 RIGHT_ARM_ROOT = Vector3(0.3, -0.25, -0.1);

	const Vector3 TORSO_J1 = Vector3(TORSO_ROOT(0) - 1.2, TORSO_ROOT(1), TORSO_ROOT(2));
	const Vector3 TORSO_J2 = Vector3(TORSO_ROOT(0) + 1.3, TORSO_ROOT(1), TORSO_ROOT(2));
	const Vector3 RIGHT_LEG_J1 = RIGHT_LEG_ROOT;
	const Vector3 RIGHT_LEG_J2 = RIGHT_LEG_ROOT;
	const Vector3 RIGHT_LEG_J3 = Vector3(RIGHT_LEG_ROOT(0), RIGHT_LEG_ROOT(1), RIGHT_LEG_ROOT(2) - 0.8);
	const Vector3 RIGHT_LEG_J4 = Vector3(RIGHT_LEG_ROOT(0), RIGHT_LEG_ROOT(1), RIGHT_LEG_ROOT(2) - 0.8);
	const Vector3 RIGHT_LEG_J5 = Vector3(RIGHT_LEG_ROOT(0), RIGHT_LEG_ROOT(1), RIGHT_LEG_ROOT(2) - 1.7);
	const Vector3 LEFT_LEG_J1 = LEFT_LEG_ROOT;
	const Vector3 LEFT_LEG_J2 = LEFT_LEG_ROOT;
	const Vector3 LEFT_LEG_J3 = Vector3(LEFT_LEG_ROOT(0), LEFT_LEG_ROOT(1), LEFT_LEG_ROOT(2) - 0.8);
	const Vector3 LEFT_LEG_J4 = Vector3(LEFT_LEG_ROOT(0), LEFT_LEG_ROOT(1), LEFT_LEG_ROOT(2) - 0.8);
	const Vector3 LEFT_LEG_J5 = Vector3(LEFT_LEG_ROOT(0), LEFT_LEG_ROOT(1), LEFT_LEG_ROOT(2) - 1.7);
	const Vector3 LEFT_ARM_J1 = LEFT_ARM_ROOT;
	const Vector3 LEFT_ARM_J2 = LEFT_ARM_ROOT;
	const Vector3 LEFT_ARM_J3 = Vector3(LEFT_ARM_ROOT(0), LEFT_ARM_ROOT(1), LEFT_ARM_ROOT(2) - 0.8);
	const Vector3 LEFT_ARM_J4 = Vector3(LEFT_ARM_ROOT(0), LEFT_ARM_ROOT(1), LEFT_ARM_ROOT(2) - 0.8);
	const Vector3 LEFT_ARM_J5 = Vector3(LEFT_ARM_ROOT(0), LEFT_ARM_ROOT(1), LEFT_ARM_ROOT(2) - 1.7);
	const Vector3 RIGHT_ARM_J1 = RIGHT_ARM_ROOT;
	const Vector3 RIGHT_ARM_J2 = RIGHT_ARM_ROOT;
	const Vector3 RIGHT_ARM_J3 = Vector3(RIGHT_ARM_ROOT(0), RIGHT_ARM_ROOT(1), RIGHT_ARM_ROOT(2) - 0.8);
	const Vector3 RIGHT_ARM_J4 = Vector3(RIGHT_ARM_ROOT(0), RIGHT_ARM_ROOT(1), RIGHT_ARM_ROOT(2) - 0.8);
	const Vector3 RIGHT_ARM_J5 = Vector3(RIGHT_ARM_ROOT(0), RIGHT_ARM_ROOT(1), RIGHT_ARM_ROOT(2) - 1.7);

	Tree * robotTorso = new Tree(QUADRUPLED_TORSO);
	Tree * rightLeg = new Tree(QUADRUPLED_RIGHT_LEG);
	Tree * leftLeg = new Tree(QUADRUPLED_LEFT_LEG);
	Tree * rightArm = new Tree(QUADRUPLED_RIGHT_ARM);
	Tree * leftArm = new Tree(QUADRUPLED_LEFT_ARM);

	// Torso joints
	Joint* tj0 = new Joint(TORSO_ROOT, unity, NUMBER(0), NUMBER(0), NUMBER(0));
	robotTorso->InsertRoot(tj0);

	Joint* tj1 = new Joint(TORSO_J1, unity, NUMBER(0), NUMBER(0), NUMBER(0));
	robotTorso->InsertChild(tj0, tj1);

	Joint* tj2 = new Joint(TORSO_J2, unity, NUMBER(0), NUMBER(0), NUMBER(0));
	robotTorso->InsertChild(tj1, tj2);

	// Right leg joints
	Joint* rlj0 = new Joint(RIGHT_LEG_ROOT, unity, NUMBER(-120), NUMBER(120), NUMBER(0));
	rightLeg->InsertRoot(rlj0);

	Joint* rlj1 = new Joint(RIGHT_LEG_J1, unitx, NUMBER(-30), NUMBER(0), NUMBER(0));
	rightLeg->InsertChild(rlj0, rlj1);

	Joint* rlj2 = new Joint(RIGHT_LEG_J2, unitz, NUMBER(-15), NUMBER(15), NUMBER(0));
	rightLeg->InsertChild(rlj1, rlj2);

	Joint* rlj3 = new Joint(RIGHT_LEG_J3, unity, NUMBER(0), NUMBER(180), NUMBER(30));
	rightLeg->InsertChild(rlj2, rlj3);

	Joint* rlj4 = new Joint(RIGHT_LEG_J4, unitx, NUMBER(-10.), NUMBER(10.), NUMBER(0.));
	rightLeg->InsertChild(rlj3, rlj4);

	Joint* rlj5 = new Joint(RIGHT_LEG_J5, zero);
	rightLeg->InsertChild(rlj4, rlj5);

	// Left leg joints
	Joint* llj0 = new Joint(LEFT_LEG_ROOT, unity, NUMBER(-180.), NUMBER(180), NUMBER(0));
	leftLeg->InsertRoot(llj0);

	Joint* llj1 = new Joint(LEFT_LEG_J1, unitx, NUMBER(0), NUMBER(30), NUMBER(0));
	leftLeg->InsertChild(llj0, llj1);

	Joint* llj2 = new Joint(LEFT_LEG_J2, unitz, NUMBER(-15), NUMBER(15), NUMBER(0));
	leftLeg->InsertChild(llj1, llj2);

	Joint* llj3 = new Joint(LEFT_LEG_J3, unity, NUMBER(0), NUMBER(120), NUMBER(30));
	leftLeg->InsertChild(llj2, llj3);

	Joint* llj4 = new Joint(LEFT_LEG_J4, unitx, NUMBER(-10.), NUMBER(10.), NUMBER(0.));
	leftLeg->InsertChild(llj3, llj4);

	Joint* llj5 = new Joint(LEFT_LEG_J5, zero);
	leftLeg->InsertChild(llj4, llj5);

	// Right Arm joints
	Joint* raj0 = new Joint(RIGHT_ARM_ROOT, unity, NUMBER(-180.), NUMBER(180.), NUMBER(0));
	rightArm->InsertRoot(raj0);

	Joint* raj1 = new Joint(RIGHT_ARM_J1, unitx, NUMBER(0), NUMBER(30), NUMBER(0));
	rightArm->InsertChild(raj0, raj1);

	Joint* raj2 = new Joint(RIGHT_ARM_J2, unitz, NUMBER(-15), NUMBER(15), NUMBER(0));
	rightArm->InsertChild(raj1, raj2);

	Joint* raj3 = new Joint(RIGHT_ARM_J3, unity, NUMBER(0), NUMBER(120), NUMBER(30));
	rightArm->InsertChild(raj2, raj3);

	Joint* raj4 = new Joint(RIGHT_ARM_J4, unitx, NUMBER(-10.), NUMBER(10.), NUMBER(0.));
	rightArm->InsertChild(raj3, raj4);

	Joint* raj5 = new Joint(RIGHT_ARM_J5, zero);
	rightArm->InsertChild(raj4, raj5);

	// Left Arm joints
	Joint* laj0 = new Joint(LEFT_ARM_ROOT, unity, NUMBER(-120), NUMBER(120), NUMBER(0));
	leftArm->InsertRoot(laj0);

	Joint* laj1 = new Joint(LEFT_ARM_J1, unitx, NUMBER(-30), NUMBER(0), NUMBER(0));
	leftArm->InsertChild(laj0, laj1);

	Joint* laj2 = new Joint(LEFT_ARM_J2, unitz, NUMBER(-15), NUMBER(15), NUMBER(0));
	leftArm->InsertChild(laj1, laj2);

	Joint* laj3 = new Joint(LEFT_ARM_J3, unity, NUMBER(0), NUMBER(180), NUMBER(30));
	leftArm->InsertChild(laj2, laj3);

	Joint* laj4 = new Joint(LEFT_ARM_J4, unitx, NUMBER(-10.), NUMBER(10.), NUMBER(0.));
	leftArm->InsertChild(laj3, laj4);

	Joint* laj5 = new Joint(LEFT_ARM_J5, zero);
	leftArm->InsertChild(laj4, laj5);

	Robot * robot = new Robot(robotBasis, robotTorso);
	robot->AddTree(rightLeg, matrices::Vector3(-1.2, 0, 0), 0);
	robot->AddTree(leftLeg, matrices::Vector3(-1.2, 0, 0), 0);
	robot->AddTree(leftArm, matrices::Vector3(0.3, 0, 0), 1);
	robot->AddTree(rightArm, matrices::Vector3(0.3, 0, 0), 1);

	return robot;
}

Robot * Manager::RobotManager::generateHumanRobot(){
	// Generate robot basis
	Matrix4 robotBasis = generateRobotBasis();

	// Create Human 
	const Tree::TREE_ID HUMAN_TORSO = 0;
	const Tree::TREE_ID HUMAN_RIGHT_LEG = 0;
	const Tree::TREE_ID HUMAN_LEFT_LEG = 1;
	const Tree::TREE_ID HUMAN_RIGHT_ARM = 2;
	const Tree::TREE_ID HUMAN_LEFT_ARM = 3;

	Tree * robotTorso = new Tree(HUMAN_TORSO);
	Tree * rightLeg = new Tree(HUMAN_RIGHT_LEG);
	Tree * leftLeg = new Tree(HUMAN_LEFT_LEG);
	Tree * rightArm = new Tree(HUMAN_RIGHT_ARM);
	Tree * leftArm = new Tree(HUMAN_LEFT_ARM);

	createHumanTorso(*robotTorso);

	createHumanRightLeg(*rightLeg);
	createHumanLeftLeg(*leftLeg);
	createHumanRightArm(*rightArm);
	createHumanLeftArm(*leftArm);

	Robot * robot = new Robot(robotBasis, robotTorso);
	
	robot->AddTree(rightLeg, Vector3(0.0, 0, 0), 0);
	robot->AddTree(leftLeg, Vector3(0.0, 0, 0), 0);
	robot->AddTree(rightArm, Vector3(0.0, 0, 1.2), 1);
	robot->AddTree(leftArm, Vector3(0.0, 0, 1.2), 1);
	return robot;
}

void Manager::RobotManager::createHumanTorso(Tree & tree){
	// Default Human
	const Vector3 TORSO_ROOT = Vector3(0.0, 0.0, 0);
	const Vector3 TORSO_J1 = Vector3(TORSO_ROOT(0), TORSO_ROOT(1), TORSO_ROOT(2) + 1.5);
	const Vector3 TORSO_J2 = Vector3(TORSO_ROOT(0), TORSO_ROOT(1), TORSO_ROOT(2) + 1.7);

	Joint* joint0 = new Joint(TORSO_ROOT, unity, NUMBER(0), NUMBER(0), NUMBER(0));
	tree.InsertRoot(joint0);
	Joint * joint1 = new Joint(TORSO_J1, unity, NUMBER(0), NUMBER(0), NUMBER(0));
	tree.InsertChild(joint0, joint1);
	Joint * joint2 = new Joint(TORSO_J2, unitx, NUMBER(0));
	tree.InsertChild(joint1, joint2);
}

void Manager::RobotManager::createHumanRightLeg(Tree & tree){
	const Vector3 RIGHT_LEG_ROOT = Vector3(0.0, -0.2, 0.1);
	const Vector3 RIGHT_LEG_J1 = RIGHT_LEG_ROOT;
	const Vector3 RIGHT_LEG_J2 = RIGHT_LEG_ROOT;
	const Vector3 RIGHT_LEG_J3 = Vector3(RIGHT_LEG_ROOT(0), RIGHT_LEG_ROOT(1), RIGHT_LEG_ROOT(2) - 1.0);
	const Vector3 RIGHT_LEG_J4 = Vector3(RIGHT_LEG_ROOT(0), RIGHT_LEG_ROOT(1), RIGHT_LEG_ROOT(2) - 1.0);
	const Vector3 RIGHT_LEG_J5 = Vector3(RIGHT_LEG_ROOT(0), RIGHT_LEG_ROOT(1), RIGHT_LEG_ROOT(2) - 2.2);

	Joint * joint0 = new Joint(RIGHT_LEG_ROOT, unity, NUMBER(-120), NUMBER(-10), NUMBER(-10));
	Joint * joint1 = new Joint(RIGHT_LEG_J1, unitx, NUMBER(-30), NUMBER(30), NUMBER(0));
	Joint * joint2 = new Joint(RIGHT_LEG_J2, unitz, NUMBER(-30), NUMBER(30), NUMBER(0));
	Joint * joint3 = new Joint(RIGHT_LEG_J3, unity, NUMBER(0), NUMBER(110), NUMBER(30));
	Joint * joint4 = new Joint(RIGHT_LEG_J4, unitx, NUMBER(-15), NUMBER(15), NUMBER(0));
	Joint * joint5 = new Joint(RIGHT_LEG_J5, zero, NUMBER(0));

	tree.InsertRoot(joint0);
	tree.InsertChild(joint0, joint1);
	tree.InsertChild(joint1, joint2);
	tree.InsertChild(joint2, joint3);
	tree.InsertChild(joint3, joint4);
	tree.InsertChild(joint4, joint5);
}

void Manager::RobotManager::createHumanLeftLeg(Tree & tree){
	const Vector3 LEFT_LEG_ROOT = Vector3(0.0, 0.2, 0.1);
	const Vector3 LEFT_LEG_J1 = LEFT_LEG_ROOT;
	const Vector3 LEFT_LEG_J2 = LEFT_LEG_ROOT;
	const Vector3 LEFT_LEG_J3 = Vector3(LEFT_LEG_ROOT(0), LEFT_LEG_ROOT(1), LEFT_LEG_ROOT(2) - 1.0);
	const Vector3 LEFT_LEG_J4 = Vector3(LEFT_LEG_ROOT(0), LEFT_LEG_ROOT(1), LEFT_LEG_ROOT(2) - 1.0);
	const Vector3 LEFT_LEG_J5 = Vector3(LEFT_LEG_ROOT(0), LEFT_LEG_ROOT(1), LEFT_LEG_ROOT(2) - 2.2);

	Joint * joint0 = new Joint(LEFT_LEG_ROOT, unity, NUMBER(-100), NUMBER(-10), NUMBER(-10));
	Joint * joint1 = new Joint(LEFT_LEG_J1, unitx, NUMBER(-30), NUMBER(30), NUMBER(0));
	Joint * joint2 = new Joint(LEFT_LEG_J2, unitz, NUMBER(-30), NUMBER(30), NUMBER(0));
	Joint * joint3 = new Joint(LEFT_LEG_J3, unity, NUMBER(0), NUMBER(110), NUMBER(30));
	Joint * joint4 = new Joint(LEFT_LEG_J4, unitx, NUMBER(-15), NUMBER(15), NUMBER(0));
	Joint * joint5 = new Joint(LEFT_LEG_J5, zero, NUMBER(0));

	tree.InsertRoot(joint0);
	tree.InsertChild(joint0, joint1);
	tree.InsertChild(joint1, joint2);
	tree.InsertChild(joint2, joint3);
	tree.InsertChild(joint3, joint4);
	tree.InsertChild(joint4, joint5);
}

void Manager::RobotManager::createHumanRightArm(Tree & tree){
	const Vector3 RIGHT_ARM_ROOT = Vector3(0.0, -0.4, 1.3);
	const Vector3 RIGHT_ARM_J1 = RIGHT_ARM_ROOT;
	const Vector3 RIGHT_ARM_J2 = RIGHT_ARM_ROOT;
	const Vector3 RIGHT_ARM_J3 = Vector3(RIGHT_ARM_ROOT(0), RIGHT_ARM_ROOT(1), RIGHT_ARM_ROOT(2) - 0.8);
	const Vector3 RIGHT_ARM_J4 = Vector3(RIGHT_ARM_ROOT(0), RIGHT_ARM_ROOT(1), RIGHT_ARM_ROOT(2) - 0.8);
	const Vector3 RIGHT_ARM_J5 = Vector3(RIGHT_ARM_ROOT(0), RIGHT_ARM_ROOT(1), RIGHT_ARM_ROOT(2) - 1.7);
	
	Joint * joint0 = new Joint(RIGHT_ARM_ROOT, unity, NUMBER(-110), NUMBER(70), NUMBER(0));
	Joint * joint1 = new Joint(RIGHT_ARM_J1, unitx, NUMBER(-30), NUMBER(90), NUMBER(0));
	Joint * joint2 = new Joint(RIGHT_ARM_J2, unitz, NUMBER(-90), NUMBER(45), NUMBER(20));
	Joint * joint3 = new Joint(RIGHT_ARM_J3, unity, NUMBER(-120), NUMBER(0), NUMBER(-30));
	Joint * joint4 = new Joint(RIGHT_ARM_J4, unitx, NUMBER(-180), NUMBER(180), NUMBER(0));
	Joint * joint5 = new Joint(RIGHT_ARM_J5, zero, NUMBER(0));

	tree.InsertRoot(joint0);
	tree.InsertChild(joint0, joint1);
	tree.InsertChild(joint1, joint2);
	tree.InsertChild(joint2, joint3);
	tree.InsertChild(joint3, joint4);
	tree.InsertChild(joint4, joint5);
}

void Manager::RobotManager::createHumanLeftArm(Tree & tree){
	const Vector3 LEFT_ARM_ROOT = Vector3(0.0, 0.4, 1.3);
	const Vector3 LEFT_ARM_J1 = LEFT_ARM_ROOT;
	const Vector3 LEFT_ARM_J2 = LEFT_ARM_ROOT;
	const Vector3 LEFT_ARM_J3 = Vector3(LEFT_ARM_ROOT(0), LEFT_ARM_ROOT(1), LEFT_ARM_ROOT(2) - 0.8);
	const Vector3 LEFT_ARM_J4 = Vector3(LEFT_ARM_ROOT(0), LEFT_ARM_ROOT(1), LEFT_ARM_ROOT(2) - 0.8);
	const Vector3 LEFT_ARM_J5 = Vector3(LEFT_ARM_ROOT(0), LEFT_ARM_ROOT(1), LEFT_ARM_ROOT(2) - 1.7);

	Joint * joint0 = new Joint(LEFT_ARM_ROOT, unity, NUMBER(-110), NUMBER(70), NUMBER(0));
	Joint * joint1 = new Joint(LEFT_ARM_J1, unitx, NUMBER(-90), NUMBER(30), NUMBER(0));
	Joint * joint2 = new Joint(LEFT_ARM_J2, unitz, NUMBER(-45), NUMBER(45), NUMBER(-20));
	Joint * joint3 = new Joint(LEFT_ARM_J3, unity, NUMBER(-120), NUMBER(0), NUMBER(-30));
	Joint * joint4 = new Joint(LEFT_ARM_J4, unitx, NUMBER(-180), NUMBER(180), NUMBER(0));
	Joint * joint5 = new Joint(LEFT_ARM_J5, zero, NUMBER(0));

	tree.InsertRoot(joint0);
	tree.InsertChild(joint0, joint1);
	tree.InsertChild(joint1, joint2);
	tree.InsertChild(joint2, joint3);
	tree.InsertChild(joint3, joint4);
	tree.InsertChild(joint4, joint5);
}
