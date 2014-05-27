#include <iostream>

#include "WorldManager.h"
#include "RobotManager.h"
#include "world\World.h"
#include "tools\MatrixDefs.h"
#include "Math.h"
#include "world\Obstacle.h"

using namespace matrices;
using namespace manip_core;
using namespace localmath;
typedef std::vector<Obstacle*> Quads;
Manager::WorldManager::WorldManager(){
}

Manager::WorldManager::~WorldManager(){
	// do nothing
}

World * Manager::WorldManager::initWorld(){
	std::cout << "Initing World...\n";
	World * world = new World();

	// BuildWorld
	float decal = -0.75;
	// FORWARD OBSTACLES
	Vector3 p1(-11, 2 + decal, 0.4f);
	Vector3 p2(-5.5, 2 + decal, 0.4f);
	Vector3 p3(-5.5, -1 + decal, 0.4f);
	Vector3 p4(-11, -1 + decal, 0.4f);

	setNextColor(0.7, 0.7, 0.7);
	addObstacle(p1, p2, p3, p4, *world);

	Vector3 p11(-5.2, 2 + decal, 0.f);
	Vector3 p21(-1, -1 + decal, 0.f);
	//manager.SetNextColor(0,0,1);
	generateStairChess(p11, p21, 0.4, 1.2, 3, 2, *world);

	Vector3 p13(-0.8, 2 + decal, 1.7f);
	Vector3 p23(3, -1 + decal, 1.7f);
	//manager.SetNextColor(1,0,0);
	generateXInclinedPlank(p13, p23, *world);

	p13 = Vector3(-0.8, 2 + decal, 0.1f);
	p23 = Vector3(3, -1 + decal, 0.1f);
	//manager.SetNextColor(1,0,0);
	generateXInclinedPlank(p13, p23, *world);

	Vector3 p131(3.1, 3 + decal, 2.0f);
	Vector3 p231(6, 1 + decal, 1.5f);
	generateXInclinedPlank(p131, p231, *world);

	Vector3 p132(3.1, 0 + decal, 1.5f);
	Vector3 p232(6, -2 + decal, 2.0f);
	//manager.SetNextColor(0,0,1);
	generateXInclinedPlank(p132, p232, *world);


	Vector3 p136(6.2, 1 + decal, 1.5f);
	Vector3 p236(18, 0.7 + decal, 1.5f);
	generateXInclinedPlank(p136, p236, *world);

	initialize(false, *world);

	return world;
}

World* Manager::WorldManager::buildWorld(){
	// Build world
	World * world = new World();

	// Generate obstacles and stuff
	return world;
}

void Manager::WorldManager::setNextColor(float r, float g, float b){
	color_[0] = r;
	color_[1] = g;
	color_[2] = b;
}

Quads& Manager::WorldManager::getQuads(){
	return quads;
}
void Manager::WorldManager::addObstacle(const Vector3 upLeft, const Vector3 upRight, const Vector3 downRight, const Vector3 downLeft, World &world){
	Obstacle * obstacle = new Obstacle(upLeft, upRight, downRight, downLeft, false);

	quads.push_back(obstacle);

	world.AddObstacle(obstacle);
}

void Manager::WorldManager::generateStairChess(const matrices::Vector3& upLeft, const matrices::Vector3& bottomRight, double heightInit, double heightFinal, const unsigned int depth, const unsigned int chessdepth, World &world){
	double xLength = bottomRight.x() - upLeft.x();
	double yLength = upLeft.y() - bottomRight.y();
	bool xIsLonger = xLength > yLength;
	double deltaHeight = (heightFinal - heightInit) / ((double)depth);
	double deltaCaseWidth = xIsLonger ? (xLength / ((double)depth)) : (yLength / ((double)depth));
	// stop condition, depth = 0, flat rectangle
	for (unsigned int i = 0; i < depth; ++i)
	{
		if (xIsLonger)
		{
			Vector3 uL(upLeft.x() + deltaCaseWidth * i, upLeft.y(), 0);
			Vector3 bR(upLeft.x() + deltaCaseWidth * (i + 1), bottomRight.y(), 0);
			generateChess(uL, bR, heightInit + i * deltaHeight, chessdepth, world);
		}
		else
		{
			Vector3 uL(upLeft.x(), bottomRight.y() + deltaCaseWidth * (i + 1), 0);
			Vector3 bR(bottomRight.x(), bottomRight.y() + deltaCaseWidth * i, 0);
			generateChess(uL, bR, heightInit + i * deltaHeight, chessdepth, world);
		}
	}
}

void Manager::WorldManager::generateChess(const matrices::Vector3& upLeft, const matrices::Vector3& bottomRight, double height, const unsigned int depth, World &world)
{
	// stop condition, depth = 0, flat rectangle
	if (0 == depth)
	{
		Vector3 nUL(upLeft.x(), upLeft.y(), height);
		Vector3 nBR(bottomRight.x(), bottomRight.y(), height);
		Vector3 upRight(bottomRight.x(), upLeft.y(), height);
		Vector3 downLeft(upLeft.x(), bottomRight.y(), height);
		addObstacle(nUL, upRight, nBR, downLeft, world);
	}
	else
	{
		Vector3 dx((bottomRight.x() - upLeft.x()) / 2., 0, 0);
		Vector3 dy(0, (upLeft.y() - bottomRight.y()) / 2., 0);
		//NUMBER dHeight = (1 == depth) ? 0.3 : 0.;
		unsigned int newDepth = depth - 1;
		generateChess(upLeft, upLeft + dx - dy, height, newDepth, world);
		if (depth != 1)
		{
			generateChess(upLeft + dx, bottomRight + dy, height, newDepth, world);
			generateChess(upLeft - dy, bottomRight - dx, height, newDepth, world);
		}
		generateChess(upLeft + dx - dy, bottomRight, height, newDepth, world);
	}
}

void Manager::WorldManager::generateXInclinedPlank(const matrices::Vector3& upLeft, const matrices::Vector3& bottomRight, World &world)
{
	Vector3 upRight(bottomRight.x(), upLeft.y(), upLeft.z());
	Vector3 downLeft(upLeft.x(), bottomRight.y(), bottomRight.z());
	addObstacle(upLeft, upRight, bottomRight, downLeft, world);
}

void Manager::WorldManager::initialize(bool activateCollision, World &world){
	world.Instantiate(activateCollision);
}