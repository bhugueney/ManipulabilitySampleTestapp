
#ifndef _CLASS_COLLISION_HANDLEROZCOLLIDE
#define _CLASS_COLLISION_HANDLEROZCOLLIDE

#include "CollisionHandler_ABC.h"

#include <memory>

struct CollisionHandlerPImpl;

class Tree;
class Robot;
class Obstacle;

class CollisionHandlerOzCollide : public CollisionHandler_ABC
{

public:
	 explicit CollisionHandlerOzCollide();
			 ~CollisionHandlerOzCollide();

//helper
public:

//request
public:
	void AddObstacle(const Obstacle* /*obstacle*/);
	bool IsColliding(const Robot& /*robot*/, const Tree& /*tree*/);
	void Instantiate();
	
private:
	std::auto_ptr<CollisionHandlerPImpl> pImpl_;
};

#endif //_CLASS_COLLISION_HANDLEROZCOLLIDE