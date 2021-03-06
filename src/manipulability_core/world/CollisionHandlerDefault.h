
#ifndef _CLASS_COLLISION_HANDLERCOLDET
#define _CLASS_COLLISION_HANDLERCOLDET

#include "CollisionHandler_ABC.h"
#include "world/World.h"
#include "Intersection.h"

#include <memory>


class Tree;
class Robot;
class Obstacle;
class World;

class CollisionHandlerDefault : public CollisionHandler_ABC
{

public:
	 explicit CollisionHandlerDefault(const World& /*world*/);
			 ~CollisionHandlerDefault();

//helper
public:

//request
public:
	virtual void AddObstacle(const Obstacle* /*obstacle*/);
	virtual bool IsColliding(const Robot& /*robot*/, const Tree& /*tree*/);
	virtual bool IsSoftColliding(const Robot& /*robot*/, const Tree& /*tree*/);
	virtual void Instantiate();
	
private:
	const World& world_;
	const Intersection intersection_;
};

#endif //_CLASS_COLLISION_HANDLERCOLDET