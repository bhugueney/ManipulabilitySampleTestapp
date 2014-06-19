
#ifndef _CLASS_COLLISION_HANDLERCOLDET
#define _CLASS_COLLISION_HANDLERCOLDET

#include "CollisionHandler_ABC.h"

#include <memory>

struct CollisionHandlerPImpl;

class Tree;
class Robot;
class Obstacle;

class CollisionHandlerColdet : public CollisionHandler_ABC
{

public:
	 explicit CollisionHandlerColdet();
			 ~CollisionHandlerColdet();

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

#endif //_CLASS_COLLISION_HANDLERCOLDET