
#ifndef _CLASS_DRAWMANAGER
#define _CLASS_DRAWMANAGER

#include "ManipManager.h"

#include "DrawPostures.h"
#include "DrawWorld.h"

namespace manip_core
{
	struct PostureManagerI;
	struct RobotI;
}

#if PROFILE
struct FPS;
#endif //PROFILE

class DrawManager{

public:
	 DrawManager(manip_core::ManipManager& /*manager*/);
	~DrawManager();

	 void Draw() const;
	 void Clear();

	 void SetTransparencyForAll(bool /*on*/);

	 const manip_core::RobotI* NextPosture();
	 const manip_core::RobotI* PreviousPosture();

	 void drawPostures(bool /*enablePostures*/);

private:
	DrawPostures drawPostures_;
	DrawWorld drawWorld_;
	bool drawShadowPostures_;
	bool transparencyOn_;
#if PROFILE
	FPS* fps_;
#endif //PROFILE
}; // class DrawWorld

#endif //_CLASS_DRAWMANAGER