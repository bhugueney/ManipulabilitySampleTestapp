
#include "DrawManager.h"
#include "DrawWorld.h"
#include "DrawPostures.h"

#include "API/RobotI.h"

#include "MatrixDefs.h"

#include "PostureManager.h"

#include <drawstuff/drawstuff.h> // The drawing library for ODE;

#include <vector>

using namespace std;
using namespace matrices;



//TODO listener in case of adding new trees
DrawManager::DrawManager(manip_core::ManipManager& manager)
	: drawWorld_(manager)
	, drawShadowPostures_(false)
	, transparencyOn_(false)
{
	manager.GetPostureManager()->RegisterPostureCreatedListenerI(&drawPostures_);
}


DrawManager::~DrawManager()
{
	// NOTHING
}

void DrawManager::drawPostures(bool enablePostures)
{
	drawShadowPostures_ = enablePostures;
}

void DrawManager::SetTransparencyForAll(bool on)
{
	transparencyOn_ = on;
}


void DrawManager::Draw() const
{
	drawWorld_.Draw();
	if(drawShadowPostures_)
	{
		drawPostures_.Draw(transparencyOn_);
	}
	else
	{
		drawPostures_.DrawOne(transparencyOn_);
	}
}

void DrawManager::Clear()
{
	drawPostures_.Clear();
}

const manip_core::RobotI* DrawManager::NextPosture()
{
	return drawPostures_.Next();
}

const manip_core::RobotI* DrawManager::PreviousPosture()
{
	return drawPostures_.Previous();
}

