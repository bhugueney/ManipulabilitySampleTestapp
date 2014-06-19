
#include "DrawSamples.h"
#include "MatrixDefs.h"
#include "Simulation.h"

#include "API/RobotI.h"
#include "API/TreeI.h"
#include "API/JointI.h"
#include "Pi.h"

#include <drawstuff/drawstuff.h> // The drawing library for ODE;

using namespace matrices;
using namespace manip_core;

namespace matrices
{
	const Vector3 unitz(0,0,1);
}

DrawSamples::DrawSamples(const manip_core::RobotI* robot, const manip_core::TreeI* tree, int id, bool collide)
	: maxManip_(-1)
	, id_(id)
{
	Simulation::GetInstance()->postureManager_->VisitSamples(robot, tree, this, false);
}



DrawSamples::~DrawSamples()
{
	for(T_Tree::iterator it = trees_.begin(); it!= trees_.end(); ++it)
	{
		(*it)->Release();
	}
}

void DrawSamples::Visit(const manip_core::RobotI* robot, manip_core::TreeI* tree)
{
	trees_.push_back(tree);
	Vector3 dir = Simulation::GetInstance()->simpParams_.initDir_;
	double manip = tree->GetManipulability(dir(0), dir(1), dir(2));
	double normal[3];
	if(tree->GetObstacleNormal(normal) && manip > 0)
	{
		Vector3 vNormal; matrices::arrayToVect3(normal, vNormal);
		vNormal.normalize();
		manip *= Simulation::GetInstance()->simpParams_.initDir_.dot(vNormal);
		manip = manip < 0 ? 0 : manip;
	}
	maxManip_ = manip > maxManip_ ? manip : maxManip_;
	if(maxManip_ > 200) maxManip_= 200;
	drawTrees_.push_back(std::make_pair(manip, DrawTree(tree, 0.02f, 0.05f, id_)));
}


void DrawSamples::Draw(const matrices::Matrix4& currentTransform, bool transparency) const
{
	for(T_DrawTree::const_iterator it = drawTrees_.begin(); it!= drawTrees_.end(); ++it)
	{
		if(Simulation::GetInstance()->simpParams_.drawManip_)
		{
			(*it).second.Draw(currentTransform, transparency, true, (it->first) / maxManip_);
		}
		else
		{
			(*it).second.Draw(currentTransform, transparency, true, -1);
		}
		
	}
}



