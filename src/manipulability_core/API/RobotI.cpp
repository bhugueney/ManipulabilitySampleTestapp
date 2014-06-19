#include "RobotI.h"
#include "kinematic/RobotFactory.h"

using namespace manip_core;

extern "C" MANIPCORE_API Robot* FromFile(const char* /*file*/);
{
	factory_.CreateRobot(file);
	int i=0;
	for(WorldManagerI::T_TreeValues::const_iterator it = values.begin(); it != values.end(); ++it, ++i)
	{
		Joint* j = res->GetTree(i)->GetRoot();;
		for(WorldManagerI::T_JointValues::const_iterator it2 = (*it).begin(); j && it2 != (*it).end(); ++it2)
		{
			j->SetTheta(*it2);
			j = j->pChild_;
		}
		res->GetTree(i)->Compute();
	}
	return res;
}

