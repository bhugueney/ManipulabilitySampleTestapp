
#ifndef _ENUMS
#define _ENUMS

#define stringify( name ) # name

#include "API/WorldManagerI.h"

namespace manip_core
{
namespace enums
{
	enum eMembers{ 
		RightArm,
		RightArmCrouch,
		RightArmCrouch180,
		RightArmEllipse,
		RightArmEscalade,
		RightArmCanap,
		LeftArm,
		LeftArmEscalade,
		LeftArmCanap,
		LeftArmCrouch,
		LeftArmCrouch180,
		LeftLeg,
		LeftLegEscalade,
		LeftLegCanap,
		LeftLegCrouch,
		LeftLegCrouch180,
		RightLeg,
		RightLegEscalade,
		RightLegCanap,
		RightLegCrouch,
		RightLegCrouch180,
		LeftLegWalk,
		RightLegWalk,
		HumanTorso,
		HumanTorsoCrouch,
		HumanTorsoCrouch180,
		QuadrupedTorso,
		QuadrupedLeg,
		QuadrupedLegLeft,
		QuadrupedLegRight,
		QuadrupedLegDownLeft,
		QuadrupedLegDownRight,
		SpiderTorso,
		SpiderTorsoRace,
		SpiderLeg,
		SpiderLegSix,
		SpiderLegRace,
		UnknownTree
	};
		
  const std::string myEnumTreesNames[] = 
  {
	  stringify( RightArm ),
	  stringify( RightArmEllipse ),
	  stringify( LeftArm ),
	  stringify( LeftLeg ),
	  stringify( RightLeg ),
	  stringify( LeftLegWalk ),
	  stringify( RightLegWalk ),
	  stringify( HumanTorso ),
	  stringify( HumanTorsoCrouch ),
	  stringify( QuadrupedTorso ),
	  stringify( QuadrupedLeg ),
	  stringify( QuadrupedLegLeft ),
	  stringify( QuadrupedLegRight ),
	  stringify( QuadrupedLegDownLeft ),
	  stringify( QuadrupedLegDownRight ),
	  stringify( SpiderTorso ),
	  stringify( SpiderTorsoRace ),
	  stringify( SpiderLeg ),
	  stringify( SpiderLegRace ),
	  stringify( UnknownTree )
  };

  const std::string myEnumRobotNames[] = 
  {
	  stringify( robot::Human ),
	  stringify( robot::HumanWalk ),
	  stringify( robot::HumanEllipse ),
	  stringify( robot::Quadruped ),
	  stringify( robot::QuadrupedDown ),
	  stringify( robot::Spider ),
	  stringify( robot::SpiderRace ),
	  stringify( robot::HumanCrouch ),
	  stringify( robot::HumanEscalade ),
	  stringify( robot::HumanCanap ),
	  stringify( robot::UnknownRobot )
  };

  static const std::string StringTreeType( const eMembers treeType )
  {
	return myEnumTreesNames[treeType];
  }

  static const std::string StringRobotType( const robot::eRobots robotType )
  {
	return myEnumRobotNames[robotType];
  }
} // namespace enums
} // namespace manip_core

#endif //_ENUMS