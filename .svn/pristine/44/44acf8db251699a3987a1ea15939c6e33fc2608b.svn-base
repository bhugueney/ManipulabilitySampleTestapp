
#include "Com.h"
#include "MatrixDefs.h"

#include <vector>

namespace factories
{
struct ComFactoryPimpl{
	ComFactoryPimpl()
	{
		// NOTHING
	}
	
	~ComFactoryPimpl()
	{
		// NOTHING
	}

	Com CreateCom(eSegmentMembers member)
	{
		switch (member)
		{
			case ForeArm:
				{
					return Com(0.022f, 0.682f);
				}
			case UpperAm:
				{
					return Com(0.028f, 0.436f);
				}
			case Thigh:
				{
					return Com(0.1f, 0.433f);
				}
			case Calf:
				{
					return Com(0.061f, 0.606f);
				}
			case Head:
				{
					return Com(0.081f, 1.f);
				}
			case Trunk:
				{
					return Com(0.497f, 0.5f);
				}
			default:
				return Com();
		}
	}
};
}

using namespace factories;


Com::Com(float weight, float distanceFromProximal)
	: weight_(weight)
	, distanceFromProximal_(distanceFromProximal)
{
	// NOTHING
}

Com::~Com()
{
	// NOTHING
}

matrices::Vector3 Com::Compute(const matrices::Vector3& proximal, const matrices::Vector3& distal) const
{
	return proximal + ((distal - proximal) * distanceFromProximal_);
	//return distal;
}


ComFactory::ComFactory()
	: pImpl_(new ComFactoryPimpl())
{
	// NOTHING
}


ComFactory::~ComFactory()
{
	// NOTHING
}

// LIB M2S, table de Dempster(1955)
Com ComFactory::CreateCom(eSegmentMembers member) const
{
	return pImpl_->CreateCom(member);
}
