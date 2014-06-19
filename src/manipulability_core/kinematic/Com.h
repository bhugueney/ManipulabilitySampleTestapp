
#ifndef _CLASS_COM
#define _CLASS_COM

#include "MatrixDefs.h"
#include "Exports.h"

#include <memory>

class Com {
public:
	 explicit Com(float weight = 0.f, float distanceFromProximal = 0.f);
	 ~Com();
	 
	matrices::Vector3 Compute(const matrices::Vector3& /*proximal*/, const matrices::Vector3& /*distal*/) const;

public:
	float weight_;
	float distanceFromProximal_;

	
private:
	 Com& Com::operator =(const Com&);
}; // Com

namespace factories
{

enum eSegmentMembers{ 
	ForeArm,
	UpperAm, 
	Thigh, 
	Calf,
	Head,
	Trunk,
	None
};

struct ComFactoryPimpl;

class ComFactory {
public:
	 ComFactory();
	~ComFactory();

public:
	 Com CreateCom(eSegmentMembers member) const;

private:
	std::auto_ptr<ComFactoryPimpl> pImpl_;

private:
	 ComFactory& ComFactory::operator =(const ComFactory&);
	 ComFactory(const ComFactory&);
	 
}; // ComFactory

} // namespace factories

#endif //_CLASS_COM