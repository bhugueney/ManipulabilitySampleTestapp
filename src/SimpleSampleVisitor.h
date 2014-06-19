
#ifndef _CLASS_SIMPLESAMPLEVISITOR
#define _CLASS_SIMPLESAMPLEVISITOR

#include "kinematic/SampleGeneratorVisitor_ABC.h"
#include "MatrixDefs.h"

#include <memory>

class Sample;
class SampleGenerator;
struct PImpl;
class SimpleSampleVisitor : public SampleGeneratorVisitor_ABC
{

public:
	 SimpleSampleVisitor();
	~SimpleSampleVisitor();

public:
	virtual void Visit(Sample& /*sample*/);
	void Configure(float /*force*/, float /*velocity*/);
	Sample* Run(const SampleGenerator& /*generator*/, const matrices::Vector3& /*directionVel*/,  const matrices::Vector3& /*directionForce*/, const matrices::Vector3& /* target */);

private:
	std::auto_ptr<PImpl> pImpl_;
};

#endif //_CLASS_SIMPLESAMPLEVISITOR