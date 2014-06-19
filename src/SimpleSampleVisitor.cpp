#include "SimpleSampleVisitor.h"

#include "kinematic/SampleGenerator.h"
#include "kinematic/Sample.h"
#include "kinematic/Tree.h"

#include <math.h>

using namespace matrices;

struct PImpl
{
	PImpl()
		: sample_(0)
		, forceCoeff_(0)
		, velocCoeff_(0)
		, currentOutput_(0)
		, currentDirForce_(0, 0, 0)
		, currentDirVel_(0, 0, 0)
		, currentTarget_(0, 0, 0)
	{
		//NOTHING
	}
	~PImpl()
	{
		//NOTHING
	}

	void Reset()
	{
		sample_ = 0;
		currentOutput_ = 0;
		currentDirVel_ = Vector3::Zero();
		currentDirForce_ = Vector3::Zero();
		currentTarget_ = Vector3::Zero();
	}

	NUMBER FastDistance(const Vector3& p1)
	{
		return pow(p1(0) - currentTarget_(0), 2) + pow(p1(1) - currentTarget_(1), 2) + pow(p1(2) - currentTarget_(2), 2);
	}

	NUMBER forceCoeff_;
	NUMBER velocCoeff_;
	Sample* sample_;
	NUMBER currentOutput_;
	Vector3 currentDirForce_;
	Vector3 currentDirVel_;
	Vector3 currentTarget_;
};

SimpleSampleVisitor::SimpleSampleVisitor()
	: SampleGeneratorVisitor_ABC()
	, pImpl_(new PImpl())
{
	//NOTHING
}

SimpleSampleVisitor::~SimpleSampleVisitor()
{
	// NOTHING
}

void SimpleSampleVisitor::Visit(Sample& sample)
{
	NUMBER value = 1 / (pImpl_->FastDistance(sample.GetPosition()));
	if(value > pImpl_->currentOutput_)
	{
		pImpl_->currentOutput_ = value;
		pImpl_->sample_ = &sample;
	}
	/*if(pImpl_->FastDistance(sample.GetPosition()) < 0.05f)
	{
		float value = 0.f; float tmp = 0.f;
		if(pImpl_->forceCoeff_ != 0)
		{
			tmp = sample.forceManipulabiliy(pImpl_->currentDirForce_);
			if (tmp <0) tmp = 0;
			value += pImpl_->forceCoeff_ * tmp;
		}
		if(pImpl_->velocCoeff_ != 0)
		{
			tmp = sample.velocityManipulabiliy(pImpl_->currentDirVel_);
			if (tmp <0) tmp = 0;
			value += pImpl_->velocCoeff_ * tmp;
		}
		if(value > pImpl_->currentOutput_)
		{
			pImpl_->currentOutput_ = value;
			pImpl_->sample_ = &sample;
		}
	}*/
}

void SimpleSampleVisitor::Configure(float force, float velocity)
{
	pImpl_->forceCoeff_ = force;
	pImpl_->velocCoeff_ = velocity;
}

Sample* SimpleSampleVisitor::Run(const SampleGenerator& generator, const matrices::Vector3& directionForce, const matrices::Vector3& directionVel, const matrices::Vector3& target)
{
	pImpl_->Reset();
	pImpl_->currentDirForce_ = directionForce;
	pImpl_->currentDirVel_   = directionVel;
	pImpl_->currentTarget_   = target;
	//generator.Request(0, *this);
	return pImpl_->sample_;
}
