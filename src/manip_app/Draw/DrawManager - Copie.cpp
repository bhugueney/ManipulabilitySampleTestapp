
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

#if PROFILE
	#include "Simulation.h"
	#include "glutfonts.h"
	#include <drawstuff/drawstuff.h> // The drawing library for ODE;
	#define MAXSAMPLES 100
	struct FPS : public TimerHandled_ABC
	{

	/* need to zero out the ticklist array before starting */
	/* average will ramp up until the buffer is full */
	/* returns average ticks per frame over the MAXSAMPPLES last frames */
	FPS()
		: tickindex(0), ticksum(0), fps_("")
	{
		// NOTHING
		for(int i=0; i<MAXSAMPLES; ++i)
		{
			ticklist[i] = 0;
		}
		Simulation::GetInstance()->timerHandler_.Register(this);
	}

	~FPS()
	{
		// NOTHING
	}

	virtual void Update(const Timer::t_time time, const Timer::t_time dt)
	{
		ticksum-=ticklist[tickindex];  /* subtract value falling off */
		ticksum+=dt;              /* add new value */
		ticklist[tickindex]=dt;   /* save new value so it can be subtracted later */
		if(++tickindex==MAXSAMPLES)    /* inc buffer index */
			tickindex=0;

		/* return average */
		stringstream convert;
		convert << ((double)ticksum/MAXSAMPLES);
		fps_ = convert.str();
	}

	void Draw()
	{
		const char * res = fps_.c_str();
		print_bitmap_string(res);
	}
	string fps_;
	int tickindex;
	int ticksum;
	int ticklist[MAXSAMPLES];
	};
#endif //PROFILE

//TODO listener in case of adding new trees
DrawManager::DrawManager(manip_core::ManipManager& manager)
	: drawWorld_(manager)
	, drawShadowPostures_(false)
	, transparencyOn_(false)
{
	manager.GetPostureManager()->RegisterPostureCreatedListenerI(&drawPostures_);
#if PROFILE
	fps_ = new FPS();
#endif //PROFILE
}


DrawManager::~DrawManager()
{
#if PROFILE
	delete fps_;
#endif //PROFILE
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
#if PROFILE

#endif //PROFILE
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

