#ifndef _XBOX_CONTROLLER_H_
#define _XBOX_CONTROLLER_H_

// We need the Windows Header and the XInput Header
#include <memory>

#include "Exports.h"

namespace xbox
{

struct PImpl;

namespace Buttons
{
enum Buttons
{ 
	DPadUp = 0x00001,
	DPadDown = 0x00002,
	DpadLeft = 0x00004,
	DPadRight = 0x00008,
	Start = 0x00010,
	Back = 0x00020,
	LeftThumb = 0x00040,
	RightThumb = 0x00080,
	LeftShoulder = 0x00100,
	RightShoulder = 0x00200,
	A = 0x01000,
	B = 0x02000,
	X = 0x04000,
	Y = 0x08000,
	LeftTrigger  = 0x10000,
	RightTrigger = 0x20000
};
} // end namespace Buttons

// XBOX Controller Class Definition
class XBOXCONTROLLER_API Controller
{
public:
	Controller(int playerNumber);
	
	/*Requests*/
	bool IsConnected() const;	
	/*Checks leftstick first*/
	void GetLeftStickMovingVector(float& /*x*/, float& /*y*/) const;	
	/*Checks rightstick*/
	void GetRightStickMovingVector(float& /*x*/, float& /*y*/) const;
	// true if above treshold
	bool GetRightTriggerPressure(float& /*x*/) const;
	bool GetLeftTriggerPressure(float& /*x*/) const;
	/*true if combinaison of all buttons are pressed*/
	/*bool AllTriggered(const int button) const;*/ // this one is never going to happen
	bool AllPressed(const int button) const;
	/*bool AllReleased(const int button) const; */ // this one is never going to happen

	/*true if any of button combinaison is working*/
	bool AnyTriggered(const int button) const;
	bool AnyPressed(const int button) const;
	/*true only if all buttons were pressed*/
	bool AnyReleased(const int button) const;

	/*none of the buttons indicated are pressed*/
	bool NonePressed(const int button) const;
	/*No button at all is pressed*/
	bool NonePressed() const;
	bool AnyPressed() const;

	/*0 if not pressed, else says for how long*/
	float ButtonPressedTime(const Buttons::Buttons /*button*/) const;

	/*0 means no vibration, 1 is max*/
	int Vibrate(const float /*leftVal*/, const float /*rightVal*/);

	void Update(float gameTime = 1.f);
private:
	std::auto_ptr<PImpl> pImpl_;
};

} // end namespace xbox
#endif