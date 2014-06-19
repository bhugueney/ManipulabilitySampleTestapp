
#include <windows.h>
#include <XInput.h>

#include "Controller.h"

/*
DPadUp = 0x0001,
DPadDown = 0x0002,
DpadLeft = 0x0004,
DPadRight = 0x0008,
Start = 0x0010,
Back = 0x0020,
LeftThumb = 0x0040,
RightThumb = 0x0080,
LeftShoulder = 0x0100,
RightShoulder = 0x0200,
A = 0x1000,
B = 0x2000,
X = 0x4000,
Y = 0x8000
*/

namespace xbox
{
struct PImpl
{
	explicit PImpl(int playerNumber)
	{
		controllerNum_ = playerNumber - 1;
		ZeroMemory(&oldState_, sizeof(XINPUT_STATE));
	}

	bool IsConnected() const
	{
		XINPUT_STATE state;
		// Zeroise the state
		ZeroMemory(&state, sizeof(XINPUT_STATE));

		// Get the state
		DWORD Result = XInputGetState(controllerNum_, &state);
		return Result == ERROR_SUCCESS;
	}

	void UpdateState(float gameTime)
	{
		oldState_ = newState_;
		// Zeroise the state
		ZeroMemory(&newState_, sizeof(XINPUT_STATE));

		// Get the state
		XInputGetState(controllerNum_, &newState_);
	}

	XINPUT_STATE oldState_;
	XINPUT_STATE newState_;

	int controllerNum_;
}; // end struct PImpl

unsigned int toVibrateRange(const float val)
{
	return (unsigned int)( val * 65535);
}

/*Left thumbstick y-axis value. The value is between -32768 and 32767.*/
void ToStickRange(const short val, float& res)
{
	float rval = (float)val;
	if(val < 0)
	{
		res = rval / 32768.f;
	}
	else
	{
		res = rval / 32767.f;
	}
}
} // end namespace xbox


using namespace xbox;

Controller::Controller(int playerNumber)
{
	pImpl_.reset(new PImpl(playerNumber));
}

bool Controller::IsConnected() const
{
	return pImpl_->IsConnected();
}

int Controller::Vibrate(const float leftVal, const float rightVal)
{
	// Create a Vibraton State
	XINPUT_VIBRATION Vibration;

	// Zeroise the Vibration
	ZeroMemory(&Vibration, sizeof(XINPUT_VIBRATION));

	// Set the Vibration Values
	/*Speed of the left motor. Valid values are in the range 0 to 65,535. Zero signifies no motor use; 65,535 signifies 100 percent motor use.*/
	Vibration.wLeftMotorSpeed = toVibrateRange(leftVal);
	Vibration.wRightMotorSpeed = toVibrateRange(rightVal);

	// Vibrate the controller
	return XInputSetState(pImpl_->controllerNum_, &Vibration);
}

void Controller::GetLeftStickMovingVector(float& x, float& y) const
{
	ToStickRange(pImpl_->newState_.Gamepad.sThumbLX, x);
	ToStickRange(pImpl_->newState_.Gamepad.sThumbLY, y);
}	

void Controller::GetRightStickMovingVector(float& x, float& y) const
{
	ToStickRange(pImpl_->newState_.Gamepad.sThumbRX, x);
	ToStickRange(pImpl_->newState_.Gamepad.sThumbRY, y);
}

bool Controller::GetRightTriggerPressure(float& x) const
{
	x = (float)(pImpl_->newState_.Gamepad.bRightTrigger) / 255.f;
	return pImpl_->newState_.Gamepad.bRightTrigger >= XINPUT_GAMEPAD_TRIGGER_THRESHOLD;
}

bool Controller::GetLeftTriggerPressure(float& x) const
{
	x = (float)(pImpl_->newState_.Gamepad.bLeftTrigger) / 255.f;
	return pImpl_->newState_.Gamepad.bLeftTrigger >= XINPUT_GAMEPAD_TRIGGER_THRESHOLD;
}

// TODO : check trigger value > XINPUT_GAMEPAD_TRIGGER_THRESHOLD

bool AllButtonsOnThiState(const int button, const XINPUT_STATE& state)
{
	// ok, first triggers are not included in buttons, so handle this case first;
	if(button & Buttons::LeftTrigger)
	{
		if(state.Gamepad.bLeftTrigger < XINPUT_GAMEPAD_TRIGGER_THRESHOLD)
		{
			return false;
		}
	}
	if(button & Buttons::RightTrigger)
	{
		if(state.Gamepad.bRightTrigger < XINPUT_GAMEPAD_TRIGGER_THRESHOLD)
		{
			return false;
		}
	}
	return ((button & 0xFFFF) == 0) || ((state.Gamepad.wButtons ^ (button & 0xFFFF)) == 0);
}

bool AnyButtonOnThiState(const int button, const XINPUT_STATE& state)
{
	// ok, first triggers are not included in buttons, so handle this case first;
	if(button & Buttons::LeftTrigger)
	{
		if(state.Gamepad.bLeftTrigger >= XINPUT_GAMEPAD_TRIGGER_THRESHOLD)
		{
			return true;
		}
	}
	if(button & Buttons::RightTrigger)
	{
		if(state.Gamepad.bRightTrigger >= XINPUT_GAMEPAD_TRIGGER_THRESHOLD)
		{
			return true;
		}
	}
	return (state.Gamepad.wButtons & (button & 0xFFFF)) != 0;
}

bool AnyButtonOnThiState(const XINPUT_STATE& state)
{
	int button = ~0;
	// ok, first triggers are not included in buttons, so handle this case first;
	if(button & Buttons::LeftTrigger)
	{
		if(state.Gamepad.bLeftTrigger >= XINPUT_GAMEPAD_TRIGGER_THRESHOLD)
		{
			return true;
		}
	}
	if(button & Buttons::RightTrigger)
	{
		if(state.Gamepad.bRightTrigger >= XINPUT_GAMEPAD_TRIGGER_THRESHOLD)
		{
			return true;
		}
	}
	return ((state.Gamepad.wButtons & (button & 0xFFFF)) != 0);
}


float Controller::ButtonPressedTime(const Buttons::Buttons button) const
{
	// TODO
	return false;
}

bool Controller::AllPressed(const int button) const
{
	return AllButtonsOnThiState(button, pImpl_->oldState_) & AllButtonsOnThiState(button, pImpl_->newState_);
}

bool Controller::AnyTriggered(const int button) const
{
	return AnyButtonOnThiState(button, pImpl_->newState_) &! AnyButtonOnThiState(button, pImpl_->oldState_);
}

bool Controller::AnyReleased(const int button) const
{
	return AllButtonsOnThiState(button, pImpl_->oldState_) &! AllButtonsOnThiState(button, pImpl_->newState_);
}

bool Controller::AnyPressed(const int button) const
{
	return AnyButtonOnThiState(button, pImpl_->oldState_) & AnyButtonOnThiState(button, pImpl_->newState_);
}

bool Controller::NonePressed(const int button) const
{
	return !AnyButtonOnThiState(button, pImpl_->newState_);
}

bool Controller::NonePressed() const
{
	return !AnyButtonOnThiState(pImpl_->newState_);
}

bool Controller::AnyPressed() const
{
	return AnyButtonOnThiState(pImpl_->newState_);
}

void Controller::Update(float gameTime)
{
	pImpl_->UpdateState(gameTime);
}
