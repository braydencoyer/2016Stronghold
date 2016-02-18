#include "WPILib.h"

class LimitSwitch
{
	DigitalInput* swtch;
	bool normOpen;
	public:
	LimitSwitch(int DIOChannel, bool normallyOpen) {
		swtch = new DigitalInput(DIOChannel);
		normOpen = normallyOpen;
	}

	//Get value, swap if normally closed
	bool Get()
	{
		if(normOpen) return swtch->Get();
		else return !swtch->Get();
	}

};
