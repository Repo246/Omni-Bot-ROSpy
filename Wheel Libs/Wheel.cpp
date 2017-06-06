/* 	Wheel.cpp and Wheel.h are cut down versions of the original MotorWheel libraries
*	provided by Nexus Robots. These files are not to be used without the express permission of 
*	Nexus robots and are not offered here for use in any way, shape, or form. 
*
*	For more info on the code used Contact Nexus Robots --> http://www.nexusrobot.com/ 
*	
*	Wheel cpp file
*/

#include <Wheel.h>


Wheel::Wheel(uint8_t _pinPWM,uint8_t _pinDir,
		uint8_t _pinIRQ,uint8_t _pinIRQB,
		PCIntvoidFuncPtr _interrupt)
	{
	

	pinPWM = _pinPWM;
	pinDir = _pinDir;
	pinIRQ = _pinIRQ;
	pinIRQB = _pinIRQB;
	interrupt = _interrupt;
	
	pinMode(pinPWM,OUTPUT);
	pinMode(pinDir,OUTPUT);
	pinMode(pinIRQ,INPUT);
	
	pulses = 0;
	previous_pulse = 0;
	
	
	if(pinIRQB!=PIN_UNDEFINED) {
		pinMode(pinIRQB,INPUT);
	}
	
	setupInterrupt();
}
	
void Wheel::setupInterrupt() {

	if(pinIRQ==2 || pinIRQ==3) attachInterrupt(pinIRQ-2,interrupt,TRIGGER);
	else {
		PCattachInterrupt(pinIRQ,interrupt,TRIGGER);	// RISING --> CHANGE 201207
	}
}

uint8_t Wheel::runPWM(uint8_t PWM,bool dir,bool saveDir) {
	speedPWM=PWM;
	if(saveDir) savedDirection=dir;
	analogWrite(pinPWM,PWM);
	digitalWrite(pinDir,dir);
	return PWM;
}

int16_t Wheel::getSpeedPPS()  {
	if (previous_pulse == pulses)
	{
		return 0;
	}
	previous_pulse = pulses;
	return speedPPS;
}

int32_t Wheel::getCurrPulse() const {
	return pulses;
}
int32_t Wheel::setCurrPulse(int32_t _pulse) {
	previous_pulse = _pulse;
	pulses=_pulse;
	return getCurrPulse();
}
int32_t Wheel::resetCurrPulse() {
	return setCurrPulse(0);
}
	


