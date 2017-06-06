/* 	Wheel.cpp and Wheel.h are cut down versions of the original MotorWheel libraries
*	provided by Nexus Robots. These files are not to be used without the express permission of 
*	Nexus robots and are not offered here for use in any way, shape, or form. 
*
*	For more info on the code used Contact Nexus Robots --> http://www.nexusrobot.com/ 
*
* 	Wheel header file 
*/


// Included libraries
#include <PinChangeInt.h>
#include <Arduino.h>

// Definitions
#define MAX_PWM 255
#define Baudrate 19200
#define PIN_UNDEFINED 255
#define MICROS_PER_SEC 1000000
#define DIR_ADVANCE HIGH
#define DIR_BACKOFF LOW
#define PLS_BUFF_SIZE 20

// Faulhaber motor definitions
#define	TRIGGER RISING	
#define DIR_INVERSE !

// Interrupt Type: RISING --> CHANGE	201207
#define irqISR(y,x) \
    void x() { \
        static bool first_pulse=true; \
        y.pulseEndMicros=micros(); \
        if(first_pulse==false && y.pulseEndMicros>y.pulseStartMicros) { \
            y.speedPPS=MICROS_PER_SEC/(y.pulseEndMicros-y.pulseStartMicros); \
			/* y.accPPSS=(y.speedPPS-y.lastSpeedPPS)*y.speedPPS; */ \
        } else first_pulse=false; \
        y.pulseStartMicros=y.pulseEndMicros; \
		/* y.lastSpeedPPS=y.speedPPS; */\
		if(y.pinIRQB!=PIN_UNDEFINED) \
			y.currDirection=DIR_INVERSE(digitalRead(y.pinIRQ)^digitalRead(y.pinIRQB)); \
		y.currDirection==DIR_ADVANCE?++y.pulses:--y.pulses; \
    } 


class Wheel {
	
public:	
	Wheel(uint8_t _pinPWM,uint8_t _pinDir,
		uint8_t _pinIRQ,uint8_t _pinIRQB,
		PCIntvoidFuncPtr _interrupt);
	
	// Function declarations
	void setupInterrupt();
	uint8_t runPWM(uint8_t PWM,bool dir,bool saveDir=true);
	int16_t getSpeedPPS() ;
	int32_t getCurrPulse() const;
	int32_t setCurrPulse(int32_t _pulse);
	int32_t resetCurrPulse() ;
	bool return_savedDir();
	
	//Field variables
	volatile int32_t pulses;	// direction sensitive
	int32_t previous_pulse;
	volatile uint32_t pulseStartMicros;
	volatile uint32_t pulseEndMicros;
	volatile uint16_t  speedPPS;
	volatile bool currDirection;
	uint8_t pinIRQB;
	uint8_t pinIRQ;	// pinIRQA
	bool savedDirection;	// saved direction
	
private:
	PCIntvoidFuncPtr interrupt;
	
	uint8_t speedPWM;	// current PWM
	uint8_t pinPWM;
	uint8_t pinDir;
	
};	



