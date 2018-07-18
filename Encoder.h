// Encoder.h

// This class written for a mechanical rotary encoder with momentary push-button

#ifndef _ENCODER_h
#define _ENCODER_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif
#endif

class Encoder{
public:
	Encoder(int pinA, int pinB, int pinS, int bounceTime);
	void init();
	int get();

private:
	int _a;
	int _b;
	int _s;
	int _count;
	int _bounceTime;
	long _lastTime;
	void pinA();
	void pinB();
	void select();
	void increment(int dir);
}
