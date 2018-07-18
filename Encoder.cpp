//
//
//

#include "Encoder.h"

Encoder::Encoder(int pinA, int pinB, int pinS, int bounceTime){
	_a = pinA;
	_b = pinB;
	_s = pinS;
	_bounceTime = bounceTime;
}

void Encoder::init(){
	pinMode(_a, INPUT_PULLUP);
	pinMode(_b, INPUT_PULLUP);
	pinMode(_s, INPUT_PULLUP);

	attachInterrupt(digitalPinToInterrupt(_a), pinA, CHANGE);
	attachInterrupt(digitalPinToInterrupt(_b), pinB, CHANGE);
	attachInterrupt(digitalPinToInterrupt(_s), select, FALLING);
}
