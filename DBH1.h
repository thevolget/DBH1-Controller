//Ensure only loaded once
#ifndef DBH1_h
#define DBH1_h

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

class DBH1{
	public:
		void init(byte _IN1A, byte _IN1B, byte _IN2A, byte _IN2B, byte _ENA, byte _ENB, byte _CTA, byte _CTB);
		void Forward(int Apwm, int Bpwm);
		void Reverse(int Apwm, int Bpwm);
		void Braking();
		void Coasting();
		int GetCurrent(int AnalogPin);
		void ForwardA(int Apwm);
		void ForwardB(int Bpwm);
		void ReverseA(int Apwm);
		void ReverseB(int Bpwm);
		void DisableA();
		void DisableB();
		void EnableA();
		void EnableB();
		void EnableBoth();
		void DisableBoth();
};

#endif		