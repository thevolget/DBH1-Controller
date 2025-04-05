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
		void init(
													// Pin assignments
		byte IN1A = 3,								//Motor controller input IN1(A) - Motor A
		byte IN1B = 5,								//Motor controller input IN1(B) - Motor B
		byte IN2A = 6, 								//Motor controller input IN2(A) - Motor A
		byte IN2B = 9,								//Motor controller input IN2(B) - Motor B
		byte ENA = 2,								//Motor A Enable
		byte ENB = 4,								//Motor B Enable
		byte CTA = A0,  							//Current input from driver for (A) - Analog Input Pin
		byte CTB = A1								//Current input from driver for (B) - Analog Input Pin
    );
		//void init(byte _IN1A, byte _IN1B, byte _IN2A, byte _IN2B, byte _ENA, byte _ENB, byte _CTA, byte _CTB); //Depreciated - Left for testing only
		float GetCurrentA();      					// Returns current in Amps (0-30A)
		float GetCurrentB();     					// Returns current in Amps (0-30A)
		float GetCurrentA(byte AnalogPinA); 		// With specified pin
		float GetCurrentB(byte AnalogPinB);  		// With specified pin
		void Forward(int Apwm, int Bpwm);
		void Reverse(int Apwm, int Bpwm);
		void Braking();								//Sets all pins HIGH and uses short-circuit braking.  Check Datasheet before use as may damage hardware.
		void Coasting();
		void ForwardA(int Apwm);
		void ForwardB(int Bpwm);
		void ReverseA(int Apwm);
		void ReverseB(int Bpwm);
		void DisableA();
		void DisableB();
		void ToggleA();
		void ToggleB();
		void ToggleBoth();
		void DisableBoth();
		void EnableBoth();
		bool EnableStatusA() const;
		bool EnableStatusB() const;
		void SetBoardVoltage(float voltage) { BoardVoltage = voltage; }		//Defines microcontroller operating voltage to determine amp calculation
	private:
		byte IN1A, IN1B, IN2A, IN2B, ENA, ENB;
		byte CTA,CTB;
		float BoardVoltage = 5;	//Voltage of microcontroller, 5V Default (Most Arduino Boards), if board is running on 3.3V, change value to 3.3
		void DetectDirection(int Dirdetect); 		//Direction Detect Helper
													// State variables
		bool EnableA = false;						//Condition for toggle of Motor A - False = Low, True = High
		bool EnableB = false;						//Condition for toggle of Motor B
		static constexpr uint8_t MAX_PWM = 250;  	// 98% duty cycle for PWM - Hardware limit, any higher will cause instability
		
};

#endif		