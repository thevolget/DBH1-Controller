/* 
DBH-1 Motor Driver Library
Written and debugged by David Williams (Thevolget). 
All rights reserved.

This library is designed to be used with the DBH-1 Series of motor drivers that 
can be found on EBay or Aliexpress (usually under the name of 'wingxine').  

This library will assume the following pins are used by default:

	ENA 	- Pin 2
	ENB 	- Pin 4
	IN1A 	- Pin 3 (Input for motor A - needs to be PWM pin)
	IN1B 	- Pin 5 (Input for motor B - needs to be PWM pin)
	IN2A 	- Pin 6 (Input for motor A - needs to be PWM pin)
	IN2B 	- Pin 9 (Input for motor B - needs to be PWM pin)
	CTA 	- Analog Pin 0 (Optional)
	CTB 	- Analog Pin 1 (Optional)
	
	Pins CTA and CTB are pins for reporting the current draw of the driver back to 
	the microcontroller.  The driver outputs a analog value based on current draw.
	
	EN1 and EN2 are pins for enabling the driver's output to the respective motor.
	
	IN1A and IN2A (as well as IN2A and IN2B) are the inputs for motor A (and B 
	respectively).  When IN1A is driven LOW and IN2A is	provided a PWM signal, the 
	motor will go in reverse.  When IN1A is provided a PWM signal and IN2A is driven 
	LOW, the motor will go forward.

	The driver requires a maximum duty cycle on the PWM input of no more than 98%.  
	Any higher might damage the driver / result in instability.
	
Library usage:

	Option 1:
		DBH1.init();	This will use the default pins
	
	Option 2:
		DBH1.init(IN1A, IN1B, IN2A, IN2B, ENA, ENB, CTA, CTB);	  This will define which
			pins are used on the microcontroller.  
	
	Once defined the motor can be controlled by using:
		DBH1.Forward(Motor A PWM value, Motor B PWM value);		Moves both motors forward
		DBH1.Reverse(Motor A PWM value, Motor B PWM value);		Moves both motors reverse
		DBH1.Braking();											UNTESTED - USE WITH CAUTION
		DBH1.Coasting();										Disables motor output
		DBH1.ForwardA(PWM value);								Moves Motor A or B at X% speed
		DBH1.ReverseA(PWM value);								Forward or Reverse
		DBH1.ForwardB(PWM value);
		DBH1.ReverseB(PWM value);
		DBH1.ToggleA();											Toggle Motor Enable state (if off, on - if on, off)
		DBH1.ToggleB();
		DBH1.GetCurrentA();										Returns motor current draw (50A max)
		DBH1.GetCurrentB();										Returns motor current draw (50A max)
		
		
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in
  the documentation and/or other materials provided with the
  distribution.

* Neither the name of the copyright holders nor the names of
  contributors may be used to endorse or promote products derived
  from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, LIFE, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.  
*/

#include "DBH1.h"

													//Following defines optional parameters that can be passed

void DBH1::init(byte IN1A, byte IN1B, byte IN2A, byte IN2B, byte ENA, byte ENB, byte CTA, byte CTB){
    this->IN1A = IN1A;								//Define variables
    this->IN1B = IN1B;
    this->IN2A = IN2A;
    this->IN2B = IN2B;
    this->ENA = ENA;
    this->ENB = ENB;
    this->CTA = CTA;
    this->CTB = CTB;
    pinMode(this->IN1A, OUTPUT);					//Initalize Pins
    pinMode(this->IN1B, OUTPUT);
    pinMode(this->IN2A, OUTPUT);
    pinMode(this->IN2B, OUTPUT);
    pinMode(this->ENA, OUTPUT);
    pinMode(this->ENB, OUTPUT);
    pinMode(this->CTA, INPUT);
    pinMode(this->CTB, INPUT);
}

void DBH1::Forward(int _Apwm, int _Bpwm){
	_Apwm = constrain(abs(_Apwm), 0, 250);  		// 250/255 ≈ 98% of 255
	_Bpwm = constrain(abs(_Bpwm), 0, 250);  		// 250/255 ≈ 98% of 255
	analogWrite(IN1A, _Apwm);						//PWM to 1st set of inputs
	analogWrite(IN1B, _Bpwm);
	DetectDirection(0);
}

void DBH1::Reverse(int _Apwm, int _Bpwm){
	_Apwm = constrain(abs(_Apwm), 0, 250);  		// 250/255 ≈ 98% of 255
	_Bpwm = constrain(abs(_Bpwm), 0, 250);  		// 250/255 ≈ 98% of 255
	analogWrite(IN2A, _Apwm);						//PWM to 2nd set of inputs
	analogWrite(IN2B, _Bpwm);
	DetectDirection(1);
}

void DBH1::Braking(){								//Sets all pins HIGH to enable brake
	#pragma message "Untested - Verify hardware supports before testing"
	digitalWrite(ENA, HIGH);			
	digitalWrite(ENB, HIGH);             
	digitalWrite(IN1A, HIGH);
	digitalWrite(IN1B, HIGH);
  	digitalWrite(IN2A, HIGH);
	digitalWrite(IN2B, HIGH);      		
}

void DBH1::Coasting(){								//Disables motor and allows for free roll
	digitalWrite(IN2A, LOW);			
	digitalWrite(IN1B, LOW);
	digitalWrite(IN1A, LOW);
	digitalWrite(IN2B, LOW);
  	digitalWrite(ENA, LOW);
	digitalWrite(ENB, LOW);
}

float DBH1::GetCurrentA() {		
    return analogRead(CTA) * (50.0 / 1023.0); 
}

float DBH1::GetCurrentB() {
    return analogRead(CTB) * (50.0 / 1023.0);
}

float DBH1::GetCurrentA(byte AnalogPinA) {			//Reads current draw from driver and outputs back value from 0 - 1023.  Uses specified pin as input. 
	return analogRead(AnalogPinA) * (50.0 / 1023);
}

float DBH1::GetCurrentB(byte AnalogPinB) {	
	return analogRead(AnalogPinB) * (50.0 / 1023);
}

void DBH1::ForwardA(int _Apwm){						//Individual Motor A Control Forward
	_Apwm = constrain(abs(_Apwm), 0, 250);  		// 250/255 ≈ 98% of 255
	analogWrite(IN1A, _Apwm);
	digitalWrite(IN2A, LOW);
}

void DBH1::ForwardB(int _Bpwm){						//Individual Motor B Control Forward
	_Bpwm = constrain(abs(_Bpwm), 0, 250);  		// 250/255 ≈ 98% of 255
	analogWrite(IN1B, _Bpwm);
	digitalWrite(IN2B, LOW);
}

void DBH1::ReverseA(int _Apwm){
	_Apwm = constrain(abs(_Apwm), 0, 250);  		// 250/255 ≈ 98% of 255
	analogWrite(IN2A, _Apwm);
	digitalWrite(IN1A, LOW);
}

void DBH1::ReverseB(int _Bpwm){
	_Bpwm = constrain(abs(_Bpwm), 0, 250);  		// 250/255 ≈ 98% of 255
	analogWrite(IN2B, _Bpwm);
	digitalWrite(IN1B, LOW);
}

void DBH1::DisableA(){
	digitalWrite(ENA, LOW);
}

void DBH1::DisableB(){
	digitalWrite(ENB, LOW);
}

void DBH1::ToggleA(){
	EnableA = !EnableA;              				// Flips the state (1→0 or 0→1)
    digitalWrite(ENA, EnableA);
}

void DBH1::ToggleB(){
	EnableB = !EnableB;              				// Flips the state (1→0 or 0→1)
    digitalWrite(ENB, EnableB);
}

void DBH1::ToggleBoth(){							//Enable Both Motors
	EnableA = !EnableA;
    EnableB = !EnableB;
    digitalWrite(ENA, EnableA);
    digitalWrite(ENB, EnableB);
}

void DBH1::DisableBoth(){
	EnableA = LOW;
	EnableB = LOW;
	digitalWrite(ENA, EnableA);
    digitalWrite(ENB, EnableB);
}

void DBH1::DetectDirection (int Dirdetect){			//Used to determine motor direction
	switch (Dirdetect){
		case 0:
			digitalWrite(IN2A, LOW);				//If Direction is 0 - Go Forward
			digitalWrite(IN2B, LOW);
			break;
		case 1:
			digitalWrite(IN1A, LOW);				//If Direction is 1 - Go Reverse
			digitalWrite(IN1B, LOW);
			break;
		default:
			break;
	}
	