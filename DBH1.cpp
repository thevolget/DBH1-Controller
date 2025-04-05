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
	the microcontroller.  The driver outputs the amp draw of the motor.  If the Arduino or
	ESP board is operating at 3.3V, the BoardVoltage value in DBH1.h should be modified 
	to the respective voltage.  Use of these pins is optional and only necessary for monitoring
	current draw.
	
	EN1 and EN2 are pins for enabling the driver's output to the respective motor.  The motors must be enabled
	prior to any forward / reverse movement.  
	
	IN1A and IN2A (as well as IN2A and IN2B) are the inputs for motor A (and B 
	respectively).  When IN1A is driven LOW and IN2A is	provided a PWM signal, the 
	motor will go in reverse.  When IN1A is provided a PWM signal and IN2A is driven 
	LOW, the motor will go forward.

	The driver requires a maximum duty cycle on the PWM input of no more than 98%.  
	Any higher might damage the driver / result in instability.
	
	Braking mode places all pins HIGH, which uses a short-circuit method to stop motors.
	Please verify with hardware and driver's datasheet before using, as this may damage 
	boards if used improperly.
	
Library usage:

	Option 1:
		DBH1.init();	This will use the default pins
	
	Option 2:
		DBH1.init(IN1A, IN1B, IN2A, IN2B, ENA, ENB, CTA, CTB);	  This will define which
			pins are used on the microcontroller.  CTA and CTB pins are optional and only
			needed to use the GetCurrentA() and GetCurrentB() functions.  This is using the 
			current scaling of 0.155V/A.
	
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
		DBH1.ToggleBoth();
		DBH1.EnableBoth();										Enables both motors
		DBH1.GetCurrentA();										Returns motor current draw using default pin(30A max)
		DBH1.GetCurrentB();										
		DBH1.GetCurrentA(pin);									Returns motor current draw using specified analog pin(30A max)
		DBH1.GetCurrentB(pin);									
		DBH1.EnableStatusA();									Returns the current motor enable status
		DBH1.EnableStatusB();
		DBH1.SetBoardVoltage(voltage)							Sets the reference voltage to calculate current draw.  5V for most boards
																such as Arduino, unless your board specifies otherwise
		
	/*
	DBH1 motor;
	void setup() {
		motor.init();  							// Default pins
		motor.ToggleBoth();						// Enable motors
		motor.Forward(200, 200);  				// 78% speed forward
		float current = motor.GetCurrentA();	// Read motor current and output in Amps
	}
	
	Example usage with custom pins:
	
	DBH1 motor;
	void setup() {
		motor.init(4,7,2,8,3,6,A3,A5);  		// Custom Pins (example only, motor input pins must be PWM capable.
		motor.ToggleBoth();						// Toggle state of motors to enable
		motor.Forward(200, 200);  				// 78% speed forward
		float current = motor.GetCurrentA();	// Read motor current and output in Amps
	}
	
	Example of return status checking.  This does not factor in any race conditions or conditions where one motor might be
	disabled.  Both motors should be disabled on initialization to prevent accidental startup.
	
	DBH1 motor;
	void setup() {
		motor.init();  		
			if (motor.EnableStatusA() == false && motor.EnableStatusB() == false) {
				motor.EnableBoth();
				Serial.println("Motors were disabled, now enabled.");
			}
		motor.Forward(200, 200);  				// 78% speed forward.  Motor must be enabled prior to use.
		float current = motor.GetCurrentA();	// Read motor current and output in Amps
	}
	void loop() {
												// Monitor motor states (optional)
		Serial.print("Motor A Enabled: "); Serial.println(motor.EnableStatusA());
		Serial.print("Motor B Enabled: "); Serial.println(motor.EnableStatusB());
		delay(1000);
	}
	
	Single Motor Control
	
	DBH1 motor;
	void setup() {
		motor.init();  		
			if (motor.EnableStatusA() == false) {
				motor.ToggleA();
				Serial.println("Motor A was disabled, now enabled.");
			}
		motor.Forward(200);		  				// 78% speed forward
		float current = motor.GetCurrentA();	// Read motor current and output in Amps
	}
	
	Motor Braking (Please see warnings all over documentation regarding)
	
	DBH1 motor;
	void setup() {
		...Your code here...
		motor.init();
  		... Your code here...
			if (motor.EnableStatusA() == true && EmerStop == true) {
				motor.Braking();				// Uses short-circuit method to cause motor to brake.  May damage motor driver if used incorrectly
				Serial.println("Brake enabled");
			}
		float current = motor.GetCurrentA();	// Read motor current and output in Amps
	}
	
	Motor Coasting (Please see warnings all over documentation regarding)
	
	DBH1 motor;
	void setup() {
		...Your code here...
		motor.init();
  		... Your code here...
			if (motor.EnableStatusA() == true && EmerStop == false) {
				motor.Coasting();				// Sets all pins LOW to allow motor to free spin.
				Serial.println("Coast enabled");
			}
		float current = motor.GetCurrentA();	// Read motor current and output in Amps
	}
	
	*/

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
	this->EnableA = false;  						// Explicitly reset to false
    this->EnableB = false;
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
	_Apwm = constrain(abs(_Apwm), 0, MAX_PWM);  		// 250/255 ≈ 98% of 255
	_Bpwm = constrain(abs(_Bpwm), 0, MAX_PWM);  		// 250/255 ≈ 98% of 255
	analogWrite(IN1A, _Apwm);						//PWM to 1st set of inputs
	analogWrite(IN1B, _Bpwm);
	DetectDirection(0);
}

void DBH1::Reverse(int _Apwm, int _Bpwm){
	_Apwm = constrain(abs(_Apwm), 0, MAX_PWM);  		// 250/255 ≈ 98% of 255
	_Bpwm = constrain(abs(_Bpwm), 0, MAX_PWM);  		// 250/255 ≈ 98% of 255
	analogWrite(IN2A, _Apwm);						//PWM to 2nd set of inputs
	analogWrite(IN2B, _Bpwm);
	DetectDirection(1);
}

void DBH1::Braking(){								//Sets all pins HIGH to enable brake
	#pragma message "Untested - This function has not been fully tested, use at own risk"
	digitalWrite(ENA, HIGH);						//WARNING: Verify that your motor driver will support this
	digitalWrite(ENB, HIGH);             			//method, as it sets all pins HIGH (short-circuit braking)
	digitalWrite(IN1A, HIGH);						//Different drivers may require specific braking outputs.
	digitalWrite(IN1B, HIGH);
  	digitalWrite(IN2A, HIGH);
	digitalWrite(IN2B, HIGH);      		
}

void DBH1::Coasting(){								//Disables motor and allows for free roll
	digitalWrite(IN1A, LOW);			
	digitalWrite(IN1B, LOW);
	digitalWrite(IN2A, LOW);
	digitalWrite(IN2B, LOW);
  	digitalWrite(ENA, LOW);
	digitalWrite(ENB, LOW);
}

float DBH1::GetCurrentA() {		
    return GetCurrentA(CTA);
}

float DBH1::GetCurrentB() {
	return GetCurrentB(CTB);
}

float DBH1::GetCurrentA(byte AnalogPinA) {			 
	int rawValue = analogRead(AnalogPinA);			//Convert ADC reading to 0-5V then apply scaling to convert to Amp
	return (rawValue * (BoardVoltage / 1023.0f)) / 0.155f;		// 0.155V/A scaling factor (verify with driver's datasheet)
}

float DBH1::GetCurrentB(byte AnalogPinB) {	
	int rawValue = analogRead(AnalogPinB);			//Convert ADC reading to 0-5V then apply scaling to convert to Amp
	return (rawValue * (BoardVoltage / 1023.0f)) / 0.155f;		// 0.155V/A scaling factor (verify with driver's datasheet)
}

void DBH1::ForwardA(int _Apwm){						//Individual Motor A Control Forward
	_Apwm = constrain(abs(_Apwm), 0, MAX_PWM);  		
	analogWrite(IN1A, _Apwm);
	digitalWrite(IN2A, LOW);
}

void DBH1::ForwardB(int _Bpwm){						//Individual Motor B Control Forward
	_Bpwm = constrain(abs(_Bpwm), 0, MAX_PWM);  		
	analogWrite(IN1B, _Bpwm);
	digitalWrite(IN2B, LOW);
}

void DBH1::ReverseA(int _Apwm){						//Individual Motor A Control Reverse
	_Apwm = constrain(abs(_Apwm), 0, MAX_PWM);  		
	analogWrite(IN2A, _Apwm);
	digitalWrite(IN1A, LOW);
}

void DBH1::ReverseB(int _Bpwm){						//Individual Motor B Control Reverse
	_Bpwm = constrain(abs(_Bpwm), 0, MAX_PWM);  	
	analogWrite(IN2B, _Bpwm);
	digitalWrite(IN1B, LOW);
}

void DBH1::DisableA(){								//Disables Motor A Function
	EnableA = false;
	digitalWrite(ENA, EnableA);
}

void DBH1::DisableB(){								//Disables Motor B Function
	EnableB = false;
	digitalWrite(ENB, EnableB);
}

void DBH1::ToggleA(){								//Toggles Motor A
	EnableA = !EnableA;              				// Flips the state (1→0 or 0→1)
    digitalWrite(ENA, EnableA);
}

void DBH1::ToggleB(){								//Toggles Motor B
	EnableB = !EnableB;              				
    digitalWrite(ENB, EnableB);
}

void DBH1::ToggleBoth(){							//Enable Both Motors
	EnableA = !EnableA;
    EnableB = !EnableB;
    digitalWrite(ENA, EnableA);
    digitalWrite(ENB, EnableB);
}

void DBH1::DisableBoth(){							//Disables Both Motors
	EnableA = false;
	EnableB = false;
	digitalWrite(ENA, EnableA);
    digitalWrite(ENB, EnableB);
}

bool DBH1::EnableStatusA(){							//Returns if motor is enabled
	return EnableA;
}

bool DBH1::EnableStatusB(){							//Returns if motor is enabled
	return EnableB;
}

void EnableBoth() {
    EnableA = EnableB = true;
    digitalWrite(ENA, HIGH);
    digitalWrite(ENB, HIGH);
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
}