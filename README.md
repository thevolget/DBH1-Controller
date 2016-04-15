# DBH1-Controller
Arduino Library for the DBH-1 Series Motor Driver
Written and debugged by David Williams (Thevolget). 
All rights reserved.

This library is designed to be used with the DBH-1 Series of motor drivers that 
can be found on EBay or Aliexpress (usually under the name of 'wingxine').  

This library will assume the following pins are used by default:
	
	ENA - Pin 2
	ENB - Pin 4
	IN1A - Pin 3 (Input for motor A - needs to be PWM pin)
	IN1B - Pin 5 (Input for motor B - needs to be PWM pin)
	IN2A - Pin 6 (Input for motor A - needs to be PWM pin)
	IN2B - Pin 9 (Input for motor B - needs to be PWM pin)
	CTA - Analog Pin 0 (Optional)
	CTB - Analog Pin 1 (Optional)
	
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
		DBH1.Braking();							UNTESTED - USE WITH CAUTION
		DBH1.Coasting();						Disables motor output
		DBH1.ForwardA(PWM value);					Moves Motor A or B at X% speed
		DBH1.ReverseA(PWM value);					Forward or Reverse
		DBH1.ForwardB(PWM value);
		DBH1.ReverseB(PWM value);
		DBH1.DisableA();
		DBH1.DisableB();
		DBH1.BrakeA();							UNTESTED - USE WITH CAUTION
		DBH1.BrakeB();							UNTESTED - USE WITH CAUTION
		
		
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
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

