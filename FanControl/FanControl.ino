/*
MIT License
Copyright 2020 Michael Schoeffler (mschoeffler.de)
Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/*
 * Fan used: Noctua NF-P12 PWM
 *    https://noctua.at/pub/media/wysiwyg/Noctua_PWM_specifications_white_paper.pdf
 *    
 * Resources on how to adjust the PWM timers on Arduino UNO
 *    https://create.arduino.cc/projecthub/tylerpeppy/25-khz-4-pin-pwm-fan-control-with-arduino-uno-3005a1
 *    https://arduino.stackexchange.com/questions/25609/set-pwm-frequency-to-25-khz
 *    
 * Technical specification of the PWM timers on Arduino (scroll down to "TIMER1 (16BIT PWM)"
 *    https://sites.google.com/site/qeewiki/books/avr-guide/pwm-on-the-atmega328
 *    
 * Resources for rotary angle sensor
 *    https://wiki.seeedstudio.com/Grove-Rotary_Angle_Sensor/
 *    
 * The code below is tested on and uses the builtin sensors/actuators/... of the Grove Beginner Kit for Arduino
 *    https://www.seeedstudio.com/Grove-Beginner-Kit-for-Arduino-p-4549.html
 */

#include <Arduino.h>
#include <Wire.h>     // required to use I2C protocol to drive the OLED display

/*
 * 
 */
const byte TACHO_PIN = 2; // on an Arduino UNO only digital pins 2 and 3 can have ISR's attached (ISR = Interrupt Service Routine)

/*
 * Rotary Angle Sensor constants
 * 
 * Note: the rotary angle sensor is a rotary potentiometer
 */
const byte ROTARY_ANGLE_SENSOR_PIN = A0;
const int  FULL_ANGLE              = 300; // angle sensor goes from 0 degrees to 300 degrees

/*
 * PWM Timer constants
 */
const word PWM_FREQ_HZ = 25000; // Noctua NF-P12 PWM, as most computer fans, prefer 25kHz 
const word TCNT1_TOP   = 16000000/(2*PWM_FREQ_HZ);
const byte OC1A_PIN    = 9;     // Fan PWM pin - we use Timer1 of the Arduino Uno so we have to use analog pins 9 and 10.


volatile unsigned long tachoTicks = 0;
unsigned int fanRpm = 0;
unsigned long tachoTelemeryTimeOld = 0;

void isrTachoTelemetry() {

  tachoTicks++;
}

void setupTachoTelemetry() {

  //pinMode(TACHO_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(TACHO_PIN), isrTachoTelemetry, RISING);
}

/*
 * Setup function called once to initialize PWM Timer1
 */
void setupPwmTimer() {

  pinMode(OC1A_PIN, OUTPUT);

  // Clear Timer 1 control and count registers
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  // Configure Timer 1 for PWM @ configured kHz (see PWM_FREQ_HZ const).
  TCCR1A = 0;           // undo the configuration done by...
  TCCR1B = 0;           // ...the Arduino core library
  TCNT1  = 0;           // reset timer
  TCCR1A = _BV(COM1A1)  // non-inverted PWM on ch. A
         | _BV(COM1B1)  // same on ch; B
         | _BV(WGM11);  // mode 10: ph. correct PWM, TOP = ICR1
  TCCR1B = _BV(WGM13)   // ditto
         | _BV(CS10);   // prescaler = 1
  ICR1   = TCNT1_TOP;   // TOP =
}

/*
 * Setup function called once to initialize our rotary angle sensor
 */
void setupRotaryAngleSensor() {

  pinMode(ROTARY_ANGLE_SENSOR_PIN, INPUT);
}


/*
 * Bootstrap setup function, from here all other setup functions are called
 */
void setup() {

  Serial.begin(115200);
  Serial.print("Starting Fan Control ... ");

  setupTachoTelemetry();

  setupPwmTimer();

  setupRotaryAngleSensor();
  
  Serial.println("done");

  /*
   * Initialize duty to 0 ensuring fan will not spin after setup is done
   */
  setPwmDuty(0);
}

void loop() {

  updateFanRpm();

  Serial.print("Fan RPM: ");
  Serial.println(fanRpm);

  long degrees = getAngleOnRotarySensor();
  
  Serial.print("The angle between the mark and the starting position: ");
  Serial.println(degrees);

  /*
   * Map the rotary angle to duty cycle value between 0% and 100%
   */
  int fanSpeed = map(degrees, 0, FULL_ANGLE, 0, 100);
  
  setPwmDuty(fanSpeed);
  
  delay(500);
}

void updateFanRpm() {

  fanRpm = 30*1000/(millis() - tachoTelemeryTimeOld) * tachoTicks;
  
  tachoTelemeryTimeOld = millis();
  tachoTicks = 0;
}

/*
 * Return, in degrees, the current setting of the rotary sensor.
 * 
 * Value will between 0 and the FULL_ANGLE constant set above.
 */
long getAngleOnRotarySensor() {

  /*
   * Values between 0V and 5V (on Arduino) are read as values between 0 and 1023.
   */
  int rotaryPinValue = analogRead(ROTARY_ANGLE_SENSOR_PIN);

  /*
   * Map the 0 to 1023 range to angle 0 degrees to FULL_ANGLE degrees
   * The map() functions works with integer so we loose fractions but that does not matter for us.
   */
  return map(rotaryPinValue, 0, 1023, 0, FULL_ANGLE);
}


/*
 * Duty from 0 to 100 interpreted as the % of TCNT1_TOP.
 * 
 * For PWM @25kHz this means O% to 100% equals OCR1A from 0 to 320.
 * 
 * 0   = 0V (or always LOW)
 * 100 = 5V (or always HIGH)
 */
void setPwmDuty(byte duty) {

  if (duty > 100) duty = 100;
  if (duty < 0) duty = 0;
  
  Serial.print("Duty of ");
  Serial.print(duty);
  
  OCR1A = (word) (duty*TCNT1_TOP)/100;
  
  Serial.print(" requested, setting OCR1A to ");
  Serial.println(OCR1A);
}
