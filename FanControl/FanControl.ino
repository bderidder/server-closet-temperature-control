/*
  MIT License
  Copyright 2020 Bavo De Ridder (bavoderidder.com)

  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation 
  files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, 
  modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the 
  Software is furnished to do so, subject to the following conditions:
  
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
  
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE 
  WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR 
  COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, 
  ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
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
 * Resources for measuring RPM
 *    https://playground.arduino.cc/Main/ReadingRPM/
 *    
 * The code below is tested on an Arduino UNO with the Grove Shield and the Grove Beginner Kit for Arduino
 *    https://www.seeedstudio.com/Grove-Beginner-Kit-for-Arduino-p-4549.html
 */

#include <Arduino.h>
#include "DHT.h"

/*
 * DHT Temperature Sensor constants and variables
 */
const byte DHT_PIN = 4;  // connected on D4
struct DHTReading 
{
   float  temperature;
   float  humidity;
} lastDHTReading;        // struct where the telemetry will store the last reading made

DHT dht(DHT_PIN, DHT22); // our instance of the DHT sensor

/*
 * Tacho telemetry constants and variables
 */
const byte TACHO_PIN               = 2;  // on an Arduino UNO only digital pins 2 and 3 can have ISR's attached (ISR = Interrupt Service Routine)
volatile unsigned long tachoTicks  = 0;  // tacho ticks measured so far in the current sampling window
unsigned long tachoTelemeryTimeOld = 0;  // millis() at wich the last reading was done
unsigned int fanRpm                = 0;  // variable where the telemetry will store the last calculation of RPM

/*
 * Rotary Angle Sensor constants
 * 
 * Note: the rotary angle sensor is a rotary potentiometer
 */
const byte ROTARY_ANGLE_SENSOR_PIN = A0;  // rotary potentiometer is connected to analog pin 0
const int  FULL_ANGLE              = 300; // angle sensor goes from 0 degrees to 300 degrees

/*
 * PWM Timer constants
 */
const word PWM_FREQ_HZ = 25000;                    // Noctua NF-P12 PWM, as most computer fans, prefer 25kHz 
const word TCNT1_TOP   = 16000000/(2*PWM_FREQ_HZ); // 16000000 = the Arduino UNO Timer runs at 16Mhz by default
const byte OC1A_PIN    = 9;                        // Fan PWM pin - we use Timer1 of the Arduino Uno so we have to use analog pins 9 and 10.

/*
 * 
 */
const long LOOP_FREQUENCY = 2;                     // number of times per second we want to run the main loop
const long LOOP_DELAY     = 1000 / LOOP_FREQUENCY; // milliseconds to delay after running the main loop

/*
 * This interrupt service routine ("ISR") is called each time an interrupt is generated.
 * 
 * See method setupTachoTelemetry() for the setup of the interrupt.
 * 
 * This method is called outside of the main loop and even interrupts it.
 */
void isrTachoTelemetry()
{
  tachoTicks++;
}

/*
 * Initialize the DHT sensor telemetry
 */
void setupDHTSensor() 
{
  lastDHTReading.temperature = 0.0;
  lastDHTReading.humidity = 0.0;

  dht.begin();
}

/*
 * Initialize the tacho telemetry
 */
void setupTachoTelemetry() 
{
  // call "isrTachoTelemetry" method each time TACHO_PIN goes from LOW to HIGH
  attachInterrupt(digitalPinToInterrupt(TACHO_PIN), isrTachoTelemetry, RISING);
}

/*
 * Initialize PWM Timer1 so we can drive the PWM of the computer fan.
 */
void setupPwmTimer() 
{
  pinMode(OC1A_PIN, OUTPUT);

  // Clear Timer 1 control and count registers
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  // Configure Timer 1 for PWM @ configured kHz (see PWM_FREQ_HZ const).
  // See https://sites.google.com/site/qeewiki/books/avr-guide/pwm-on-the-atmega328 for more information
  // on these registers and their values.
  TCCR1A = 0;           // undo the configuration done by...
  TCCR1B = 0;           // ...the Arduino core library
  TCNT1  = 0;           // reset timer
  TCCR1A = _BV(COM1A1)  // non-inverted PWM on ch. A
         | _BV(COM1B1)  // same on ch; B
         | _BV(WGM11);  // mode 10: ph. correct PWM, TOP = ICR1
  TCCR1B = _BV(WGM13)   // ditto
         | _BV(CS10);   // prescaler = 1
  ICR1   = TCNT1_TOP;
}

/*
 * Initialize our rotary angle sensor
 */
void setupRotaryAngleSensor() 
{
  pinMode(ROTARY_ANGLE_SENSOR_PIN, INPUT);
}


/*
 * Bootstrap setup function, from here all other setup functions are called
 */
void setup() 
{
  Serial.begin(115200);
  Serial.print("Starting Fan Control ... ");

  setupDHTSensor();

  setupTachoTelemetry();

  setupPwmTimer();

  setupRotaryAngleSensor();
  
  Serial.println("done");

  // Initialize duty to 0 ensuring fan will not spin after setup is done
  setPwmDuty(0);
}

/*
 * Main method which is called in a loop.
 */
void loop() 
{
  updateFanRpm();

  Serial.print("Fan RPM: ");
  Serial.println(fanRpm);

  updateDHTReading();

  Serial.print("Temperature: ");
  Serial.println(lastDHTReading.temperature);

  long degrees = getAngleOnRotarySensor();

  // Uncomment if you want to debug the rotary angle sensor
  // Serial.print("The angle between the mark and the starting position: ");
  // Serial.println(degrees);

  // Map the rotary angle to duty cycle value between 0% and 100%
  int fanSpeed = map(degrees, 0, FULL_ANGLE, 0, 100);
  
  setPwmDuty(fanSpeed);

  // delay restarting the loop so we get our loop frequency set in LOOP_FREQUENCY
  delay(LOOP_DELAY);
}

/*
 * Update the measured RPM since the last time we updated it.
 * 
 * We take the number of tacho ticks and divide it by the time since we last measured.
 * The result is multiplied by 
 */
void updateFanRpm()
{
  // store the millis() at which we updated our fan RPM measurement
  // remember, millis() returns the number of milliseconds since the Arduino began running this program
  unsigned long tachoTelemeryTimeNew = millis(); 

  // the number of milliseconds we have been counting tacho ticks since we last reset it to 0
  unsigned long samplingPeriod = tachoTelemeryTimeNew - tachoTelemeryTimeOld;

  // Time is measured in milliseconds, multiply by 1000 and 60 to get 1 minute, divide by sampling period
  float tachoTicksInOneMinute = 60.0 * 1000.0 / samplingPeriod * tachoTicks;

  // we get two tach ticks per revolution so we have to divide by 2 to get RPM
  fanRpm = tachoTicksInOneMinute / 2;

  // start a new sampling period
  tachoTelemeryTimeOld = tachoTelemeryTimeNew;
  tachoTicks = 0;
}

/*
 * Update the last DHT reading
 */
void updateDHTReading() 
{
  float temp_hum_val[2] = {0};

  if (!dht.readTempAndHumidity(temp_hum_val)) 
  {
        lastDHTReading.humidity = temp_hum_val[0];
        lastDHTReading.temperature = temp_hum_val[1];
  } 
  else 
  {
      Serial.println("Failed to get temprature and humidity value.");
  }
}

/*
 * Return, in degrees, the current setting of the rotary sensor.
 * 
 * Value will between 0 and the FULL_ANGLE constant set above.
 */
long getAngleOnRotarySensor() 
{
  // Values between 0V and 5V (on Arduino) are read as values between 0 and 1023.
  int rotaryPinValue = analogRead(ROTARY_ANGLE_SENSOR_PIN);

  // Map the 0 to 1023 range to angle 0 degrees to FULL_ANGLE degrees
  // The map() functions works with integer so we loose fractions but that does not matter for us.
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
void setPwmDuty(byte duty) 
{
  if (duty > 100) duty = 100;
  if (duty < 0) duty = 0;
  
  Serial.print("Duty of ");
  Serial.print(duty);
  
  OCR1A = (word) (duty*TCNT1_TOP)/100;
  
  Serial.print(" requested, setting OCR1A to ");
  Serial.println(OCR1A);
}
