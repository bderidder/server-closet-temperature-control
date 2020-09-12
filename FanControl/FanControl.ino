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
 * Resources for OLED display
 *    https://wiki.seeedstudio.com/Grove-OLED-Display-0.96-SSD1315/
 */

#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>

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
const byte OC1A_PIN    = 9;     // Fan PWM pin - We use Timer1 of the Arduino Uno so we have to use analog pins 9 and 10.
const byte OC1B_PIN    = 10;    // Fan tacho pin - We use Timer1 of the Arduino Uno so we have to use analog pins 9 and 10.

/*
 * OLED display instance, connected on I2C
 */
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);

/*
 * Setup function called once to initialize PWM Timer1
 */
void setupPwmTimer() {

  pinMode(OC1A_PIN, OUTPUT);
  pinMode(OC1B_PIN, OUTPUT);

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
 * Setup function called once to initialize OLED display
 */
void setupOled() {

  u8g2.begin();
}

/*
 * Bootstrap setup function, from here all other setup functions are called
 */
void setup() {

  Serial.begin(115200);
  Serial.print("Starting Fan Control ... ");

  setupPwmTimer();

  setupRotaryAngleSensor();

  setupOled();
  
  Serial.println("done");

  /*
   * Initialize duty to 0 ensuring fan will not spin after setup is done
   */
  setPwmDuty(0);
}

void loop() {

  long degrees = getAngleOnRotarySensor();
  
  Serial.print("The angle between the mark and the starting position: ");
  Serial.println(degrees);

  /*
   * Map the rotary angle to duty cycle value between 0 and 100 
   */
  int fan_speed = map(degrees, 0, FULL_ANGLE, 0, 100);
  
  setPwmDuty(fan_speed);
  
  delay(500);
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

void drawDutyOnOled(byte duty) {

  /*
   * Translate a numeric value into a string so we can show it on the OLED
   */
  char snum[5];
  itoa(duty, snum, 10);

  u8g2.clearBuffer();                   // clear the internal memory
  u8g2.setFont(u8g2_font_ncenB08_tr);   // choose a suitable font
  u8g2.drawStr(0,10,snum);              // write something to the internal memory
  u8g2.sendBuffer();
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

  drawDutyOnOled(duty);
}
