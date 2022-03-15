/*
 * Original sourse: https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library
 * This is the Arduino code PAC6985 16 channel servo controller
 * watch the video for details and demo http://youtu.be/y8X9X10Tn1k
 *  * 
 
 * Watch video for this code: 
 * 
 * Related Videos
V5 video of PCA9685 32 Servo with ESP32 with WiFi https://youtu.be/bvqfv-FrrLM
V4 video of PCA9685 32 Servo with ESP32 (no WiFi): https://youtu.be/JFdXB8Za5Os
V3 video of PCA9685 how to control 32 Servo motors https://youtu.be/6P21wG7N6t4
V2 Video of PCA9685 3 different ways to control Servo motors: https://youtu.be/bal2STaoQ1M
V1 Video introduction to PCA9685 to control 16 Servo  https://youtu.be/y8X9X10Tn1k

 * Written by Ahmad Shamshiri for Robojax Video channel www.Robojax.com
 * Date: Dec 16, 2017, in Ajax, Ontario, Canada
 * Permission granted to share this code given that this
 * note is kept with the code.
 * Disclaimer: this code is "AS IS" and for educational purpose only.
 * this code has been downloaded from http://robojax.com/learn/arduino/
 * 
 */
/*************************************************** 
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 16 servos, one after the other

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815

  These displays use I2C to communicate, 2 pins are required to  
  interface. For Arduino UNOs, thats SCL -> Analog 5, SDA -> Analog 4

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

//-----------------setting up servo driver stuff---------------------

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(); //First servo driver

Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41); //second servo driver


#define SERVOMIN_FRET  110 
#define SERVOMAX_FRET  575 
#define SERVOMIN_STRUM  110 
#define SERVOMAX_STRUM  575 //setting limits of different servo travels to the corresponding PWM pulse length for the servo driver, so that I can just tell the servo what angle to go to.

// our servo # counter
uint8_t servonum = 0;

//--------------------define variables with regards to fretting and strumming mechanisms-----------------------

int E1 = 0; //in regards to the fretting mechanism, refers to the servo responsible for the fret on the E string (fattest) that is closest to the head of the guitar
int E2 = 1; //the integer denoting which pin on the servo driver will increase past 15 (each has pins from 0-15 for 16 servos total). In code, when feeding these as an argument, I will need to have a check for if the value of the servos pin is less than or greater than 15
int E3 = 2; //if less than 15, use the object "pwm1" to tell the arduino that it is on the first servo driver, if above 15, first subtract 16 from its value (to set it to between 0 and 15), then use the object "pwm2" to tell the arduino to use the second servo driver
int E4 = 3;

int A1 = 4;
int A2 = 5;
int A3 = 6;
int A4 = 7;

int D1 = 8;

void setup() {
  Serial.begin(9600);
  
  pwm1.begin();
  pwm2.begin();
  
  pwm1.setPWMFreq(60);  
  pwm2.setPWMFreq(60);

  //yield();
}


void loop() {


  pwm.setPWM(0, 0, angleToPulse(180) );
  pwm.setPWM(1, 0, angleToPulse(180) );
  pwm.setPWM(2, 0, angleToPulse(180) );
  pwm.setPWM(3, 0, angleToPulse(180) );
  pwm.setPWM(4, 0, angleToPulse(180) );
  pwm.setPWM(5, 0, angleToPulse(180) );

  delay(500);

  pwm.setPWM(0, 0, angleToPulse(0) );
  pwm.setPWM(1, 0, angleToPulse(0) );
  pwm.setPWM(2, 0, angleToPulse(0) );
  pwm.setPWM(3, 0, angleToPulse(0) );
  pwm.setPWM(4, 0, angleToPulse(0) );
  pwm.setPWM(5, 0, angleToPulse(0) );
  delay(500);

 
}

/*
 * angleToPulse(int ang)
 * gets angle in degree and returns the pulse width
 * also prints the value on seial monitor
 * written by Ahmad Nejrabi for Robojax, Robojax.com
 */
int angleToPulseFret(int ang){
   int pulse = map(ang,0, 180, SERVOMIN_FRET,SERVOMAX_FRET);// map angle of 0 to 180 to Servo min and Servo max 
   return pulse;
}
int angleToPulseStrum(int ang){
   int pulse = map(ang,0, 180, SERVOMIN_STRUM,SERVOMAX_STRUM);// map angle of 0 to 180 to Servo min and Servo max 
   return pulse;
}
