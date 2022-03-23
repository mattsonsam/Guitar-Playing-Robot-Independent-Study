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
    //-----fretting servos--------
                                                                // the first row represents the pin number of the servo in relation to the two servo drivers 
int E[3][4] = {{0,1,2,3},{170,170,160,180},{100,100,100,100}};  //the second row in each array indicates the "up" position of each manipulator in terms of degrees
int A[3][4] = {{4,5,6,7},{180,140,160,160},{100,100,100,100}};  //the third row in each array indicates the "down" position of each manipulator in terms of degrees
int D[3][4] = {{8,9,10,11},{170,170,170,170},{100,100,100,100}}; //this way, if each servo responds slightly differently to a pwm command sent to it, I can compensate for the specific servos by giving them
int G[3][4] = {{12,13,14,15},{0,10,10,20},{70,70,70,70}};           // their own unique up and down positions. There are probably better ways to do this
int B[3][4] = {{16,17,18,19},{0,10,20,10},{70,70,70,70}}; 
int e[3][4] = {{20,21,22,23},{10,10,0,0},{70,70,70,70}}; 

    //------strumming servos will be exclusively on the second servo driver--------

int Strum_E = 24-16; //defined in this way so that its clear that these are going to be plugged in to the second servo driver
int Strum_A = 25-16;
int Strum_D = 26-16;
int Strum_G = 27-16;
int Strum_B = 28-16;
int Strum_e = 29-16; 




void setup() {
  Serial.begin(9600);
  
  pwm1.begin();
  pwm2.begin();
  
  pwm1.setPWMFreq(60);  
  pwm2.setPWMFreq(60);

  //yield();
}


void loop() {
pentatonicscale(500);


 
}

//-----------map desired angle to pwm value to give to servos---------
int angleToPulseFret(int ang){
   int pulse = map(ang,0, 180, SERVOMIN_FRET,SERVOMAX_FRET);// map angle of 0 to 180 to Servo min and Servo max 
   return pulse;
}
int angleToPulseStrum(int ang){
   int pulse = map(ang,0, 180, SERVOMIN_STRUM,SERVOMAX_STRUM);// map angle of 0 to 180 to Servo min and Servo max 
   return pulse;
}

//---------basic functions to move servos----------------
void movefrettingservo(int servo, int ang){
  if(servo<=15){
    pwm1.setPWM(servo, 0, angleToPulseFret(ang));
  }
  if(servo>15){
    pwm2.setPWM(servo-16, 0, angleToPulseFret(ang));
  }
}

void movestrummingservo(int servo, int ang){
  pwm2.setPWM(servo, 0, angleToPulseStrum(ang));
}

void fretup(int servo[3][4], int i){
  movefrettingservo(servo[0][i], servo[1][i]);
}
void fretdown(int servo[3][4], int i){
  movefrettingservo(servo[0][i], servo[2][i]);
}
void pentatonicscale(int d){
 fretdown(E,0);
 delay(d);
 fretup(E,0);
 fretdown(E,3);
 delay(d);
 fretup(E,3);
 fretdown(A,0);
 delay(d);
 fretup(A,0);
 fretdown(A,2);
 delay(d);
 fretup(A,2);
 fretdown(D,0);
 delay(d);
 fretup(D,0);
 fretdown(D,2);
 delay(d);
 fretup(D,2);
 fretdown(G,0);
 delay(d);
 fretup(G,0);
 fretdown(G,2);
 delay(d);
 fretup(G,2);
 fretdown(B,0);
 delay(d);
 fretup(B,0);
 fretdown(B,3);
 delay(d);
 fretup(B,3);
 fretdown(e,0);
 delay(d);
 fretup(e,0);
 fretdown(e,3);
 delay(d);
 fretup(e,3);

}
