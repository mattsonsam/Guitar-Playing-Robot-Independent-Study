#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Servo.h>
#include <LiquidCrystal.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

//---------------define servo angle min and maxes-----------------
#define SERVOMIN_FRET  110 
#define SERVOMAX_FRET  575 
#define SERVOMIN_STRUM  110 
#define SERVOMAX_STRUM  575 //setting limits of different servo travels to the corresponding PWM pulse length for the servo driver, so that I can just tell the servo what angle to go to.

//------------create stepper and servo driver objects------------------

AccelStepper fretting(1, 60, 61); //using y-step pins
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(); //First servo driver
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41); //second servo driver
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

    //----------------stepper motor parameters
long fret_strokelengthmm = 400; //the length of the entire fretting linear rail
double fret_step_per_rev = 200; // steps per revolution of stepper
double fret_mm_per_rev = 72; //linear distance travelled in one rotation of stepper
double fret_mmPerStep = fret_mm_per_rev / fret_step_per_rev; //linear distance travelled in one step of the stepper
int fretHomingSpeed = -200;
int frethome = 3; //pin for limit switch for fretter


    //-------strumming servos positions *****subject to change************-----------
double strum_mute_right=125;
double strum_mute_left=124;
double strum_angle=25;
double strum_right=strum_mute_right+strum_angle;
double strum_left=strum_mute_right-strum_angle;

//-----------------------------------Declare fretting chord positions, and timing relevent to hard coding songs-----------------------------------------

int E = 53; int EPos = 0; //all other positions are based off E to accomodate for any moves in the limit switch
int F = 36 + E; int FPos = 1;
int Fs = 67 + E; int FsPos = 2; //the "s" in Fs stands for "sharp"
int G = 97 + E; int GPos = 3;
int Gs = 124 + E; int GsPos = 4;
int A = 148 + E; int APos = 5;
int As = 172 + E; int AsPos = 6;
int B = 191 + E; int BPos = 7;
int C = 212 + E; int CPos = 8;
int Cs = 235 + E; int CsPos = 9;
int D = 275 + E; int DPos = 10;
int Ds = 295 + E; int DsPos = 11;

int chordMatrix[] = {E, F, Fs, G, Gs, A, As, B, C, Cs, D, Ds}; //matrix of all chords, chord position in matrix corresponds to pos variable

void setup(){
  Serial.begin(9600);
  pinMode(frethome, INPUT_PULLUP); //sets two limit switches as inputs with internal resistors
  pwm1.begin();
  pwm2.begin();
  pwm1.setPWMFreq(60);  
  pwm2.setPWMFreq(60);
  fretting.setMaxSpeed(1000000.0); //set arbitrarily high max speed, in units of [pulses/sec] (max reliable is about 4000 according to accelstepper).Prevents speed from being limited by code, we will not use this
  delay(1000);

  goHome(fret_strokelengthmm, fret_mmPerStep, fretting, fretHomingSpeed);
}

void loop(){
  
}

//------------FUNCTION DEFINITIONS---------------
  //----------Stepper motor motion functions-----------
    //---go home----
void goHome(long strokeLengthmm, long mmPerStep, AccelStepper stepper, int homingSpeed) {
  long strokelength = ceil(strokeLengthmm / mmPerStep); //calculates number of steps to traverse entire rail
  stepper.moveTo(strokelength);

  while (digitalRead(frethome) == 0) {
    stepper.setSpeed(homingSpeed);
    stepper.run();
    //Serial.println(stepper.currentPosition());
  }
  stepper.setCurrentPosition(0); //once home, set position to zero for reference
  delay(1000);
}

void gotochord(int chordAsNum, double t) { //posInSongMatrixStrings will be the counter in a for loop
  long targetsteps = floor(chordAsNum / fret_mmPerStep);
  float accel = (4.5 * (fretting.currentPosition() - targetsteps)) / pow(t, 2);
  float v_max = (1.5 * (fretting.currentPosition() - targetsteps)) / t;
  fretting.setSpeed(v_max);
  fretting.setAcceleration(accel);
  fretting.moveTo(targetsteps);
  while (fretting.currentPosition() != fretting.targetPosition()) { //while not at the target position, execute steps
    fretting.run();
  }
}

  //--------servo functions----------
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
