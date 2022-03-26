#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Servo.h>
#include <LiquidCrystal.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <EEPROM.h>

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
int String_E[3][4] = {{0,1,2,3},{168,170,160,180},{100,100,100,100}};  //the second row in each array indicates the "up" position of each manipulator in terms of degrees
int String_A[3][4] = {{4,5,6,7},{180,140,160,160},{100,100,100,100}};  //the third row in each array indicates the "down" position of each manipulator in terms of degrees
int String_D[3][4] = {{8,9,10,11},{170,170,170,170},{100,100,100,100}}; //this way, if each servo responds slightly differently to a pwm command sent to it, I can compensate for the specific servos by giving them
int String_G[3][4] = {{12,13,14,15},{0,10,10,20},{0,70,70,70}};           // their own unique up and down positions. There are probably better ways to do this
int String_B[3][4] = {{16,17,18,19},{0,10,20,10},{70,70,70,70}}; 
int String_e[3][4] = {{20,21,22,23},{10,10,0,0},{70,70,70,70}}; 

    //------strumming servos will be exclusively on the second servo driver--------

int Strum_E[6] = {8,120,101,90,100}; int Strum_E_Pos=90;// declared as: Strummer= {pin number, strum right position, mute right position, strum left position, mute left position, current position}
int Strum_A[6] = {9,90,90,90,90};   int Strum_A_Pos=90;
int Strum_D[6] = {10,90,90,90,90};  int Strum_D_Pos=90;
int Strum_G[6] = {11,90,90,90,90};  int Strum_G_Pos=90;
int Strum_B[6] = {12,90,90,90,90};  int Strum_B_Pos=90;
int Strum_e[6] = {13,90,90,90,90}; int Strum_e_Pos=90;

int strum_right=1; //useful variables for addressing the arrays defined above ^ in functions for strumming
int mute_right=2;
int strum_left=3;
int mute_left=4;
int current_pos=5;

    //----------------stepper motor parameters
long fret_strokelengthmm = 400; //the length of the entire fretting linear rail
double fret_step_per_rev = 200; // steps per revolution of stepper
double fret_mm_per_rev = 72; //linear distance travelled in one rotation of stepper
double fret_mmPerStep = fret_mm_per_rev / fret_step_per_rev; //linear distance travelled in one step of the stepper
int fretHomingSpeed = -200;
int frethome = 3; //pin for limit switch for fretter



//-----------------------------------Declare fretting chord positions, and timing relevent to hard coding songs-----------------------------------------

int f1 = 37; int EPos = 0; //all other positions are based off E to accomodate for any moves in the limit switch
int f2 = 32 + f1; int FPos = 1;
int f3 = 62 + f1; int FsPos = 2; //the "s" in Fs stands for "sharp"
int f4 = 90 + f1; int GPos = 3;
int f5 = 115 +f1; int GsPos = 4;
int f6 = 140 +f1; int APos = 5;
int f7 = 163 +f1; int AsPos = 6;
int f8 = 185 + f1; int BPos = 7;
int f9 = 208 + f1; int CPos = 8;


int fretmatrix[] = {f1,f2,f3,f4,f5,f6,f7,f8,f9}; //matrix of all chords, chord position in matrix corresponds to pos variable

void setup(){
  Serial.begin(9600);
  pinMode(frethome, INPUT_PULLUP); //sets two limit switches as inputs with internal resistors
  pwm1.begin();
  pwm2.begin();
  pwm1.setPWMFreq(60);  
  pwm2.setPWMFreq(60);
  fretting.setMaxSpeed(1000000.0); //set arbitrarily high max speed, in units of [pulses/sec] (max reliable is about 4000 according to accelstepper).Prevents speed from being limited by code, we will not use this
  delay(1000);

  //goHome(fret_strokelengthmm, fret_mmPerStep, fretting, fretHomingSpeed);
}

void loop(){
 strum_E();
 delay(500);
 mute_E();
 delay(500);
  
}

//------------FUNCTION DEFINITIONS---------------
  //----------Stepper motor motion functions-----------
    //---go home----
void goHome(long strokeLengthmm, long mmPerStep, AccelStepper stepper, int homingSpeed) {
  long strokelength = ceil(strokeLengthmm / mmPerStep); //calculates number of steps to traverse entire rail
  stepper.moveTo(strokelength);
  allfretsup;
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
  allfretsup;
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

void movestrummingservo(int servo[6], int ang){
  pwm2.setPWM(servo[0], 0, angleToPulseStrum(ang));
}

void fretup(int servo[3][4], int i){
  movefrettingservo(servo[0][i], servo[1][i]);
}
void fretdown(int servo[3][4], int i){
  movefrettingservo(servo[0][i], servo[2][i]);
}

void allfretsup(){
  for (int i=0; i<=3; i++){
    fretup(String_E,i);
    fretup(String_A,i);
    fretup(String_D,i);
    fretup(String_G,i);
    fretup(String_B,i);
    fretup(String_e,i);
    delay(500);
    
  }
  fretup(String_e,0);
  delay(500);
  for (int i=0; i<16; i++){
    pwm1.setPWM(i, 0, 4096);
    pwm2.setPWM(i, 0, 4096);
  }
}
void pentatonicscale(int d){
 fretdown(String_E,0);
 delay(d);
 fretup(String_E,0);
 fretdown(String_E,3);
 delay(d);
 fretup(String_E,3);
 fretdown(String_A,0);
 delay(d);
 fretup(String_A,0);
 fretdown(String_A,2);
 delay(d);
 fretup(String_A,2);
 fretdown(String_D,0);
 delay(d);
 fretup(String_D,0);
 fretdown(String_D,2);
 delay(d);
 fretup(String_D,2);
 fretdown(String_G,0);
 delay(d);
 fretup(String_G,0);
 fretdown(String_G,2);
 delay(d);
 fretup(String_G,2);
 fretdown(String_B,0);
 delay(d);
 fretup(String_B,0);
 fretdown(String_B,3);
 delay(d);
 fretup(String_B,3);
 fretdown(String_e,0);
 delay(d);
 fretup(String_e,0);
 fretdown(String_e,3);
 delay(d);
 fretup(String_e,3);

}

//------------------------strumming function---------------
void strum_E(){
  if(Strum_E_Pos>Strum_E[mute_left]){
    movestrummingservo(Strum_E, Strum_E[strum_left]);
    Strum_E_Pos=Strum_E[strum_left];
  }
  if(Strum_E_Pos<Strum_E[mute_right]){
    movestrummingservo(Strum_E, Strum_E[strum_right]);
    Strum_E_Pos=Strum_E[strum_right];
  }
}

void mute_E(){
  if(Strum_E_Pos>Strum_E[mute_right]){
    movestrummingservo(Strum_E, Strum_E[mute_right]);
    Strum_E_Pos=Strum_E[mute_right];
  }
  if(Strum_E_Pos<Strum_E[mute_left]){
    movestrummingservo(Strum_E, Strum_E[mute_left]);
    Strum_E_Pos=Strum_E[mute_left];
  }
}
