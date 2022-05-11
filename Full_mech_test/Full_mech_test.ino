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

    
    //the first one on D is just fucked and needs to be replaced, the first one on G seems to work when used with another arduino board, have to test with different pins on the actual servo drivers?                                                            // the first row represents the pin number of the servo in relation to the two servo drivers 
int String_E[3][4] = {{0,1,2,3},{168,180,180,180},{90,130,120,120}};  //the second row in each array indicates the "up" position of each manipulator in terms of degrees
int String_A[3][4] = {{4,5,6,7},{180,175,180,160},{110,120,120,100}};  //the third row in each array indicates the "down" position of each manipulator in terms of degrees
int String_D[3][4] = {{8,9,10,11},{170,170,165,170},{110,110,95,130}}; //this way, if each servo responds slightly differently to a pwm command sent to it, I can compensate for the specific servos by giving them
int String_G[3][4] = {{12,13,14,15},{0,10,10,20},{70,70,70,70}};           // their own unique up and down positions. There are probably better ways to do this
int String_B[3][4] = {{16,17,18,19},{5,10,40,10},{60,70,95,65}}; 
int String_e[3][4] = {{20,21,22,23},{10,10,0,0},{83,70,45,70}}; 

    //------strumming servos will be exclusively on the second servo driver--------
int strum_right=1; //useful variables for addressing the arrays defined above ^ in functions for strumming
int mute_right=2;
int strum_left=3;
int mute_left=4;
int current_pos=5;

int Strum_E[6] = {8,120,103,90,102,103}; int Strum_E_Pos=Strum_E[mute_right];// declared as: Strummer= {pin number, strum right position, mute right position, strum left position, mute left position, current position}
int Strum_A[6] = {9,137,121,106,120,121};   int Strum_A_Pos=Strum_A[mute_right];
int Strum_D[6] = {10,128,109,90,108,109};  int Strum_D_Pos=Strum_D[mute_right];
int Strum_G[6] = {11,105,88,70,87,88};  int Strum_G_Pos=Strum_G[mute_right];
int Strum_B[6] = {12,147,135,121,134,135};  int Strum_B_Pos=Strum_B[mute_right];
int Strum_e[6] = {13,90,75,58,74,75}; int Strum_e_Pos=Strum_e[mute_right];



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

//--------------------------any functions that need optional arguements, their prototypes are declared here--------------
//void multifretdown (int E=-1, int A=-1, int D=-1, int G=-1, int B=-1, int e=-1);

void setup(){
  
  
  Serial.begin(9600);
  pinMode(frethome, INPUT_PULLUP); //sets two limit switches as inputs with internal resistors
  pwm1.begin();
  pwm2.begin();
  pwm1.setPWMFreq(60);  
  pwm2.setPWMFreq(60);
  fretting.setMaxSpeed(1000000.0); //set arbitrarily high max speed, in units of [pulses/sec] (max reliable is about 4000 according to accelstepper).Prevents speed from being limited by code, we will not use this
  delay(1000);
  allfretsup();
  goHome(fret_strokelengthmm, fret_mmPerStep, fretting, fretHomingSpeed);
 
  movestrummingservo(Strum_E[0],Strum_E[strum_right]);
  movestrummingservo(Strum_A[0],Strum_A[strum_right]);
  movestrummingservo(Strum_D[0],Strum_D[strum_right]);
  movestrummingservo(Strum_G[0],Strum_G[strum_right]);
  movestrummingservo(Strum_B[0],Strum_B[strum_right]);
  movestrummingservo(Strum_e[0],Strum_e[strum_right]);
  delay(2000);
  movestrummingservo(Strum_E[0],Strum_E[mute_right]);
  movestrummingservo(Strum_A[0],Strum_A[mute_right]);
  movestrummingservo(Strum_D[0],Strum_D[mute_right]);
  movestrummingservo(Strum_G[0],Strum_G[mute_right]);
  movestrummingservo(Strum_B[0],Strum_B[mute_right]);
  movestrummingservo(Strum_e[0],Strum_e[mute_right]);
  delay(2000);
}

void loop(){
 //everybodyhurts(350);
 //back_in_black(80);
  herecomesthesun(300);
  /*gotochord(f1,5);
  fretdown(String_B,2);
  strum(Strum_B);
  delay(500);*/
 
  
  
}

//------------FUNCTION DEFINITIONS---------------
  //----------Stepper motor motion functions-----------
    //---go home----
void goHome(long strokeLengthmm, long mmPerStep, AccelStepper stepper, int homingSpeed) {
  long strokelength = ceil(strokeLengthmm / mmPerStep); //calculates number of steps to traverse entire rail
  stepper.moveTo(strokelength);
  allfretsup;
  fretup(String_e,3);
  fretup(String_B,0);
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

void movestrummingservo(int servo, int ang){
  pwm2.setPWM(servo, 0, angleToPulseStrum(ang));
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
    //delay(500);
    fretup(String_A,i);
    //delay(500);
    fretup(String_D,i);
    //delay(500);
    fretup(String_G,i);
    //delay(500);
    fretup(String_B,i);
    //delay(500);
    fretup(String_e,i);
    //delay(500);
    
  }
  //fretup(String_e,0);
  //delay(500);
  //for (int i=0; i<16; i++){
    //pwm1.setPWM(i, 0, 4096);
   // pwm2.setPWM(i, 0, 4096);
  //}
}
void allfretsdown(){
  for (int i=0; i<=3; i++){
    fretdown(String_E,i);
    //delay(500);
    fretdown(String_A,i);
    //delay(500);
    fretdown(String_D,i);
    //delay(500);
    fretdown(String_G,i);
    //delay(500);
    fretdown(String_B,i);
    //delay(500);
    fretdown(String_e,i);
    //delay(500);
    
  }
}

void multifretdown (int E, int A, int D, int G, int B, int e){ //an input of zero for any argument corrresponds to not moving the fretting servo
  if (E>0){
    fretdown(String_E,E-1);
  }
  if (A>0){
    fretdown(String_A,A-1);
  }
  if (D>0){
    fretdown(String_D,D-1);
  }
  if(G>0){
    fretdown(String_G,G-1);
  }
  if (B>0){
    fretdown(String_B,B-1);
  }
  if(e>0){
    fretdown(String_e,e-1);
  }
  
}

void multifretup (int E, int A, int D, int G, int B, int e){ //an input of zero for any argument corrresponds to not moving the fretting servo
  if (E>0){
    fretup(String_E,E-1);
  }
  if (A>0){
    fretup(String_A,A-1);
  }
  if (D>0){
    fretup(String_D,D-1);
  }
  if(G>0){
    fretup(String_G,G-1);
  }
  if (B>0){
    fretup(String_B,B-1);
  }
  if(e>0){
    fretup(String_e,e-1);
  }
  
}

//------------------------strumming function---------------
void strum(int servo[6]){
  if(servo[current_pos]>servo[mute_left]){
    movestrummingservo(servo[0],servo[strum_left]);
    servo[current_pos]=servo[strum_left];
    return;
  }
  if(servo[current_pos]<servo[mute_right]){
    movestrummingservo(servo[0],servo[strum_right]);
    servo[current_pos]=servo[strum_right];
    return;
  }
}

void mute(int servo[6]){
  if(servo[current_pos]>servo[mute_right]){
    movestrummingservo(servo[0],servo[mute_right]);
    servo[current_pos]=servo[mute_right];
    return;
  }
  if(servo[current_pos]<servo[mute_left]){
    movestrummingservo(servo[0],servo[mute_left]);
    servo[current_pos]=servo[mute_left];
    return;
  }
}
void muteall(){
  mute(Strum_E);
  mute(Strum_A);
  mute(Strum_D);
  mute(Strum_G);
  mute(Strum_B);
  mute(Strum_e);
}

void noteon(bool E, bool A, bool D, bool G, bool B, bool e, double duration){//for any string marked true, strums said string(s) and then mutes them after the given duration in milliseconds
  if (E==true){
    strum(Strum_E);
  }
  if (A==true){
    strum(Strum_A);
  }
  if (D==true){
    strum(Strum_D);
  }
  if (G==true){
    strum(Strum_G);
  }
  if (B==true){
    strum(Strum_B);
  }
  if(e==true){
    strum(Strum_e);
  }
  delay(duration);
  muteall();
}

void strum_multiple(bool E, bool A, bool D, bool G, bool B, bool e){//for any string marked true, strums said string(s) and then mutes them after the given duration in milliseconds
  if (E==true){
    strum(Strum_E);
  }
  if (A==true){
    strum(Strum_A);
  }
  if (D==true){
    strum(Strum_D);
  }
  if (G==true){
    strum(Strum_G);
  }
  if (B==true){
    strum(Strum_B);
  }
  if(e==true){
    strum(Strum_e);
  }
}

void mute_multiple(bool E, bool A, bool D, bool G, bool B, bool e ){//for any string marked true, strums said string(s) and then mutes them after the given duration in milliseconds
  if (E==true){
    mute(Strum_E);
  }
  if (A==true){
    mute(Strum_A);
  }
  if (D==true){
    mute(Strum_D);
  }
  if (G==true){
    mute(Strum_G);
  }
  if (B==true){
    mute(Strum_B);
  }
  if(e==true){
    mute(Strum_e);
  }
}

//----------------song functions------------
void pentatonicscale(int d){
 gotochord(f5,1);
 fretdown(String_E,0);
 delay(d);
 strum(Strum_E);
 delay(d);
 mute(Strum_E);
 fretup(String_E,0);
 fretdown(String_E,3);
 delay(d);
 strum(Strum_E);
 delay(d);
 mute(Strum_E);
 fretup(String_E,3);
 fretdown(String_A,0);
 delay(d);
 strum(Strum_A);
 delay(d);
 mute(Strum_A);
 fretup(String_A,0);
 fretdown(String_A,2);
 delay(d);
 strum(Strum_A);
 delay(d);
 mute(Strum_A);
 fretup(String_A,2);
 fretdown(String_D,0);
 delay(d);
 strum(Strum_D);
 delay(d);
 mute(Strum_D);
 fretup(String_D,0);
 fretdown(String_D,2);
 delay(d);
 strum(Strum_D);
 delay(d);
 mute(Strum_D);
 fretup(String_D,2);
 fretdown(String_G,0);
 delay(d);
 strum(Strum_G);
 delay(d);
 mute(Strum_G);
 fretup(String_G,0);
 fretdown(String_G,2);
 delay(d);
 strum(Strum_G);
 delay(d);
 mute(Strum_G);
 fretup(String_G,2);
 fretdown(String_B,0);
 delay(d);
 strum(Strum_B);
 delay(d);
 mute(Strum_B);
 fretup(String_B,0);
 fretdown(String_B,3);
 delay(d);
 strum(Strum_B);
 delay(d);
 mute(Strum_B);
 fretup(String_B,3);
 fretdown(String_e,0);
 delay(d);
 strum(Strum_e);
 delay(d);
 mute(Strum_e);
 fretup(String_e,0);
 fretdown(String_e,3);
 delay(d);
 strum(Strum_e);
 delay(d);
 mute(Strum_e);
 fretup(String_e,3);

}

void everybodyhurts_D(int d){
  //-----form D+ chord---
  
  fretdown(String_G,1);
  fretdown(String_B,2);
  fretdown(String_e,1);
  ///----strum sequence-----
  strum(Strum_D);
  delay(d);
  strum(Strum_G);
  delay(d);
  strum(Strum_B);
  delay(d);
  strum(Strum_e);
  delay(d);
  strum(Strum_B);
  delay(d);
  strum(Strum_G);
  delay(d);
 
}
void everybodyhurts_G(int d){
  fretdown(String_e,2);
  //--------strum sequence again-------
  strum(Strum_D);
  delay(d);
  strum(Strum_G);
  delay(d);
  strum(Strum_B);
  delay(d);
  strum(Strum_e);
  delay(d);
  strum(Strum_B);
  delay(d);
  strum(Strum_G);
  delay(d);
 
  
}

void everybodyhurts_Em(int d){
  fretdown(String_D,1);
  //--------strum sequence again-------
  strum(Strum_D);
  delay(d);
  strum(Strum_G);
  delay(d);
  strum(Strum_B);
  delay(d);
  strum(Strum_e);
  delay(d);
  strum(Strum_B);
  delay(d);
  strum(Strum_G);
  delay(d);
  
  
}
 void everybodyhurts_A(int d){
  fretdown(String_D,1);
  fretdown(String_G,1);
  fretdown(String_B,1);
  //--------strum sequence again-------
  strum(Strum_D);
  delay(d);
  strum(Strum_G);
  delay(d);
  strum(Strum_B);
  delay(d);
  strum(Strum_e);
  delay(d);
  strum(Strum_B);
  delay(d);
  strum(Strum_G);
  delay(d);
  
 }

 void everybodyhurts(int d){
  gotochord(f1,1);

  
  everybodyhurts_D(d);
  everybodyhurts_D(d);
  allfretsup();
  everybodyhurts_G(d);
  everybodyhurts_G(d);
  allfretsup();

  
  everybodyhurts_D(d);
  everybodyhurts_D(d);
  allfretsup();
  everybodyhurts_G(d);
  everybodyhurts_G(d);
  allfretsup();

  everybodyhurts_D(d);
  everybodyhurts_D(d);
  allfretsup();
  everybodyhurts_G(d);
  everybodyhurts_G(d);
  allfretsup();

  everybodyhurts_Em(d);
  everybodyhurts_Em(d);
  allfretsup();
  everybodyhurts_A(d);
  everybodyhurts_A(d);
  allfretsup();

  everybodyhurts_Em(d);
  everybodyhurts_Em(d);
  allfretsup();
  everybodyhurts_A(d);
  everybodyhurts_A(d);
  allfretsup();

  everybodyhurts_Em(d);
  everybodyhurts_Em(d);
  allfretsup();
  everybodyhurts_A(d);
  everybodyhurts_A(d);
  everybodyhurts_A(d);
  delay(d/4);
  strum(Strum_D);
  strum(Strum_G);
  strum(Strum_B);
  strum(Strum_e);
  delay(3*d);
  allfretsup();
 }

 void back_in_black(double tempo){
  //tempo is in bpm
  double quarter=(60/tempo)*1000; //duration of a quarter note in milliseconds
  double eighth=quarter/2;
  double sixteenth=eighth/2;

  gotochord(f2,1);

  fretdown(String_A,0);
  fretdown(String_D,0);

  noteon(1,1,1,0,0,0,eighth);

  allfretsup();

  fretdown(String_G,0);
  fretdown(String_B,1);

  delay(eighth*2);

  noteon(0,0,1,1,1,0,sixteenth);
  noteon(0,0,1,1,1,0,sixteenth);
  noteon(0,0,1,1,1,0,sixteenth);

  allfretsup();

  fretdown(String_D,0);
  fretdown(String_G,0);

  delay(eighth*2);

  noteon(0,1,1,1,0,0,sixteenth);
  noteon(0,1,1,1,0,0,sixteenth);
  noteon(0,1,1,1,0,0,sixteenth);

  allfretsup();

  fretdown(String_e,1);

  delay(eighth+quarter+sixteenth);

  noteon(0,0,0,0,0,1,sixteenth);

  fretup(String_e,1);

  noteon(0,0,0,0,0,1,sixteenth);

  fretdown(String_B,1);

  noteon(0,0,0,0,1,0,sixteenth);

  fretup(String_B,1);

  noteon(0,0,0,0,1,0,sixteenth);

  fretdown(String_G,2);
  fretdown(String_G,0);

  noteon(0,0,0,1,0,0,sixteenth);

  fretup(String_G,2);

  noteon(0,0,0,1,0,0,sixteenth);

  fretup(String_G,0);

  noteon(0,0,0,1,0,0,sixteenth);

  delay(sixteenth);

  fretdown(String_A,0);
  fretdown(String_D,0);

  noteon(1,1,1,0,0,0,eighth);

  allfretsup();

  fretdown(String_G,0);
  fretdown(String_B,1);

  delay(eighth*2);

  noteon(0,0,1,1,1,0,sixteenth);
  noteon(0,0,1,1,1,0,sixteenth);
  noteon(0,0,1,1,1,0,sixteenth);

  allfretsup();

  fretdown(String_D,0);
  fretdown(String_G,0);

  delay(eighth*2);

  noteon(0,1,1,1,0,0,sixteenth);
  noteon(0,1,1,1,0,0,sixteenth);
  noteon(0,1,1,1,0,0,sixteenth);

  allfretsup();

  gotochord(f4,sixteenth/1000);

  fretdown(String_E,3);

  noteon(1,0,0,0,0,0,sixteenth);

  fretup(String_E,3);
  fretdown(String_E,0);

  noteon(1,0,0,0,0,0,eighth);

  fretup(String_E,0);
  fretdown(String_E,3);

  noteon(1,0,0,0,0,0,sixteenth);

  fretup(String_E,3);
  fretdown(String_E,1);

  noteon(1,0,0,0,0,0,eighth);

  fretup(String_E,1);
  fretdown(String_E,3);

  noteon(1,0,0,0,0,0,sixteenth);

  fretup(String_E,3);
  fretdown(String_E,2);

  noteon(1,0,0,0,0,0,eighth);
  
  fretup(String_E,2);
  fretdown(String_E,3);

  noteon(1,0,0,0,0,0,sixteenth);

  allfretsup(); 

  strum(Strum_E);
  
  gotochord(f2,eighth/1000);

  
  
 }


 void herecomesthesun (double tempo){
  //tempo is in bpm
  double quarter=(60/tempo)*1000; //duration of a quarter note in milliseconds
  double eighth=quarter/2;
  double sixteenth=eighth/2;
  double half=quarter*2;

  int fretdelay=50; //give frets some time to get to their positions

  //------opening riff-----

  gotochord(f7,3);

  multifretdown(0,0,1,3,4,3);
  delay(fretdelay);
  strum_multiple(0,0,1,1,1,1);

  delay(eighth);

  strum_multiple(0,0,0,0,1,0);

  delay(eighth);

  fretup(String_e,2);
  fretdown(String_e,0);
  delay(fretdelay);
  strum(Strum_e);

  delay(eighth);

  //fretup(String_e,0);
  fretdown(String_e,2);
  delay(fretdelay);
  strum(Strum_e);

  delay(quarter);

  strum(Strum_B);

  delay(quarter);

  fretdown(String_e,2);
  //fretup(String_e,0);
  delay(fretdelay);
  strum(Strum_e);

  delay(quarter);

  fretdown(String_e,0);
  fretup(String_e,2);
  delay(fretdelay);
  strum(Strum_e);

  delay(quarter);

  strum(Strum_B);

  delay(quarter);

  allfretsup();
  multifretdown(4,3,1,1,1,0);
  delay(fretdelay);
  strum_multiple(1,1,1,1,1,0);
//------------------------------------------------
  delay(quarter);

  fretdown(String_B,3);
  delay(fretdelay);
  strum(Strum_B);

  delay(quarter);

  fretup(String_B,3);
  fretdown(String_B,0);
  fretdown(String_e,0);
  delay(fretdelay);
  strum_multiple(0,0,0,0,1,1);

  delay(quarter);

  fretdown(String_B,3);
  fretup(String_B,0);
  strum(Strum_B);

  delay(quarter);

  allfretsup();
  multifretdown(0,1,3,1,1,0);
  delay(fretdelay);
  strum_multiple(0,1,1,1,1,0);

  delay(eighth);

  //fretup(String_B,0);
  fretdown(String_B,2);
  strum(Strum_B);

  delay(eighth);

  fretdown(String_B,0);
  fretup(String_B,2);
  delay(fretdelay);
  strum(Strum_B);

  delay(eighth);

  fretdown(String_B,2);
  //fretup(String_B,0);
  strum(Strum_B);

  delay(eighth);

  fretdown(String_B,3);
  fretup(String_B,2);
  delay(fretdelay);
  strum(Strum_B);

  delay(quarter);

  allfretsup();
  multifretdown(0,0,0,0,3,1);
  delay(fretdelay);
  strum_multiple(0,0,0,0,1,1);

  //-------------------------------------------

  delay(half*2);
  
  allfretsup();
   multifretdown(0,0,1,3,4,3);
  delay(fretdelay);
  strum_multiple(0,0,1,1,1,1);

  delay(eighth);

  strum_multiple(0,0,0,0,1,0);

  delay(eighth);

  fretup(String_e,2);
  fretdown(String_e,0);
  delay(fretdelay);
  strum(Strum_e);

  delay(eighth);

  //fretup(String_e,0);
  fretdown(String_e,2);
  delay(fretdelay);
  strum(Strum_e);

  delay(quarter);

  strum(Strum_B);

  delay(quarter);

  fretdown(String_e,2);
  //fretup(String_e,0);
  delay(fretdelay);
  strum(Strum_e);

  delay(quarter);

  fretdown(String_e,0);
  fretup(String_e,2);
  delay(fretdelay);
  strum(Strum_e);

  delay(quarter);

  strum(Strum_B);

  delay(quarter);

  allfretsup();
  multifretdown(4,3,1,1,1,0);
  delay(fretdelay);
  strum_multiple(1,1,1,1,1,0);

  //-------------

  delay(quarter);

  fretdown(String_e,2);
  fretdown(String_e,0);
  delay(fretdelay);
  strum(Strum_e);

  delay(quarter);

  fretup(String_e,2);
  delay(fretdelay);
  strum(Strum_e);

  delay(quarter);

  fretdown(String_B,3);
  delay(fretdelay);

  strum(Strum_B);

  delay(quarter);

  allfretsup();
  multifretdown(1,1,3,1,3,1);
  delay(fretdelay);

  strum_multiple(0,1,1,1,1,0);

  delay(half*4);

  //-----------------------------------verse 1-----------

  allfretsup();
  multifretdown(1,1,1,3,4,3);
  delay(fretdelay);
  strum_multiple(0,0,1,1,1,0);

  delay(quarter);

  strum(Strum_e);

  delay(eighth);

  fretup(String_e,2);
  fretdown(String_e,0);
  delay(fretdelay);
  strum(Strum_e);

  delay(quarter);

  fretdown(String_e,2);
  delay(fretdelay);
  strum(Strum_e);

  delay(quarter);

  strum(Strum_B);

  delay(half);

  strum(Strum_e);

  delay(eighth);

  strum(Strum_B);

  delay(eighth);

  fretup(String_e,2);
  delay(fretdelay);
  strum(Strum_e);

  delay(eighth);

  fretdown(String_e,2);
  delay(fretdelay);
  strum(Strum_e);

  delay(quarter);

  allfretsup();
  multifretdown(4,3,1,1,1,1);
  delay(fretdelay);
  strum_multiple(1,1,1,1,1,0);

  delay(quarter);

  fretdown(String_e,2);
  delay(fretdelay);
  strum(Strum_e);

  delay(quarter);

  fretup(String_e,2);
  delay(fretdelay);
  strum(Strum_e);

  delay(quarter);

  fretdown(String_e,2);
  delay(fretdelay);
  strum(Strum_e);

  delay(quarter);

  allfretsup();
  multifretdown(1,3,1,2,4,1);
  delay(fretdelay);
  strum_multiple(1,1,1,1,1,0);

  delay(half);

  allfretsup();
  multifretdown(1,1,1,3,4,3);
  delay(fretdelay);
  strum_multiple(0,0,1,1,1,0);

  delay(quarter);

  strum(Strum_e);

  delay(quarter);

  fretup(String_e,2);
  fretdown(String_e,0);
  delay(fretdelay);
  strum(Strum_e);

  delay(quarter);

  strum(Strum_B);

  delay(quarter);

  //---------------dododododododododododododo-------------

  allfretsup();
  multifretdown(0,0,0,1,1,1);
  fretdown(String_G,2);
  delay(fretdelay);
  strum(Strum_G);

  delay(eighth);
  
  strum(Strum_B);

  delay(eighth);

  fretdown(String_B,3);
  delay(fretdelay);
  strum(Strum_B);

  delay(eighth);

  strum(Strum_e);

  delay(eighth);

  fretdown(String_G,2);
  delay(fretdelay);
  strum(Strum_G);

  delay(eighth);

  strum(Strum_B);

  delay(eighth);

  strum(Strum_e);

  delay(eighth);

  fretup(String_G,2);
  delay(fretdelay);
  strum(Strum_G);

  delay(eighth);

  strum(Strum_B);

  delay(eighth);

  strum(Strum_e);

  delay(eighth);

  fretdown(String_G,2);
  delay(fretdelay);
  strum(Strum_G);

  delay(eighth);

  strum(Strum_B);

  delay(eighth);

  strum(Strum_e);

  delay(eighth);

  strum(Strum_B);

  delay(eighth);

  fretdown(String_B,2);
  fretup(String_B,3);
  delay(fretdelay);
  strum(Strum_B);

  delay(eighth);

  fretup(String_B,2);
  delay(fretdelay);
  strum(Strum_B);

  delay(eighth);

  strum(Strum_G);

  delay(quarter);

 /* allfretsup();
  multifretdown(0,0,1,3,4,3);
  delay(fretdelay);
  strum_multiple(0,0,1,1,1,0);

  delay(quarter);

  strum(Strum_e);

  delay(eighth);

  strum(Strum_B);

  delay(eighth)*/

  

  

  

  


  


  //delay(5000);

  allfretsup();

  
  

  
 }
 
 
