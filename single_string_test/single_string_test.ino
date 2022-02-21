//TO DO:reformat iron man to fit in regular code
//edits to push

//-------------------------------------SETUP VARIABLES----------------------------------------------------------------------------------------------------
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Servo.h>
#include <LiquidCrystal.h>


AccelStepper fretting(1, 60, 61); //using y-step pins
Servo strum;
Servo fret; //hijack fret servo to do test for new subsystems

int frethome = 3; //pin for limit switch for fretter
int strumhome = 18; //pin for limit switch for strummer

//LCD displays
int rsPin = 16;
//rw is to ground
int enablePin = 17;
int d4Pin = 23;
int d5Pin = 25;
int d6Pin = 27;
int d7Pin = 29;

LiquidCrystal lcd(rsPin, enablePin, d4Pin, d5Pin, d6Pin, d7Pin);

int backButton = 31;
int playButton = 33;
int forwardButton = 35;

long fret_strokelengthmm = 400; //the length of the entire fretting linear rail
double fret_step_per_rev = 200; // steps per revolution of stepper
double fret_mm_per_rev = 72; //linear distance travelled in one rotation of stepper
double fret_mmPerStep = fret_mm_per_rev / fret_step_per_rev; //linear distance travelled in one step of the stepper



int fretHomingSpeed = -200;


double fret_up = 0;    //initial servo position
double fret_distance=5; //mm
double dp=15; //diametral pitch of pinion gear in mm
double fret_angle=(fret_distance*2/dp)*57;
double fret_down=(fret_up+fret_angle);
double fret_time=60; //time in ms to complete fretting move 

double strum_mute_right=83;
double strum_mute_left=86;
double strum_angle=15;
double strum_right=strum_mute_right+strum_angle;
double strum_left=strum_mute_right-strum_angle;









//---------------------------------------Experimental variables, these may change----------------------------------------------------------------------



//-----------------------------------Declare fretting chord positions, and timing relevent to hard coding songs-----------------------------------------

int E = 4; int EPos = 0; //all other positions are based off E to accomodate for any moves in the limit switch
//no E sharp
int F = 35 + E; int FPos = 1;
int Fs = 65 + E; int FsPos = 2; //the "s" in Fs stands for "sharp"
int G = 97 + E; int GPos = 3;
int Gs = 128 + E; int GsPos = 4;
int A = 155 + E; int APos = 5;
int As = 181 + E; int AsPos = 6;
int B = 215 + E; int BPos = 7;
//no B sharp
int C = 240 + E; int CPos = 8;
int Cs = 255 + E; int CsPos = 9;
int D = 275 + E; int DPos = 10;
int Ds = 295 + E; int DsPos = 11;

int chordMatrix[] = {E, F, Fs, G, Gs, A, As, B, C, Cs, D, Ds}; //matrix of all chords, chord position in matrix corresponds to pos variable


//------------------------------------SETUP FUNCTION-------------------------------------------------------------------------------------------------------
void setup() {
  Serial.begin(9600);
  pinMode(strumhome, INPUT_PULLUP); //sets two limit switches as inputs with internal resistors
  

  pinMode(backButton, INPUT_PULLUP);
  pinMode(playButton, INPUT_PULLUP);
  pinMode(forwardButton, INPUT_PULLUP);

  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Starting...");

  fret.attach(11);  //attaches fret servo to pin 11
  strum.attach(4); 
  fretting.setMaxSpeed(1000000.0); //set arbitrarily high max speed, in units of [pulses/sec] (max reliable is about 4000 according to accelstepper).Prevents speed from being limited by code, we will not use this
  delay(1000);

  goHome(fret_strokelengthmm, fret_mmPerStep, fretting, frethome, fretHomingSpeed);


 

}

//-------------------------------------END SETUP------------------------------------------------------------------


//-------------------------------------START VOID LOOP------------------------------------------------------------

void loop() { //here is where we will call all our functions
  strum.write(strum_mute_left);
for (int i=0; i<12; i++){
  gotochord(chordMatrix[i], true, 1);
  if(i%2==0){
    strum.write(strum_right);
    delay(500);
    strum.write(strum_mute_right);
    delay(500);
  }
  if(i%2 != 0){
    strum.write(strum_left);
    delay(500);
    strum.write(strum_mute_left);
    delay(500);
  }
}
  

}


//------------------------------------END VOID LOOP---------------------------------------------------------------


//------------------------------------START FUNCTIONS-------------------------------------------------------------








//---go home----
void goHome(long strokeLengthmm, long mmPerStep, AccelStepper stepper, int limitSwitch, int homingSpeed) {
  fret.write(fret_up);
  strum.write(strum_left);
  delay(500);
  long strokelength = ceil(strokeLengthmm / mmPerStep); //calculates number of steps to traverse entire rail
  stepper.moveTo(strokelength);

  while (digitalRead(limitSwitch) == 0) {
    stepper.setSpeed(homingSpeed);
    stepper.run();
    //Serial.println(stepper.currentPosition());
  }
  stepper.setCurrentPosition(0); //once home, set position to zero for reference
  delay(1000);
}




void gotochord(int chordAsNum, bool major, double t) { //posInSongMatrixStrings will be the counter in a for loop
  fret.write(fret_up);
  bool is_major = major;
  //bool major = fretBoolean(arrayposition, major_minor_array);
  long targetsteps = floor(chordAsNum / fret_mmPerStep);
  float accel = (4.5 * (fretting.currentPosition() - targetsteps)) / pow(t, 2);
  float v_max = (1.5 * (fretting.currentPosition() - targetsteps)) / t;
  fretting.setSpeed(v_max);
  fretting.setAcceleration(accel);
  fretting.moveTo(targetsteps);
  while (fretting.currentPosition() != fretting.targetPosition()) { //while not at the target position, execute steps
    fretting.run();
  }
  if (is_major == true) {
    fret.write(fret_down);
    delay(500);
  }
}


//----------------- end functions ----------------------------------------
