/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Sweep
*/

#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

double pos = 0;    //initial servo position
double fret_distance=5; //mm
double dp=15; //diametral pitch of pinion gear in mm
double fret_angle=(fret_distance*2/dp)*57;
double fret_time=60; //time in ms to complete fretting move 


void setup() {
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
}

void loop() {
  myservo.write(180);
  delay(500);
  
  
}
