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

int pos = 85;    // variable to store the servo position
int strum_angle=15;
int deg_per_sec=300; //degrees per secong of MG995 servo

void setup() {
  myservo.attach(4);  // attaches the servo on pin 9 to the servo object
}

void loop() {
  /*myservo.write(pos);
  delay(200);*/
  myservo.write(pos+strum_angle);
  delay(1000);
  /*myservo.write(pos);
  delay(200);*/
  myservo.write(pos-strum_angle);
  delay(1000);
}
