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

int pos = 119;    // variable to store the servo position
int strum_length=6;

void setup() {
  myservo.attach(4);  // attaches the servo on pin 9 to the servo object
}

void loop() {
  /*myservo.write(pos);
  delay(500);*/
  myservo.write(pos+strum_length);
  //delay(500);
  /*myservo.write(pos);
  delay(500);*/
  //myservo.write(pos-strum_length);
  //delay(500);
}
