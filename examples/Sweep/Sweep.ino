// Sweep
// by BARRAGAN <http://barraganstudio.com>

// http://arduiniana.org/libraries/pwmservo/

//   Board                     SERVO_PIN_A   SERVO_PIN_B   SERVO_PIN_C
//   -----                     -----------   -----------   -----------
//   Arduino Uno, Duemilanove       9            10          (none)
//   Arduino Mega                  11            12            13
//   Sanguino                      13            12          (none)
//   Teensy 1.0                    17            18            15
//   Teensy 2.0                    14            15             4
//   Teensy++ 1.0 or 2.0           25            26            27
//   Teensy LC & 3.x                 (all PWM pins are usable)

#include <PWMServo.h>

PWMServo<float> myservo(23);                // create servo object to control a servo

float pos = 0;    // variable to store the servo position

void setup() {
  myservo.attach();  // attaches the servo on pin 23 to the servo object
}


void loop() {
  for(pos = myservo.kAngleMin; pos < myservo.kAngleMax; pos += 0.1) { // goes from 0 degrees to 180 degrees, 0.1 degree steps
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(2);                       // waits 2ms for the servo to reach the position
  }
  for(pos = myservo.kAngleMax; pos > myservo.kAngleMin; pos-=0.1) {   // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(2);                       // waits 2ms for the servo to reach the position
  }
}
