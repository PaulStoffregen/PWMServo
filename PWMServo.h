#ifndef PWMServo_h
#define PWMServo_h

/*
  PWMServo.h - Hardware Servo Timer Library
  http://arduiniana.org/libraries/pwmservo/
  Author: Jim Studt, jim@federated.com
  Copyright (c) 2007 David A. Mellis.  All right reserved.
  renamed to PWMServo by Mikal Hart
  ported to other chips by Paul Stoffregen

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <inttypes.h>
#include "Arduino.h"

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) // Arduino
  #define SERVO_PIN_A 9
  #define SERVO_PIN_B 10
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) // Arduino Mega
  #define SERVO_PIN_A 11
  #define SERVO_PIN_B 12
  #define SERVO_PIN_C 13
#elif defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644__) // Sanguino
  #define SERVO_PIN_A 13
  #define SERVO_PIN_B 12
#elif defined(__AVR_AT90USB162__) // Teensy 1.0
  #define SERVO_PIN_A 17
  #define SERVO_PIN_B 18
  #define SERVO_PIN_C 15
#elif defined(__AVR_ATmega32U4__) // Teensy 2.0
  #define SERVO_PIN_A 14
  #define SERVO_PIN_B 15
  #define SERVO_PIN_C 4
#elif defined(__AVR_AT90USB646__) || defined(__AVR_AT90USB1286__) // Teensy++
  #define SERVO_PIN_A 25
  #define SERVO_PIN_B 26
  #define SERVO_PIN_C 27
#elif defined(__arm__) && defined(TEENSYDUINO)
  #define SERVO_PIN_A 9
  #define SERVO_PIN_B 10
  // all PWM pins supported, but names not defined here...
  // just use the actual PWM pin number with attach()
#else
  #define SERVO_PIN_A 9
  #define SERVO_PIN_B 10
#endif

template <class T>
class PWMServo {
 private:
#if defined(__AVR__)
  uint8_t pin;
  uint8_t min16;  // minimum pulse, 16uS units  (default is 34)
  uint8_t max16;  // maximum pulse, 16uS units, 0-4ms range (default is 150)
  static void seizeTimer1();
  static void releaseTimer1();
  static uint8_t attachedA;
  static uint8_t attachedB;

 public:
  void detach();

#ifdef SERVO_PIN_C
 private:
  static uint8_t attachedC;
#endif
#elif defined(__arm__) && defined(TEENSYDUINO)
  const uint8_t pin;
#endif
  T angle;
  const uint16_t kDutyMinUs;  // minimum pulse, uS unit
  const uint16_t kDutyMaxUs;  // maximum pulse, uS unit
  const float kPeriodUs;      // uS unit

 public:
  const T kAngleMin;
  const T kAngleMax;
  const float kFrequency;
  bool attached = false;

  // attach to a pin, sets pinMode, returns 0 on failure, won't
  // position the servo until a subsequent write() happens
  PWMServo(int pin, int dutyMin = 544, int dutyMax = 2400, T angleMin = 0,
           T angleMax = 180, float frequency = 50.0)
      : pin(pin),
        kDutyMinUs(dutyMin),
        kDutyMaxUs(dutyMax),
        kPeriodUs(1000000 / frequency),
        kAngleMin(angleMin),
        kAngleMax(angleMax),
        kFrequency(frequency) {}

  uint8_t attach() {
    if (pin < 0 || pin >= NUM_DIGITAL_PINS) return 0;
    if (!digitalPinHasPWM(pin)) return 0;
    if (kPeriodUs < kDutyMaxUs) return 0;  // dutyMax must be lower than period
    analogWriteFrequency(pin, kFrequency);
    digitalWrite(pin, LOW);
    pinMode(pin, OUTPUT);
    attached = true;
    return 1;
  }

  void detach() {
    analogWrite(pin, 0);
    attached = false;
  }

  // specify the angle
  void write(T angleArg) {
    if (!attached) return;
    if (angleArg < kAngleMin) angleArg = kAngleMin;
    if (angleArg > kAngleMax) angleArg = kAngleMax;
    angle = angleArg;
    const int TIMER_RESOLUTION = 15;  // highest resolution for Teensy 4.X
    const int periodTicks = (1 << TIMER_RESOLUTION) - 1;
    const T dutyRangeUs = kDutyMaxUs - kDutyMinUs;
    const T angleRange = kAngleMax - kAngleMin;
    T dutyUs = (angleArg - kAngleMin) * dutyRangeUs / angleRange + kDutyMinUs;
    int dutyTicks = round(periodTicks * dutyUs / kPeriodUs);
    noInterrupts();
    uint32_t oldres = analogWriteResolution(TIMER_RESOLUTION);
    analogWrite(pin, dutyTicks);
    analogWriteResolution(oldres);
    interrupts();
  }

  T read() { return angle; }
};

#endif
