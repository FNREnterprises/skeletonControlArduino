
#ifndef feedback_h
#define feedback_h

#include "Arduino.h"

#define TCA9548_ADDRESS 0x70
#define AS5600_ADDRESS 0x36

extern int readCurrentMagnetAngle(byte channel, bool isVerbose);
extern int absAngleDiff(int a, int b);

#endif