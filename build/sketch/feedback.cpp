
#include <Wire.h>
#include "feedback.h"


float calcLinear(float base, float speed, float offset) {
    // calcType 1
    return base * speed + offset;
}

float calcPower(float base, float speed, float offset) {
  // calcType 2
  return base * pow(speed, offset);
}

float calcLog(float base, float speed, float offset) {
  // calcType 3
  Serial.print("calc log, base: "); Serial.print(base); Serial.print(" speed: ");Serial.print(speed);Serial.print(" offset: "); Serial.println(offset);  
  return (base * log(speed)) + offset;    // arduino log = ln, use log10 otherwise
}

int getRegisterValue(int registerAddr) {
  /* Read Low Byte */
  Wire.beginTransmission(AS5600_ADDRESS);
  Wire.write(registerAddr);
  Wire.endTransmission();
  Wire.requestFrom(AS5600_ADDRESS, 1);
  while (Wire.available() == 0)    ;
  return Wire.read();
}

/*
 * Funcion: selectChannel
 * ----------------------
 * AS5600 has a fixed i2c address. to connect more than 1 AS5600 the multiplexer
 * TCA9548 is used.
 * The multiplexer has address 
 */
void selectChannel(byte channel) {
  Wire.beginTransmission(TCA9548_ADDRESS);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

/*
 * Function: readCurrentMagnetAngle
 * --------------------------------
 *   returns: the magnet angle as a value between 0 and 360 degrees.
 */
int readCurrentMagnetAngle(byte channel) {
  selectChannel(channel);
  int raw_hi = getRegisterValue(RAW_ANGLE_HI);
  int raw_lo = getRegisterValue(RAW_ANGLE_LO);
  int raw = (raw_hi << 8) + raw_lo;
  //Serial.print(raw_hi);Serial.print(",");Serial.print(raw_lo); Serial.print(",");Serial.print(value);
  return int(raw / 4096.0 * 360);
}

int absAngleDiff(int a, int b) {
  int diff = (a % 360) - (b % 360) + 360;   // arduino can not modulo with neg numbers
  return abs((diff + 180) % 360 - 180) % 360; 
}
