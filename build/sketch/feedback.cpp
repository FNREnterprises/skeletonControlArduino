
#include <Wire.h>
#include "feedback.h"

//int AS5600_ADDRESS=0x36;
//int TCA9548_ADDRESS=0x70;
int RAW_ANGLE_HI=0x0c;
int RAW_ANGLE_LO=0x0d;

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

/*
 * Funcion: selectChannel
 * ----------------------
 * AS5600 has a fixed i2c address. to connect more than 1 AS5600 the multiplexer
 * TCA9548 is used.
 * The multiplexer has a fixed address 0x70 .. 0x77 (soldered on chip)
 */
void selectChannel(byte channel) {
  //Serial.print("i62 select channel: "); Serial.println(channel);
  Wire.beginTransmission(TCA9548_ADDRESS);
  Wire.write(1 << channel);
  Wire.endTransmission();
  //Serial.println("i63 channel selected");
}


int getRegisterValue(int registerAddr) {
  /* Read Low Byte */
  //Serial.print("i64 getRegisterValue: "); Serial.println(registerAddr);
  Wire.beginTransmission(AS5600_ADDRESS);
  Wire.write(registerAddr);
  Wire.endTransmission();
  Wire.requestFrom(AS5600_ADDRESS, 1);
  unsigned long timeout = millis() + 10;
  while (Wire.available() == 0 && millis() < timeout);
  if (millis() >= timeout) {Serial.println("i65 no response from i2c"); return 0;}
  byte value = Wire.read();
  //Serial.print("i66 value read "); Serial.print(registerAddr); Serial.print(": "); Serial.println(value);
  return value;
}


/*
 * Function: readCurrentMagnetAngle
 * --------------------------------
 *   returns: the absolut magnet angle as a value between 0 and 360 degrees.
 */
int readCurrentMagnetAngle(byte channel, bool isVerbose) {
  selectChannel(channel);
  int raw_hi = getRegisterValue(RAW_ANGLE_HI);
  int raw_lo = getRegisterValue(RAW_ANGLE_LO);
  int raw = (raw_hi << 8) + raw_lo;
  int angle = int(raw / 4096.0 * 360);
  if (isVerbose) {
    Serial.print("i69 magnet hi: "); Serial.print(raw_hi); 
    Serial.print(", lo: "); Serial.print(raw_lo); 
    Serial.print(", total: "); Serial.print(raw);
    Serial.print(", angle: "); Serial.println(angle);
  }
  return angle;
}

int absAngleDiff(int a, int b) {
  int diff = (a % 360) - (b % 360) + 360;   // arduino can not modulo with neg numbers
  return abs((diff + 180) % 360 - 180) % 360; 
}
