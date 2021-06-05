

#define AS5600_ADDRESS 0x36
#define TCA9548_ADDRESS 0x70
#define RAW_ANGLE_HI 0x0c
#define RAW_ANGLE_LO 0x0d

extern int readCurrentMagnetAngle(byte channel);
extern int absAngleDiff(int a, int b);