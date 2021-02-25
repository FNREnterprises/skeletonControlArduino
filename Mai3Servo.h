
#ifndef Mai3Servo_h
#define Mai3Servo_h

#include "Arduino.h"
#include <Servo.h>

extern int arduinoId;
extern bool verbose;

class Mai3Servo 
{
private:

	Servo servo;
	float nextPos;
	int loggedLastPos;
	int numIncrements;    // number of 20 milli steps
	float increment;
    unsigned long startMillis; // millis of moveTo initiated
	unsigned long lastStatusUpdate;  // millis of last status update
	int min;
	int max;

	unsigned long arrivedMillis;	// millis when servo should have arrived
	bool stopped;

public:
	bool assigned;
	bool moving;
	int autoDetachMs;
	bool inverted;
	int pin;
	int servoPowerPin;
	int lastPosition;		// servo.read did not work for me
	bool inMoveRequest;
	bool thisServoVerbose;
	char servoName[20];

	// assign servo
	void begin(int pin, int min, int max, int autoDetachMs, bool inverted, int lastPos, int servoPowerPin);

	// powerUp
	void powerUp();

	// stop servo
	void stopServo();

	// detach servo
	void detachServo(bool forceDetach);

	// set servoPosition
	void setLastPosition(int newLastPosition);

	// attach servo
	void attach();

	// return attached state as bool
	bool attached();
		
	// check requested position for being in min/max range for the servo
	int adjustOutlierPosition(int targetPos);

	// write servo position
	void writeServoPosition(int position, bool inverted);

	// move to servo position in the range 0..180
	// inversion is handled in the move command
	// the commanding task needs to convert degrees to the relative range
	void moveTo(int targetPos, int durationMillis);


	// needs repeated call
    void update();
};

#endif
