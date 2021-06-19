
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
	int loggedLastPos;
	int numIncrements;    // number of 20 milli steps
	float increment;
	unsigned long lastStatusUpdate;  // millis of last status update
	int min;
	int max;

	unsigned long arrivedMillis;	// millis when servo should have arrived
	bool stopped;

public:
	float nextPos;
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
    unsigned long startMillis; // millis of moveTo initiated

	// definitions of feedback servo
	bool isFeedbackServo;
	byte i2cMultiplexerAddress;
	byte i2cMultiplexerChannel;
	int speedACalcType;
	float speedAFactor;
	float speedAOffset;
	int speedBCalcType;
	float speedBFactor;
	float speedBOffset;
	float degPerPos;
	int servoSpeedRange;

	// runtime data of feedback servo
	int magnetStartAngle;		// 0..359
	int magnetPreviousAngle;
	int magnetCurrentAngle;		// 0..359
	int magnetAngleToMove;		// can be more than 360
	int angleFromFullRotations;
	int magnetAngleMoved;
	int startPosition;
	int currentPosition;
	bool clockwise;
	bool servoBlocked = false;
	
	// by controlling speed with small linear position updates the joints show a hefty lag and keep moving
	// when the final requested target has been sent to the servo
	// to compensate start the move with a request for final position, than reduce that value to control speed
	bool startupBoostActive = false;
	int boostPos;


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
