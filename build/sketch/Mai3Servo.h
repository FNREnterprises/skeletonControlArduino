
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
	//nt loggedLastPos;
	int numPartialSteps;    // number of 20 milli steps
	float stepIncrement;
	unsigned long lastStatusUpdate;  // millis of last status update
	int min;
	int max;

	unsigned long finalPositionRequestedMillis;	// millis when final position requested
	unsigned long arrivedMillis;	// non-feedback-servos: millis when final position requested
									// feedback-servos: millis when final position reached
	bool stopped;

public:
	float nextPos;
	bool assigned;
	bool moving;
	int autoDetachMs;
	int durationMs;			// duration of the move in millis
	int startPosition;		// the current position when requesting the move	
	int targetPosition;		// the move target position
	int currentPosition;	// for non-feedback servos the wantedPosition as we do not know better
							// for feedback servos the measured position from the feedback sensor							
	float wantedPosition;	// linear position progress in move
							// currently a linear position between start and end over time
	byte servoWritePosition;	// position written to the servo
	bool inverted;
	int pin;
	int servoPowerPin;
	//int lastPosition;		// servo.read did not work for me
	bool inMoveRequest;
	bool thisServoVerbose;
	char servoName[20];
    unsigned long startMillis; // millis of moveTo initiated

	// definitions of feedback servo
	bool isFeedbackServo;
	byte i2cMultiplexerAddress;
	byte i2cMultiplexerChannel;
	//int speedACalcType;
	//float speedAFactor;
	//float speedAOffset;
	//int speedBCalcType;
	//float speedBFactor;
	//float speedBOffset;

	// PID
	bool pidEnabled = true;
	float kp = 4;
	float ki = 0;
	float kd = 0;
	float prevStepMillis;
	float pidError;
	float pidLastError;
	float cumError;
	float rateError;

	float degPerPos;
	//int servoSpeedRange;
	bool feedbackInverted;

	// runtime data of feedback servo
	int magnetStartAngle;		// 0..359
	int magnetPreviousAngle;
	int magnetCurrentAngle;		// 0..359
	int magnetAngleToMove;		// can be more than 360
	int angleFromFullRotations;
	int magnetAngleMoved;
	bool isFeedbackClockwise;
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
	void setCurrentPosition(int newCurrentPosition);

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

	byte evalPositionFromFeedbackSensor();

	// needs repeated call
    void update();

	// PID control
	bool usePidControl = true;
	int computePid();

	bool useBandControl = false;
	int computeBand();
};

#endif
