
#include <Arduino.h>
#include "Mai3Servo.h"
#include <math.h>
#include "writeMessages.h"
#include "feedback.h"

bool log_i21 = true;

// set up a servo with speed control
void Mai3Servo::begin(int servoPin, int servoMin, int servoMax, int servoAutoDetachMs, bool servoInverted, int servoLastPos, int powerPin) {

	assigned = true;
	pin = servoPin;
	servoPowerPin = powerPin;
	min = servoMin;
	max = servoMax;
	autoDetachMs = servoAutoDetachMs;
	inverted = servoInverted;
	currentPosition = servoLastPos;
	inMoveRequest = false;
	thisServoVerbose = false;		// assume verbose off
	servo.detach();
}

// powerUp
void Mai3Servo::powerUp() {

	// on power up set servo to the last known position
	startMillis = millis();
	writeServoPosition(currentPosition, inverted);

	if (thisServoVerbose) {
		Serial.print("i14 powerUp, pin: "); Serial.print(pin);
		Serial.print(", currentPosition: "); Serial.print(currentPosition);
		Serial.print(", inverted: "); Serial.print(inverted);
		Serial.println();
	}
	attach();
}

// stop servo
void Mai3Servo::stopServo() {
	numPartialSteps = 0;		// this will stop writing new positions to the servo
	finalPositionRequestedMillis = millis();
	arrivedMillis = millis();
	moving = false;
	inMoveRequest = false;
	if (!isFeedbackServo) {
		currentPosition = wantedPosition;
	}
	if (log_i21 || thisServoVerbose) {
		Serial.print("i21 servo stop received, ");
		Serial.print(servoName);
		Serial.print(", currentPosition: ");
		Serial.print(currentPosition);
		Serial.println();
	}
	byte status = buildStatusByte(assigned, moving, servo.attached(), autoDetachMs>0, thisServoVerbose, true);
	sendServoStatus(pin, status, currentPosition);
	//loggedLastPos = lastPosition;
	lastStatusUpdate = millis();
}


// only update servoPosition, do not move the servo
void Mai3Servo::setCurrentPosition(int newCurrentPosition) {
	currentPosition = newCurrentPosition;
}


// attach servo
void Mai3Servo::attach() {
	servo.attach(pin);
}


// return attached state of servo
bool Mai3Servo::attached() {
	return servo.attached();
}


// keep requested position in min/max range of servo
int Mai3Servo::adjustOutlierPosition(int targetPos) {

	int adjustedPos = targetPos;

	// adjust if smaller than min
	if (targetPos < min) {
		adjustedPos = min;
		Serial.print("w01 ");
		Serial.print(servoName);
		Serial.print(", position adjusted, requested position: ");
		Serial.print(targetPos);
		Serial.print(" min pos: ");
		Serial.print(min);
		Serial.println();
	}

	// .. or greater than max
	if (targetPos > max) {
		adjustedPos = max;
		Serial.print("w02 ");
		Serial.print(servoName);
		Serial.print(", position adjusted, requested position: ");
		Serial.print(targetPos);
		Serial.print(" max pos: ");
		Serial.print(max);
		Serial.println();
	}

	return adjustedPos;
}


// move to relative position
void Mai3Servo::moveTo(int targetPos, int thisDuration) {

	if (!assigned) {
		Serial.println("e01 no action, servo not assigned yet");
		return;
	}

	if (!attached()) {
		// individual servos might get detached by reaching autodetach time after finished move
		//Serial.print("e02 sequence error, servo not attached "); Serial.print(servoName); Serial.println();
		attach();
	}

	targetPosition = adjustOutlierPosition(targetPos);

	startMillis = millis();		// for realtime log
	startPosition = currentPosition;
	durationMs = thisDuration;

	if (targetPosition == currentPosition ) {
		// ignore move command to current position
		//expectedPosition = currentPosition;		// make sure to report last position in servo update
		if (thisServoVerbose) {
			Serial.print("i01 request for move to current position, request ignored ");
			Serial.print(servoName);
			Serial.println();
		}
		// send target reached message to controller
		byte status = buildStatusByte(assigned, moving, servo.attached(), autoDetachMs > 0, thisServoVerbose, true);
		sendServoStatus(pin, status, currentPosition);
		return;
	}

	inMoveRequest = true;

	// for feedback servos: to calculate movedAngle we need to know how the sensor rotates in the requested move
	if (feedbackInverted) {
		isFeedbackClockwise = targetPosition > currentPosition;
	} else {
		isFeedbackClockwise = targetPosition < currentPosition;
	}

	// break the move into partial requests in 20 ms intervalls
	numPartialSteps = durationMs / 20;
	stepIncrement = (targetPosition - currentPosition) / float(numPartialSteps);
	// start with wanted position = currentPosition and request a linear move to the target
	wantedPosition = currentPosition;

	if (isFeedbackServo) {
		// initialize variables for monitoring
		magnetStartAngle = readCurrentMagnetAngle(i2cMultiplexerChannel, false);
		magnetCurrentAngle = magnetStartAngle;
		magnetPreviousAngle = magnetStartAngle;
		magnetAngleToMove = (targetPosition - currentPosition) * degPerPos;
		angleFromFullRotations = 0;
		magnetAngleMoved = 0;
		//startupBoostActive = true;		// this requests the final position to get things going
		//Serial.println("startupBoost activated");
		//boostPos = targetPos;

		// initialize PID controller
		pidError = 0;
		cumError = 0;
		rateError = 0;		
		prevStepMillis = millis();
		pidLastError = targetPosition - currentPosition;

		// set an initial servoWritePosition to force the start of the servo
		servoWritePosition = wantedPosition - (2 * (startPosition - targetPosition));
	}

	moving = true;
	lastStatusUpdate = millis();

	if (thisServoVerbose) {
		Serial.print("v01 ");
		Serial.print(servoName);
		Serial.print(", a"); Serial.print(arduinoId);
		Serial.print(", moveTo"); 
		Serial.print(", pin: "); Serial.print(pin);
		Serial.print(", targ: "); Serial.print(targetPos);
		Serial.print(", dur: "); Serial.print(durationMs);
		Serial.print(", start: "); Serial.print(currentPosition);
		Serial.print(", numSteps: "); Serial.print(numPartialSteps);
		Serial.print(", stepInc: "); Serial.print(stepIncrement);
		Serial.println();
	}
}


// inverted flag is only treated here, do not include it in position calculation
void Mai3Servo::writeServoPosition(int position, bool inverted) {

	if (thisServoVerbose)  {
		Serial.print(millis()-startMillis); Serial.print(" ms ");
		Serial.print("writeServoPosition: "), Serial.print(position); Serial.println();
	}

	if (inverted) {
		servo.write(180 - position);
	}
	else {
		servo.write(position);
	}
}


void Mai3Servo::detachServo(bool forceDetach) {
	if (servo.attached() || forceDetach) {
		servo.detach();

		if (thisServoVerbose) {
			Serial.print("m14 pin: "); Serial.print(pin); 
			Serial.print(", "); Serial.print(servoName); Serial.print(" detached");
			Serial.println();
		}
	}
}

byte Mai3Servo::evalPositionFromFeedbackSensor() {

	// magnet angle moved is a +/- angle)
	if (isFeedbackClockwise) {
		magnetAngleMoved = magnetStartAngle - magnetCurrentAngle + angleFromFullRotations;
	} else {
		magnetAngleMoved = magnetStartAngle - magnetCurrentAngle - angleFromFullRotations;
	}
	return startPosition + round(magnetAngleMoved / degPerPos);
}


int Mai3Servo::computePid() {

	unsigned long currentTime = millis();
    unsigned long elapsedTime = (currentTime - prevStepMillis) / 20;       //compute time elapsed from previous computation
	int out;

    pidError = wantedPosition - currentPosition;

	cumError += pidError * elapsedTime;               // compute integral
	rateError = (pidError - pidLastError)/elapsedTime;   // compute derivative

	out = wantedPosition + int(kp*pidError + ki*cumError + kd*rateError);  //PID output      

	if (out < 0) out = 0;
	if (out > 180) out = 180;
	
	pidLastError = pidError;            //remember current error
	prevStepMillis = currentTime;       //remember current time

	if (thisServoVerbose) {
		Serial.print("PID, pidError: "); Serial.print(pidError);
		Serial.print(", cumError: "); Serial.print(cumError);
		Serial.print(", rateError: "); Serial.print(rateError);
		Serial.print(", out:"); Serial.print(out);
		Serial.println();
	}

	return out;                         //return the new servoWritePosition
}

int Mai3Servo::computeBand() {
	/*
	compare wanted and current position
	if wanted position differs from the current position, add a fixed offset to the wanted
	position to set the servoWritePosition
	*/
	int band = 40;
	int offset = wantedPosition - currentPosition;
	int controlPosition = wantedPosition;
	if (offset > 2) {		// current below wanted
		controlPosition = wantedPosition + band;
	}
	if (offset < -2) {
		controlPosition = wantedPosition - band;
	}
	// limit to allowed values 0..180
	if (controlPosition > 180) {controlPosition = 180;}
	if (controlPosition < 0)   {controlPosition = 0;}
	return controlPosition;
	
}


// call this in loop
void Mai3Servo::update()
{
	if (!assigned) {
		return;
	}

	if (!servo.attached()) {
		return;
	}

	if (thisServoVerbose) {
		Serial.print("servo update ms: "); Serial.println(millis() - startMillis);
	}

	// limit duration in general (if we can't get to our position)
	int maxDuration = 2 * durationMs + autoDetachMs;
	if (millis() - startMillis > (2 * durationMs + autoDetachMs)) {
		Serial.print("forced servo stop, maxDuration exceeded: "); Serial.println(maxDuration);
		stopServo();
	}	


	byte status = buildStatusByte(assigned, moving, servo.attached(), autoDetachMs, thisServoVerbose, false);

	if (isFeedbackServo) {
		if (thisServoVerbose) {
			Serial.print(millis() - startMillis); Serial.print(" ms, ");
			Serial.print("i60 read magnet, channel: "); Serial.print(i2cMultiplexerChannel);
			Serial.println();
		}
		magnetCurrentAngle = readCurrentMagnetAngle(i2cMultiplexerChannel, true);
		//if (log_i6x) {Serial.print("i61 magnet position: "); Serial.println(magnet);}
		int ms = millis() - startMillis;

		// detect overflow of magnet rotation
		if (abs(magnetPreviousAngle - magnetCurrentAngle) > 180) {
			// take special care when magnetCurrentAngle has hysteresis (moves forth and back over overflow position)
			if (isFeedbackClockwise && magnetCurrentAngle > 180) {angleFromFullRotations += 360;}
			if (isFeedbackClockwise && magnetCurrentAngle < 180) {angleFromFullRotations -= 360;}
			if (!isFeedbackClockwise && magnetCurrentAngle > 180) {angleFromFullRotations -= 360;}
			if (!isFeedbackClockwise && magnetCurrentAngle < 180) {angleFromFullRotations += 360;}
		}
		magnetPreviousAngle = magnetCurrentAngle;

		// detect move started and stop boost
		//if (startupBoostActive && abs(magnetStartAngle - magnetCurrentAngle) > 3) {
		//	startupBoostActive = false;
		//	Serial.println("startupBoost deactivated");
		//}

		currentPosition = evalPositionFromFeedbackSensor();

		if (thisServoVerbose) {
			Serial.print(ms); Serial.print(" ms");
			Serial.print(", startPos: "); Serial.print(startPosition);
			Serial.print(", targetPos: "); Serial.print(targetPosition);			
			Serial.print(", currPos: "); Serial.print(currentPosition);			
			Serial.print(", magStart: "); Serial.print(magnetStartAngle);
			Serial.print(", magToMove: "); Serial.print(magnetAngleToMove);						
			Serial.print(", magCurr: "); Serial.print(magnetCurrentAngle);
			Serial.print(", fullRot: "); Serial.print(angleFromFullRotations);
			Serial.print(", magMoved: "); Serial.print(magnetAngleMoved);
			Serial.println();
		}

		sendFeedbackStatus(pin, status, currentPosition, ms, servoWritePosition, wantedPosition);

	} else {
		sendServoStatus(pin, status, currentPosition);
	}
	//loggedLastPos = int(nextPos);
	lastStatusUpdate = millis();



	// check for target reached	(this might need an update as requesting final position is not
	// the same as arriving there. For non-feedback servos a delay might be useful)

	if (isFeedbackServo) {
		// feedback servo
		// ===============
		// check for close to requested position
		if (moving && abs(currentPosition - targetPosition) <= 2) {
			moving = false;
			arrivedMillis = millis();
			inMoveRequest = false;
			int ms = millis() - startMillis;

			if (verbose || thisServoVerbose) {
				Serial.print("i12 target reached "); Serial.print(servoName);
				Serial.print(", currentPos: "); Serial.print(currentPosition);
				Serial.println();
			}
			byte status = buildStatusByte(assigned, moving, servo.attached(), autoDetachMs > 0, thisServoVerbose, true);
			sendFeedbackStatus(pin, status, currentPosition, ms, servoWritePosition, wantedPosition);
			return;
		}
	} else {
		// non feedback servo
		// ==================
		if (moving && numPartialSteps <= 0) {
			moving = false;
			arrivedMillis = millis();
			currentPosition = wantedPosition;		// the assumed reached position

			if (verbose || thisServoVerbose) {
				Serial.print("i11 target reached "); Serial.print(servoName);
				Serial.print(", position: "); Serial.print(currentPosition);
				Serial.println();
			}
			byte status = buildStatusByte(assigned, moving, servo.attached(), autoDetachMs > 0, thisServoVerbose, true);			
			sendServoStatus(pin, status, currentPosition);
			return;
		}
	}

	// detach servo with delay after we have arrived at target position
	if (autoDetachMs > 0) {

		// check for move ended and autoDetachMs expired
		if ((!moving) && ((millis() - finalPositionRequestedMillis) > autoDetachMs)) {

			// set servo's inMoveRequest to false
			inMoveRequest = false;

			if (thisServoVerbose) {
				Serial.print("i13 servo "); Serial.print(servoName);
				Serial.print(" inMoveRequest cleared, autoDetachMs "); Serial.print(autoDetachMs);
				Serial.print(" ms after arrived: "); Serial.print((millis()-arrivedMillis));
				Serial.println();
			}
			return;
		} 
	}

	// ==============================================================================
	// this is the last step in the update procedure, request the next servo position
	// ==============================================================================

	if (isFeedbackServo) {
		int msInMove = millis() - startMillis;

		// wanted position is a linear position between startPosition and targetPosition within the move duration time
		wantedPosition = startPosition + ((targetPosition - startPosition) / float(durationMs) * msInMove);

		// try to improve positioning by adding a sinusoidal part for acceleration/deceleration
		float xFactor = float(msInMove) / durationMs;
		int yRange = targetPosition - startPosition;
		int linearY = xFactor * yRange * 1.05;		// for compensating lag request a slightly ahead position

		float sinePart = xFactor * 4 - 2;
		int sineFactor = 8;
		float sineYOffset =  sineFactor * sin(sinePart*3.1416/2);

		int sinedY;
		if (targetPosition > startPosition) {
			sinedY = int(startPosition + linearY + sineYOffset);
		} else {
			sinedY = int(startPosition + linearY - sineYOffset);
		}
		Serial.print("startPosition: "); Serial.print(startPosition);
		Serial.print(", linearY: "); Serial.print(linearY);
		Serial.print(", sinePart: "); Serial.print(sinePart);
		Serial.print(", sineYOffset: "); Serial.print(sineYOffset); 
		Serial.print(", sinedY: "); Serial.print(sinedY);
		Serial.println();

		wantedPosition = sinedY;

		// if the move time has elapsed limit the wantedPositon to the targetPosition
		if (msInMove >= durationMs) {
			wantedPosition = targetPosition;
		}

		if (usePidControl)  {
			// until the joint starts to move give it kind of a far target
			if (abs(magnetAngleMoved) < 3) {
				servoWritePosition = wantedPosition - (2 * (startPosition - targetPosition));
			} else {
				// during the rest of the move use the PID value based on the difference of the
				// wanted position versus the current position
				servoWritePosition = computePid();
			}
		} 
		

		// set start of autoDetach time when target is reached and stop the servo
		if (abs(currentPosition - targetPosition) < 1) {
			finalPositionRequestedMillis = millis();
			stopServo();
			return;
		}
		writeServoPosition(servoWritePosition, inverted);

		if (thisServoVerbose) {
			Serial.print("i17, millisInMove: "); Serial.print(msInMove);
			Serial.print(", startPos: "); Serial.print(startPosition);
			Serial.print(", targetPos: "); Serial.print(targetPosition);
			Serial.print(", wantedPos: "); Serial.print(wantedPosition);
			Serial.print(", servoPos: "); Serial.print(servoWritePosition);
			Serial.println();
		}

	} else {

		// non-feedback servo
		// ==================
		if (numPartialSteps > 0) {

			wantedPosition += stepIncrement;				// ... and nextPos as the next step position
			numPartialSteps -= 1;
			// if we have sent the target position to the servo note this time
			// to limit the duration with feedback servos
			if (numPartialSteps <= 0) {
				finalPositionRequestedMillis = millis();
				if (thisServoVerbose) {
					Serial.println("finalPositionRequestedMillis set");
				}
			}
			servoWritePosition = round(wantedPosition);
			//if (startupBoostActive) {
			//	servoWritePosition = boostPos;
			//}
			writeServoPosition(servoWritePosition, inverted);
			if (thisServoVerbose) {
				Serial.print("writeServoPosition, step: "); Serial.print(numPartialSteps);
				Serial.print(", wantedPosition: "); Serial.print(wantedPosition); 
				Serial.print(", servoWritePosition: "); Serial.print(servoWritePosition); 
				Serial.println();
			}
		}
	}
}


