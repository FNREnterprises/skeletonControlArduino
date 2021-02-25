
#include <Arduino.h>
#include "Mai3Servo.h"
#include <math.h>
#include "writeMessages.h"

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
	lastPosition = servoLastPos;
	inMoveRequest = false;
	thisServoVerbose = false;		// assume verbose off
	servo.detach();
}

// powerUp
void Mai3Servo::powerUp() {

	// on power up set servo to the last known position

	writeServoPosition(lastPosition, inverted);

	if (thisServoVerbose) {
		Serial.print("i14 powerUp, pin: "); Serial.print(pin);
		Serial.print(", lastPosition: "); Serial.print(lastPosition);
		Serial.print(", inverted: "); Serial.print(inverted);
		Serial.println();
	}
	attach();
}

// stop servo
void Mai3Servo::stopServo() {
	numIncrements = 0;		// this will stop writing new positions to the servo
	arrivedMillis = millis();
	lastPosition = int(nextPos);	// the last write position for the servo (might however not be the actual position as we have no feedback from servo)
	if (log_i21) {
		Serial.print("i21 servo stop received, ");
		Serial.print(servoName);
		Serial.print(", lastPosition: ");
		Serial.print(lastPosition);
		Serial.println();
	}
	sendServoStatus(pin, lastPosition, assigned, moving, servo.attached(), autoDetachMs, thisServoVerbose, true);
	loggedLastPos = lastPosition;
	lastStatusUpdate = millis();
}


// only update servoPosition, do not move the servo
void Mai3Servo::setLastPosition(int newLastPosition) {
	lastPosition = newLastPosition;
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
void Mai3Servo::moveTo(int targetPos, int duration) {

	if (!assigned) {
		Serial.println("e01 no action, servo not assigned yet");
		return;
	}

	if (!attached()) {
		// individual servos might get detached by reaching autodetach time after finished move
		//Serial.print("e02 sequence error, servo not attached "); Serial.print(servoName); Serial.println();
		attach();
	}

	targetPos = adjustOutlierPosition(targetPos);

	// encountered problems (power off of group) when multiple request
	// came in sequence and arrivedMillis was not set, causing power group to get powered off
	startMillis = millis();		// for realtime log
	//arrivedMillis = millis() + duration;
	inMoveRequest = true;

	if (targetPos == lastPosition ) {
		// ignore move command to current position
		nextPos = lastPosition;		// make sure to report last position in servo update
		if (thisServoVerbose) {
			Serial.print("i01 request for move to current position, request ignored ");
			Serial.print(servoName);
			Serial.println();
		}
		// send target reached message to controller
		sendServoStatus(pin, lastPosition, assigned, moving, servo.attached(), autoDetachMs > 0, thisServoVerbose, true);
		return;
	}


	numIncrements = duration / 20;
	increment = (targetPos - lastPosition) / float(numIncrements);
	nextPos = lastPosition;

	moving = true;
	lastStatusUpdate = millis();

	if (thisServoVerbose) {
		Serial.print("v01 ");
		Serial.print(servoName);
		Serial.print(", a"); Serial.print(arduinoId);
		Serial.print(", moveTo"); 
		Serial.print(", pin: "); Serial.print(pin);
		Serial.print(", targ: "); Serial.print(targetPos);
		Serial.print(", dur: "); Serial.print(duration);
		Serial.print(", start: "); Serial.print(nextPos);
		Serial.print(", numInc: "); Serial.print(numIncrements);
		Serial.print(", inc: "); Serial.print(increment);
		Serial.println();
	}
}


// inverted flag is only treated here, do not include it in position calculation
void Mai3Servo::writeServoPosition(int position, bool inverted) {

	if (thisServoVerbose)  {
		Serial.print(millis()-startMillis); Serial.print("ms ");
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

#ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char* sbrk(int incr);
#else  // __ARM__
extern char *__brkval;
#endif  // __arm__
 
int freeMemory() {
  char top;
#ifdef __arm__
  return &top - reinterpret_cast<char*>(sbrk(0));
#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
  return &top - __brkval;
#else  // __arm__
  return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#endif  // __arm__
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

	// send current position and status over serial with limited interval
	if ((int(nextPos) != loggedLastPos) && (millis() - lastStatusUpdate > 90)) {
		sendServoStatus(pin, nextPos, assigned, moving, servo.attached(), autoDetachMs, thisServoVerbose, false);
		loggedLastPos = int(nextPos);
		lastStatusUpdate = millis();

		if (thisServoVerbose) {
			Serial.print(servoName); Serial.print(", m99 currentPos: "); Serial.print(nextPos);
			//Serial.print(", freeMemory: "); Serial.print(freeMemory());
			Serial.println();
		}
	}


	// check for target reached
	if (moving && numIncrements <= 0) {
		moving = false;
		arrivedMillis = millis();
		lastPosition = round(nextPos);		// the assumed reached position

		if (verbose || thisServoVerbose) {
			Serial.print("i11 target reached "); Serial.print(servoName);
			Serial.print(", position: "); Serial.print(lastPosition);
			Serial.println();
		}
		sendServoStatus(pin, lastPosition, assigned, moving, servo.attached(), autoDetachMs > 0, thisServoVerbose, true);

		return;
	}

	// detach with delay after we should have arrived at target position
	if (inMoveRequest && (autoDetachMs > 0)) {

		// check for arrivedMillis in the future, do nothing if so
		//if (arrivedMillis > millis()) return;
		/* {
			
			if (thisServoVerbose) {
				Serial.print("i12 detach servo "); Serial.print(servoName);
				Serial.print(", arrivedMillis ("); Serial.print(arrivedMillis); Serial.print(" in the future");
				Serial.print(" , currMillis: "); Serial.print(millis());
				Serial.println();
			}
			return;
		}
		*/
		// check for move ended and autoDetachMs expired
		if ((!moving) && ((millis() - arrivedMillis) > autoDetachMs)) {

			// set servo's inMoveRequest to false
			inMoveRequest = false;

			if (thisServoVerbose) {
				Serial.print("i13 servo "); Serial.print(servoName);
				Serial.print(" inMoveRequest cleared, autoDetachMs "); Serial.print(autoDetachMs);
				Serial.print(" ms after arrived: "); Serial.print((millis()-arrivedMillis));
				Serial.println();
			}
			//sendServoStatus(pin, nextPos, assigned, moving, servo.attached(), autoDetachMs, thisServoVerbose);
			return;
		}
	}


	// if still in move set next servo partial target position
	if (numIncrements > 0) {
		if (thisServoVerbose) {
			Serial.print("numIncrements: "); Serial.print(numIncrements);
			Serial.print(", nextPos: "); Serial.print(nextPos); Serial.println();
		}
		lastPosition = round(nextPos);		// set last position as the assumed reached position
		nextPos += increment;				// ... and nextPos as the next step position
		numIncrements -= 1;
		writeServoPosition(nextPos, inverted);
	}

}


