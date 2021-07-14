// 
// 
// 
#include <Arduino.h>

#include "writeMessages.h"

byte statusMsg[10];

byte buildStatusByte(bool isAssigned, bool isMoving, bool isAttached, bool isAutoDetach, bool isVerbose, bool hasTargetReached) {
	byte statusByte = 0x80;
	if (isAssigned)       { statusByte = statusByte | 0x01; }
	if (!hasTargetReached) {
		if (isMoving)         { statusByte = statusByte | 0x02; }
	}
	if (isAttached)       { statusByte = statusByte | 0x04; }
	if (isAutoDetach)     { statusByte = statusByte | 0x08; }
	if (isVerbose)        { statusByte = statusByte | 0x10; }
	if (hasTargetReached) { statusByte = statusByte | 0x20; }
	return statusByte;
}

void sendServoStatus(byte pin, byte status, byte currentPosition) {
	// servo status will be sent every 20 ms for moving servos
	// as all servos could be moving at the same time the message needs to be as short as possible
	// therefore pack info tightly and avoid a generated \n in the data bytes as it would terminate 
	// the readline of the receiver

	statusMsg[0] = 0xC0 | pin;		// marker for compressed status message
	statusMsg[1] = status;
	statusMsg[2] = 0x10 + currentPosition;		// add offset to position to avoid 0x0A as byte value

	// send binary data bytewise
	for (int i=0; i<3; i++) {
		Serial.write(statusMsg[i]);
	}
	Serial.write(0x0A);		// newline as terminator
}


void sendFeedbackStatus(byte pin, byte status, byte currentPosition, int ms, byte servoWritePosition, byte wantedPosition) {

	// in order to avoid sending termination value 0x0A add an offset of 4112 to int values
	// and 0x10 to byte values

	int codedInt;

	statusMsg[0] = 0xC0 | pin;		// marker for compressed status message
	statusMsg[1] = status;
	statusMsg[2] = 0x10 + currentPosition;		// add offset to position to avoid 0x0A as byte value

	// ms since move start 
	codedInt = ms + 4112;		// 4096 + 16
	statusMsg[3] = codedInt >> 8;
	statusMsg[4] = codedInt & 0x00FF;

	statusMsg[5] = 0x10 + servoWritePosition;
	statusMsg[6] = 0x10 + wantedPosition;

	// send binary data bytewise
	for (int i=0; i<7; i++) {
		Serial.write(statusMsg[i]);
	}
	Serial.write(0x0A);		// newline as terminator
}
