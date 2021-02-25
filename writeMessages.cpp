// 
// 
// 
#include <Arduino.h>

#include "writeMessages.h"

char statusMsg[10];


void sendServoStatus(int pin, int pos, bool assigned, bool isRunning, bool attached, bool autoDetach, bool verbose, bool targetReached) {
	// servo status will be sent every 20 ms for moving servos
	// as all servos could be moving at the same time the message needs to be as short as possible
	// therefore pack info tightly and avoid a generated \n as it will terminate the message

	// 17.08.2020 Jürg, never ending problems trying to send the status massage as byte array
	// after about 10 servo requests the arduino stopped sending data or maybe python stopped receiving
	// this solution so far has worked now.

	Serial.print(0x80 | pin, HEX);

	Serial.print(0x28 + pos, HEX);
	
	byte statusByte = 0x80;
	if (assigned)      { statusByte = statusByte | 0x01; }
	if (isRunning)     { statusByte = statusByte | 0x02; }
	if (attached)      { statusByte = statusByte | 0x04; }
	if (autoDetach)    { statusByte = statusByte | 0x08; }
	if (verbose)       { statusByte = statusByte | 0x10; }
	if (targetReached) { statusByte = statusByte | 0x20; }

	Serial.print(statusByte, HEX);
	Serial.println(statusMsg);
	//Serial.flush();
}
