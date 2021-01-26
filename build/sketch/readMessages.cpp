
#include <Arduino.h>
#include "readMessages.h"

char receivedChars[64];
int numChars = 0;
bool newData = false;
byte ndx = 0;

bool log_r0 = false;

// fill buffer with message
// set newData flag to true, data is processed in inmoovArduino

void recvWithEndMarker() {
	char rc;

	while (Serial.available() > 0 && newData == false) {
		rc = Serial.read();
		
		if (rc == 0) continue;		// do not know why I get zero values ???

		if (rc != '\n') {
			receivedChars[ndx] = rc;
			ndx++;
			if (ndx >= 60) {
				ndx = 59;
			}
			numChars = ndx;
		}
		else {
			receivedChars[ndx] = '\0'; // terminate the string
			ndx = 0;
			newData = true;
		}
	}
}



int checkCommand() {

	if (Serial.available() > 0) {
		recvWithEndMarker();
		//standalone: SerialMonitor, select Line Feed and send command, 
		// e.g. 1,1,200,2000 for go forward 2000 mm with speed 200
	}

	if (newData) {

		char mode = receivedChars[0];
		strncpy(msgCopyForParsing, receivedChars, numChars);
		if (log_r0) {
			Serial.print("r00 received chars: "); Serial.print(receivedChars); Serial.println();
			Serial.print("r01 msgCopyForParsing: "); Serial.print(msgCopyForParsing); Serial.println();
		}
		newData = false;

		return mode;
	}
	else {
		return 'x';
	}
}
