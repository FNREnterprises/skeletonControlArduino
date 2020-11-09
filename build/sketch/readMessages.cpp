
#include <Arduino.h>
#include "readMessages.h"

char receivedChars[64];

bool newData = false;


// fill buffer with message
// set newData flag to true, data is processed in inmoovArduino

void recvWithEndMarker() {
	static byte ndx = 0;
	char endMarker = '\n';
	char rc;

	while (Serial.available() > 0 && newData == false) {
		rc = Serial.read();

		if (rc != endMarker) {
			receivedChars[ndx] = rc;
			ndx++;
			if (ndx >= 60) {
				ndx = 59;
			}
		}
		else {
			receivedChars[ndx] = '\0'; // terminate the string
			ndx = 0;
			newData = true;
		}
	}
}



int checkCommand() {

	int numBytes;
	if (Serial.available() > 0) {
		recvWithEndMarker();
		//standalone: SerialMonitor, select Line Feed and send command, 
		// e.g. 1,1,200,2000 for go forward 2000 mm with speed 200
	}

	if (newData) {

		char mode = receivedChars[0];
		//strlcpy(msgCopyForParsing, receivedChars, sizeof(msgCopyForParsing));
		strcpy(msgCopyForParsing, receivedChars);
		//Serial.print("msg received: "); Serial.print(msgCopyForParsing); Serial.println();
		newData = false;

		return mode;
	}
	else {
		return 'x';
	}
}
