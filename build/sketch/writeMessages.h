// writeMessages.h

#ifndef _WRITEMESSAGES_h
#define _WRITEMESSAGES_h
#endif

extern char msg[100];
byte buildStatusByte(bool assigned, bool moving, bool attached, bool autoDetach, bool verbose, bool targetReached);
void sendServoStatus(byte pin, byte status, byte currentPosition);
void sendFeedbackStatus(byte pin, byte status, byte currentPosition, int ms, byte servoWritePosition, byte wantedPosition);
