// writeMessages.h

#ifndef _WRITEMESSAGES_h
#define _WRITEMESSAGES_h
#endif

extern char msg[100];

void sendServoStatus(int pin, int pos, bool assigned, bool isRunning, bool attached, bool autoDetach, bool verbose, bool targetReached);
