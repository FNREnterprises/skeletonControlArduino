

char version[10] = "v2.11";

/*
 Name:		inmoovArduino.ino
 Created:	01.09.2018 13:59:51
 Author:	juerg.maier

 This is software that runs on an arduino mega.
 For other boards you might need to adjust the number of servos that can be handled

 It is using the arduino servo library to control the servos, making it easy to understand and modify
 and does not limit number of servos to PWM ports.

 A request to move a servo has to be in the range of 0..180. 
 It is up to the servoController to request only allowed positions or to translate degree positions into
 this range.

 All move requests need to be timed in milliseconds and the servo increments are devided into 20 ms sub steps
 The "standard" inmoovServoControl software expects servo definitions including maximum move speed.
 inmoovServoControl will increase requested move times if given move time is below the servo spec

 It is expected that the controlling application maintains persistance of last position for each servo.

 Before a move can be requested the servo needs to be assigned to its pin, given the last known position and
 whether autoDetach should be performed (detaching after move).

 BE CAREFUL: 
 "requested position reached" is based on the given move duration. As (at least in current implementation)
 we do not have a feedback from the servo about its current position the servo might get detached too early
 for your servo to actually reach the requested position due to an unrealistic given move duration (a heavy
 load will slow down servo performance).
 
 When restarting the program the last known position of the servo is unknown and has to be provided with
 the assign command. Failure to do so will result in wrong servo moves.

 For safety reasons (programming errors?) a min and max position can be passed to this program and any request
 for moves below min or above max will be limited to min/max. If your physical setup requires limits based on
 degrees the controlling task needs to convert these into the relative servo limits (0..180).
 e.g. a servo having a total degree range of 320 that should not move below 90 degrees should be given a min value of
 (320/2-90) * (180/320) = 39
 It is strongly adviced to save the last reached servo position in the controlling program and 
 provide this last setting in the assign command as is the case when using inmoovServoControl with this program

 With autoDetachMs > 0 the servo will be detached when no movement is required. This avoids noise and overheating but
 might cause the external force to move the passive servo. It is strongly requested to have a mechanical setup that avoids a 
 servo repositioning on its own when not controlled (using additional gears or springs).

 BE CAREFUL: 
 With autoDetachMs set to 0 you can easily burn your servo. Reduce this risk by adding a thermoresistor (thermistor) into the 
 power loop of the servo. It will prevent the overload of the servo and make the servo available again when it has cooled down.
 The termoresistor needs to be chosen with respect to the power consumption of the servo. A small thermoresistor might prevent
 moving of larger servos after a short moving time, a big thermoresistor will allow to burn a small servo before it starts to 
 limit the current.

 This arduino sketch is primarily meant to work with the python inmoovServoControl task. There is also a Gui task available: inmoovServoGui.
 Other tasks can be used to connect to the inmooServoControl using rpyc commands.
  
 inmoovServoControl loads the servo parameters from a json file. It can receive servo commands over rpyc from a higher-level control instance 
 e.g. like inmoovServoGui or any other controller you create. See inmoovServoControl readme for further information

 The inmoovServoGui task shows the current servo stati and allows to manually control the servos using inmoovServoControl. 

 This is a private setup and comes with no guarantee to work as described, feel free to adjust it for your own needs.

 =======================================================================================================

 Accepted commands:
 servoAssign:  0,<servoId>,<pin>,<minPosition>,<maxPosition>,<autoDetachMs>,<inverted><lastPos>
	servoId: unique servoId per Arduino from inmoovServoControl.servoList
	minPosition: a value between 0 and 180, degrees to position calculation has to be done by caller
	maxPosition: a value between 0 and 180, minPosition has to be lower than maxPosition
	autoDetachMs: after target reached this is the wait time until detach of the servo, 0 for never detach
	inverted: before servo.write() position will be subtracted from 180 making the servo move opposite

 servoMoveTo:  1,<servoId>,<position>,<duration>
	servoId: unique servoId per Arduino from inmoovServoControl.servoList
	position: a value between 0 and 180, degrees to position calculation done in inmoovServoControl
		the code checks for requests < minPosition, > maxPosition and limits value accordingly
	duration: ms for the move. servos move in 20 ms steps from current position to target position

stop servo: 2,<servoId>
	servoId: unique servoId per Arduino from inmoovServoControl.servoList
		detaches the servo

stop all servos: 3
		detaches all assigned servos of the arduino

report servo status to caller: 4,<servoId>
	servoId: unique servoId per Arduino from inmoovServoControl.servoList
		sends an update message containing servoId, currentPos, assigned, moving, attached, autoDetachMs, verbose

update autoDetachMs: 5,<servoId>,<milliseconds>
	servoId: unique servoId per Arduino from inmoovServoControl.servoList
		updates the autoDetachMs value of the servo, 0 prevents detach after move

update lastPosition without moving: 6,<servoId>,<position>
	servoId: unique servoId per Arduino from inmoovServoControl.servoList
		if physical servo position does not match the lastPosition value in the arduino the lastPosition is overwritten

change log level for servo: 7,<servoId>,<newState>
	servoId: unique servoId per Arduino from inmoovServoControl.servoList
		sets the verbose flag of a servo
			setting the verbose flag to true may impact the timing



logs:
control with log_r0 in readMessages
r00 received characters in readMessages
r01 parsing buffer of message

e01 no action, servo not assigned yet
e02 no action, servo not attached
e03 servo set verbose for unknown servo

e04 maxPosition < minPosition
e06 moveTo received but servo is not attached

w01 requested position smaller than min
w02 requested position greater than max
w03 new move request while still in move 

i01 request to move to current position
i10 request to move to new position
i11 target reached
i12 arrived time in the future
i13 detach servo
i14 set servo last position before powerup

i20 new autoDetach value received 
i21 servo stop received
i22 stop all servos received

i30 digital pin set to HIGH
i31 digital pin set to LOW 
 
i40 power pin test in setup
i41 powergroup ON/OFF

i50 log of incoming messages
i51 temporary logs for debugging

// logs for servos with servoVerbose set
v01 move to request
============================================================================================================ */

bool exec_i40=false;
bool log_i41=true;
bool log_i10=true;
bool log_i50=true;
bool log_i51=true;

#include "Arduino.h"

#include <Wire.h>
#include "Servo.h"

//#define NULL 0
#define SERVO_POWER_OFF HIGH
#define SERVO_POWER_ON LOW

#include "Mai3Servo.h"
#include "readMessages.h"
#include "writeMessages.h"

bool verbose = false;

const int NUMBER_OF_SERVOS = 20;		// max number of servos
Mai3Servo servoList[NUMBER_OF_SERVOS];
int servoIdOfPinList[NUMBER_OF_SERVOS];	// list of servoId for assigned pin

const int NUMBER_OF_POWER_PINS = 8;	// number of power sections
typedef struct {
	int powerPin;
	bool powerOn;
	char powerGroupName[20];
} powerGroupType;

powerGroupType powerGroup[NUMBER_OF_POWER_PINS] = {
	{14, false, "IN1-leftArm"},
	{15, false, "IN2-leftHand"},
	{16, false, "IN3-rightArm"},
	{17, false, "IN4-rightHand"},
	{18, false, "IN5-head"},
	{19, false, "IN6-torso"},
	{20, false, "unused"},
	{20, false, "unused"}
};

char mode = 'x';
int ledToggle = 0;

char msgCopyForParsing[64];
char msg[100];

int arduinoId = 0;
int assignedServos = 0;

int highMillis;
int lowMillis;
unsigned long lastServoUpdateMillis = millis();
unsigned long ledToggleMillis = millis();

// the setup function runs once when you press reset, power the board or open the serial connection
void setup() {

	//Serial.begin(115200);
	Serial.begin(115200);
	delay(500);

	// check for arduino Id
	// Arduino 0 (left) has a connection of Pin 50 with Ground
	// read pin 50, in left arduino it is connected to ground
	pinMode(50, INPUT_PULLUP);
	int signal = digitalRead(50);
	if (signal == 0) {
		arduinoId = 0;
	}
	else {
		arduinoId = 1;
	}	
	// respond with either S0 or S1 as ready response
	Serial.print("S"); Serial.print(arduinoId);
	Serial.print(" skeletonControlArduino "); Serial.println(version);


	pinMode(LED_BUILTIN, OUTPUT);

	// the power pins, set all power off
	// this is run on both arduinos so assign pins 40..46 on both arduinos to power control only
	for (int powerGroupIndex = 0; powerGroupIndex < NUMBER_OF_POWER_PINS; powerGroupIndex++) {

		// ATTENTION: relais board did not work with 6V power supply, connect board vcc to arduino due 5v!
		digitalWrite(powerGroup[powerGroupIndex].powerPin, SERVO_POWER_OFF);		// should switch relais off (apply before setting pinmode!)
		pinMode(powerGroup[powerGroupIndex].powerPin, OUTPUT);
	}
	delay(1000);

	// show different loop frequency on arduinos
	if (arduinoId == 0) {
		highMillis = 100;
		lowMillis = 500;
	}
	else {
		highMillis = 500;
		lowMillis = 100;
	}

	if (exec_i40) {
		// set each powergroup on for 1 sec in setup
		for (int powerGroupIndex = 0; powerGroupIndex < NUMBER_OF_POWER_PINS; powerGroupIndex++) {

			Serial.print("i40 powerPin ON:  "); Serial.println(powerGroup[powerGroupIndex].powerPin);
			digitalWrite(powerGroup[powerGroupIndex].powerPin, SERVO_POWER_ON);		// test power on
			delay(2000);
			Serial.print("i40 powerPin OFF: "); Serial.println(powerGroup[powerGroupIndex].powerPin);
			digitalWrite(powerGroup[powerGroupIndex].powerPin, SERVO_POWER_OFF);	// test power off
			powerGroup[powerGroupIndex].powerOn = false;		
		}
	}
}


// each servo assign adds the servo to the servoList
// as commands are given for a pin a translation list pin -> servoId is used
int servoIdOfPin(int pin) {
	int servoId = -1;
	for (int i = 0; i < NUMBER_OF_SERVOS; i++) {
		if (servoIdOfPinList[i] == pin) {
			return i;
		}
	}
	return servoId;
}

// any move request for a servo has to check for current servo group power
// if the servo group power is currently off, all servos in the group
// need to be set to their "lastPosition" and  need to be attached
// NOTE: powerOff is handled in loop
void powerUpServoGroup(int servoId) {

	// find powerGroupIndex with same power pin as the assigned power pin of the servo
	for (int powerGroupIndex=0; powerGroupIndex < NUMBER_OF_POWER_PINS; powerGroupIndex++)  {

		if (powerGroup[powerGroupIndex].powerPin == servoList[servoId].servoPowerPin) {

			// check servo's powerGroup state
			if (powerGroup[powerGroupIndex].powerOn) {
				if (verbose) {
					Serial.print("power already on for power group "); Serial.print(powerGroup[powerGroupIndex].powerGroupName);
					Serial.print(" by "); Serial.print(servoList[servoId].servoName);
					Serial.println();
				}
			} else {

				// activate power relais
				if (log_i51) {
					Serial.print("i51 powerUpServoGroup for servoId: ");Serial.print(servoId);
					Serial.print(", powerPin: "); Serial.print(servoList[servoId].servoPowerPin);
					Serial.println();
				}
				pinMode(powerGroup[powerGroupIndex].powerPin, OUTPUT);
				digitalWrite(powerGroup[powerGroupIndex].powerPin, SERVO_POWER_ON);
				powerGroup[powerGroupIndex].powerOn = true;

				// power up all servos in this power group
				for (int s = 0; s < assignedServos; s++) {
					if (servoList[s].servoPowerPin == powerGroup[powerGroupIndex].powerPin) {
						servoList[s].powerUp();
					}
				}

				delay(50);
			}
		}
	}
}

// check for possible power off for an active servo group
bool hasPowerGroupActiveMovements(int powerGroupIndex) {

	bool active = false;

	for (int s = 0; s < assignedServos; s++) {
		if (servoList[s].servoPowerPin == powerGroup[powerGroupIndex].powerPin) {
			if (servoList[s].inMoveRequest) {
				active = true;
			};
		}
	}
	return active;
}


// servo assign
// 0,<servoName>,<pin>,<min>,<max>,<autoDetachMs>,<inverted>,<lastPos>,<servoPowerPin>
void servoAssign() {

	char * strtokIndx; // this is used by strtok() as an index
	char servoName[20] = {0};

	strtokIndx = strtok(msgCopyForParsing, ","); // first item
	int cmd = atoi(strtokIndx);			// cmd for servo assign = 0

	strtokIndx = strtok(NULL, ",");		// position for next list item
	strcpy(servoName, strtokIndx);		// the servo Name

	strtokIndx = strtok(NULL, ",");		// position for next list item
	int pin = atoi(strtokIndx);			// pin number for servo position write

	strtokIndx = strtok(NULL, ",");		// position for next list item
	int min = atoi(strtokIndx);			// the lowest requestable position value for this servo

	strtokIndx = strtok(NULL, ",");		// position for next list item
	int max = atoi(strtokIndx);         // the highest requestable position value for this servo

	strtokIndx = strtok(NULL, ",");		// position for next list item
	int autoDetachMs = atoi(strtokIndx); // number of millis to keep servo attached after move end

	strtokIndx = strtok(NULL, ",");		// position for next list item
	int inverted = atoi(strtokIndx);    // if true the requested position will be subtracted from 180

	strtokIndx = strtok(NULL, ",");		// position for next list item
	int lastPos = atoi(strtokIndx);     // the last known position of this servo (maintained by servoControl task)

	strtokIndx = strtok(NULL, ",");		// position for next list item
	int servoPowerPin = atoi(strtokIndx);    // the power pin the servo is attached to

	// check for pin already assigned
	int servoId = servoIdOfPin(pin);

	// append list of servoId / pin relation
	if (servoId == -1) {				// pin not assigned yet
		servoId = assignedServos;
		assignedServos += 1;
		servoIdOfPinList[servoId] = pin;
		if (log_i51) {
			Serial.print("i51 assigning servoId: "); Serial.print(servoId);
			Serial.print(" to pin: "); Serial.print(pin);
			Serial.println();
		}
	}

	strcpy(servoList[servoId].servoName, servoName);
	servoList[servoId].begin(pin, min, max, autoDetachMs, inverted, lastPos, servoPowerPin);

	//if (servoList[servoId].thisServoVerbose) {
	if (log_i51) {
		Serial.print("i51 servo begin, servoId: "); Serial.print(servoId);
		Serial.print(" , servoName: "); Serial.print(servoName);
		Serial.print(", pin: "); Serial.print(pin);
		Serial.print(", min: "); Serial.print(min);
		Serial.print(", max: "); Serial.print(max);
		Serial.print(", autoDetachMs: "); Serial.print(autoDetachMs);
		Serial.print(", inverted: "); Serial.print(inverted);
		Serial.print(", lastPos: "); Serial.print(lastPos);
		Serial.print(", servoPowerPin: "); Serial.print(servoPowerPin);
		Serial.println();
	}
	servoList[servoId].detachServo(true);
	sendServoStatus(pin, lastPos, true, false, true, autoDetachMs, verbose, true);

}

// servo move request
// 1,<pin>,<position>,<duration>
void servoMoveTo() {

	char * strtokIndx;					// this is used by strtok() as an index

	strtokIndx = strtok(msgCopyForParsing, ","); // first item
	int cmd = atoi(strtokIndx);			// command moveTo = 1

	strtokIndx = strtok(NULL, ",");		// position for next list item
	int pin = atoi(strtokIndx);    		// pin for move

	strtokIndx = strtok(NULL, ",");		// position for next list item
	int position = atoi(strtokIndx);    // requested position

	strtokIndx = strtok(NULL, ",");		// position for next list item
	int duration = atoi(strtokIndx);    // duration of move

	int servoId = servoIdOfPin(pin);

	// check for servo known
	if (servoId == -1) {
		Serial.print("move request for unassigned servo, pin: "); Serial.print(pin); Serial.println();
		return;
	}

	if (log_i10) {
		Serial.print("i10 "); Serial.print(servoList[servoId].servoName); 
		Serial.print(", servoMoveTo, pin: "); Serial.print(pin);
		Serial.print(", pos: "); Serial.print(position);
		Serial.print(", dur: "); Serial.print(duration);
		Serial.println();
	}
	powerUpServoGroup(servoId);
	
	// check for servo already in move and if so stop it first
	if (servoList[servoId].moving) {
		servoList[servoId].stopServo();
		if (servoList[servoId].thisServoVerbose) {
			Serial.print("w03 new moveTo position request while still moving, stop current move ");
			Serial.print(servoList[servoId].servoName);
			Serial.println();
		}
	}

	servoList[servoId].moveTo(position, duration);
}

// 
void servoStopCmd() {

	char * strtokIndx; // this is used by strtok() as an index

	strtokIndx = strtok(msgCopyForParsing, ","); // first item
	int cmd = atoi(strtokIndx);     	// command id

	strtokIndx = strtok(NULL, ",");		// next list item
	int pin = atoi(strtokIndx);     	// pin number

	int servoId = servoIdOfPin(pin);

	if (servoId == -1) {
		Serial.print("stop request for unassigned servo, pin: "); Serial.print(pin); Serial.println();
		return;
	}
	servoList[servoId].stopServo();

	if (verbose) {
		Serial.print("stopServo, servoId: "); Serial.print(servoId);
		Serial.print(", pin: "); Serial.print(pin);
		Serial.print(", "); Serial.print(servoList[servoId].servoName);
		Serial.println();
	}
}


void servoStopAllCmd() {

	Serial.println("i22 servo stop all received");

	// stop all servos
	for (int i = 0; i < assignedServos; i++) {
		servoList[i].stopServo();
	}
}


void reportServoStatus() {

	char * strtokIndx; // this is used by strtok() as an index

	strtokIndx = strtok(msgCopyForParsing, ","); // first item
	int cmd = atoi(strtokIndx);			// cmd

	strtokIndx = strtok(NULL, ",");		// next item
	int pin = atoi(strtokIndx);     // pin

	int servoId = servoIdOfPin(pin);

	if (servoId == -1) {
		Serial.print("report servo request for unassigned servo, pin: "); Serial.print(pin); Serial.println();
		return;
	}

	int position = servoList[servoId].lastPosition;
	bool assigned = servoList[servoId].assigned; 
	bool isMoving = servoList[servoId].moving;
	bool attached = servoList[servoId].attached();
	int autoDetachMs = servoList[servoId].autoDetachMs;
	bool verbose = servoList[servoId].thisServoVerbose;

	if (verbose) {
		Serial.print("servoStatus, servoId: "); Serial.print(servoId);
		Serial.print(", position: "); Serial.print(position);
		Serial.print(", assigned: "); Serial.print(assigned);
		Serial.print(", isMoving: "); Serial.print(isMoving);
		Serial.print(", attached: "); Serial.print(attached);
		Serial.println();
	}
	sendServoStatus(pin, position, assigned, isMoving, attached, autoDetachMs, verbose, false);
}


void setAutoDetach() {

	char * strtokIndx; // this is used by strtok() as an index

	strtokIndx = strtok(msgCopyForParsing, ","); // first item
	int cmd = atoi(strtokIndx);     	// command id

	strtokIndx = strtok(NULL, ",");		// next list element
	int pin = atoi(strtokIndx);     	// pin

	strtokIndx = strtok(NULL, ",");		// first item
	int newMs = atoi(strtokIndx);       // convert this part to an integer


	int servoId = servoIdOfPin(pin);
	if (servoId == -1) {
		Serial.print("setAutoDetach request for unassigned servo, pin: "); Serial.print(pin); Serial.println();
		return;
	}

	if (newMs != servoList[servoId].autoDetachMs) {

		// new autoDetach value
		servoList[servoId].autoDetachMs = newMs;

		if (verbose) {
			Serial.print("i20 new setAutoDetach value: "); Serial.print(newMs);
			Serial.print(" for "); Serial.print(servoList[servoId].servoName);
			Serial.println();
		}
	}
}


void setPosition() {
	// overwrite lastPosition of servo with given newPos
	// does not move the servo!

	char * strtokIndx; // this is used by strtok() as an index

	strtokIndx = strtok(msgCopyForParsing, ","); // first item
	int cmd = atoi(strtokIndx);			// convert this part to an integer

	strtokIndx = strtok(NULL, ",");		// next list item
	int pin = atoi(strtokIndx);     	// convert this part to an integer

	strtokIndx = strtok(NULL, ",");		// next list item
	int newPos = atoi(strtokIndx);      // convert this part to an integer

	int servoId = servoIdOfPin(pin);
	if (servoId == -1) {
		Serial.print("move request for unassigned servo, pin: "); Serial.print(pin); Serial.println();
		return;
	}

	servoList[servoId].lastPosition = newPos;
}


void setVerbose() {

	char * strtokIndx; // this is used by strtok() as an index

	strtokIndx = strtok(msgCopyForParsing, ","); // first item
	int cmd = atoi(strtokIndx);			// cmd

	strtokIndx = strtok(NULL, ",");		// next item
	int pin = atoi(strtokIndx);     	// pin

	strtokIndx = strtok(NULL, ",");		// next item
	int state = atoi(strtokIndx);       // verbose state

	int servoId = servoIdOfPin(pin);
	if (servoId == -1) {
		Serial.print("e04 set verbose request for unassigned servo, pin: "); Serial.print(pin); Serial.println();
		return;
	}

	if (state == 0) {
		servoList[servoId].thisServoVerbose = false;
	}
	else {
		servoList[servoId].thisServoVerbose = true;
	}
}

// "h,<pin number>,..<pin number>"
void pinHigh() {

	char * strtokIndx; // this is used by strtok() as an index

	strtokIndx = strtok(msgCopyForParsing, ","); // first item
	strtokIndx = strtok(NULL, ",");				 // command code h

	while (strtokIndx != NULL) {				// for each pin in the list
		int digitalPin = atoi(strtokIndx);      // pin number
		
		pinMode(digitalPin, OUTPUT);
		digitalWrite(digitalPin, HIGH);
		Serial.print("i30 digital pin set to HIGH: "); Serial.print(digitalPin); Serial.println();

		strtokIndx = strtok(NULL, ",");		// next item
	}
}

// "l,<pin number>,..<pin number>"
void pinLow() {

	char * strtokIndx; // this is used by strtok() as an index

	strtokIndx = strtok(msgCopyForParsing, ","); // first item, command code
	strtokIndx = strtok(NULL, ",");		// first item after command

	while (strtokIndx != NULL) {		// list of pins to set low
		int digitalPin = atoi(strtokIndx);     // convert this part to an integer

		Serial.print("i31 digital pin set to LOW: "); Serial.print(digitalPin); Serial.println();
		pinMode(digitalPin, OUTPUT);
		digitalWrite(digitalPin, LOW);

		strtokIndx = strtok(NULL, ",");		// next item
	}
}



// the loop function runs over and over again until power down or reset
void loop() {

	if (millis() - ledToggleMillis < highMillis) {
		digitalWrite(LED_BUILTIN, HIGH);
	}
	else {
		digitalWrite(LED_BUILTIN, LOW);
	}
	if ((millis() - ledToggleMillis) > (highMillis + lowMillis)) {
		ledToggleMillis = millis();
	}

	/////////////////////////////////////////////////////////////////////
	// for currently moving servos request the next incremental position
	/////////////////////////////////////////////////////////////////////
	// update servos max 50 times per second
	if ((millis() - lastServoUpdateMillis) >= 20) {
		for (int i = 0; i < assignedServos; i++) {
			servoList[i].update();
		}
		lastServoUpdateMillis = millis();
	}

	/////////////////////////////////////////////////////////////////////
	// for currently activated power groups check for possible power off
	/////////////////////////////////////////////////////////////////////
	for (int powerGroupIndex=0; powerGroupIndex < NUMBER_OF_POWER_PINS; powerGroupIndex++) {
		if (powerGroup[powerGroupIndex].powerOn) {
			if (!hasPowerGroupActiveMovements(powerGroupIndex)) {
				digitalWrite(powerGroup[powerGroupIndex].powerPin, SERVO_POWER_OFF);
				powerGroup[powerGroupIndex].powerOn = false;
				if (log_i41) {
					Serial.print("i41, servo group powered off "); Serial.print(powerGroup[powerGroupIndex].powerGroupName); Serial.println();
				}

				// detach servos in this power group
				for (int s = 0; s < assignedServos; s++) {
					if (servoList[s].servoPowerPin == powerGroup[powerGroupIndex].powerPin) {
						servoList[s].detachServo(true);		// force detach
					}
				}
			}
		}
	}

	///////////////////////////////////////////////
	// check for new requests over serial
	//////////////////////////////////////////////
	mode = checkCommand();

	if (log_i50 && mode != 'x') {
		Serial.print("i50 ");
		Serial.print(msgCopyForParsing);
		Serial.println();
	}

	switch (mode) {

	case 'x':
		break;

	case 'i':	// reply with ready message
		Serial.println("depricated request for arduinoId received");
		break;

	case '0':	// assign <servo>,<pin>,<min>,<max>
		servoAssign();
		break;

	case '1':	// move to absolute <servo>,<position>,<duration>
		servoMoveTo();
		break;

	case '2':	// stop servo
		servoStopCmd();
		break;

	case '3':	// stop all servos
		servoStopAllCmd();
		break;

	case '4':	// report servo status to caller <pin>
		reportServoStatus();
		break;

	case '5':	// update autoDetachMs (servo detached when not moving)
		setAutoDetach();
		break;

	case '6':	// update lastPosition without moving
		setPosition();
		break;

	case '7':	// change log level for servo
		setVerbose();
		break;

	case 'h':	// set pins high
		pinHigh();
		break;

	case 'l':	// set pins low
		pinLow();
		break;

	default:
		Serial.print("unknown mode: <"); Serial.print(mode); Serial.println(">");
	}
}
