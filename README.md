# skeletonControlArduino
arduino code for servo control with JuergInMoov software

Cabling
2 arduinos for servo control

when looking at Marvin's back 
the left Arduino controls the left arm and the head
the right Arduino controls the right arm

Arduino ports are connected to a displayport-terminalblock
Exceptions: 
servos in the head are directly connected to the ports
head up/down is also directly connected to 

Displayport-Cable with mini-connector (DELOCK, Reichelt DE)
Terminalblock 22 Pin, DELOCK 65394, Reichelt

Right Side, Right Arduino, COM6 , /dev/ttyA1
Arduino
Pin and
Servo

Cable
HDMI
HDMI
Arduino- Termina Ader
>
l
Terminal

HDMI
Ader

1
7 Wrist

blue

HDMI
Cable
Arduino
Termina Arduino- Pin and
l
>
Servo
Terminal
GND

2

BROWN

GND

22
black
Omoplat
e

3

WHITE/
brown
[White]

20

10
purple
Shoulder

4

RED
[black]

19

9 Rotate pink

5

BROWN
/white
[Brown]

YELLOW 18
/white

Black

6 Pinky

24 Bicep yellow

6

WHITE

RED/wh 17
ite

White

5 Ring

WHITE/ 16
yellow

purple

4
Majeure

green

3 Index

Yellow

2 Thumb

7
8

YELLOW

WHITE/ 15
red

9

WHITE/
green

14

10

WHITE/
blue

13

11

White/
GREEN

BLUE/w 12
hite

Left Side, Left Arduino, COM, /dev/ttyA2

Arduino	Kabel	HDMI  Pin and Arduino- Termina Ader
servo>Terminal 8 black

HDMI
Ader

HDMI Kabel
Arduino
Termina Arduino- Pin and
l
>
Servo
Terminal

1

GND

2

GND

3

WHITE/
InMoov Marvin Page 1

20

8
black
Omoplat
e

3

WHITE/
brown

9
white
Shoulde

4

RED

BLACK

19

blue

2 Wrist

10
Rotate

5

BROWN
/white

YELLOW
/white

18

green

3 Pinky

6

WHITE

RED/wh
te

17

yellow

4 Ring

7

WHITE/
yellow

16

pink

5
Majeure

8

WHITE/
ed

15

red

6 Index

brown

7 Thumb

grey

11 Bicep purple

20

9

14

10

13

11

BLUE/w
hite

Direct Connections 4 wire head cable with arduino left:
22 EyeX

white

23 EyeY

green

24 Jaw

red

25

black

Direct cable Arduino->Servo
21 Neck orange
50 - GND Arduino

InMoov Marvin Page 2

12

Verkabelung Kopf (linker Arduino)
Donnerstag, 31. Dezember 2015
10:39

Androi

Farbe

Funktion

Rest

Min

Max

12

Weiss/Gelb

KopfY,neck

90

20

160

13

weiss

KopfX,rothea

90

40

140

22

rot

eyeX

80

60

100

24

gelb

eyeY

90

40

70

26

blau

Mund

10

10

25

InMoov Marvin Page 3

Verkabelung Torso (rechter Arduino)
Montag, 13. Juni 2016
15:19

Androi

Farbe

27
28

weiss

Funktion

Rest

Min

Max

Torso bend 90

20

160

Torso

40

140

90

InMoov Marvin Page 4

Verkabelung Lagesensor bno055
Montag, 21. Dezember 2015
10:54

Name

Nano

Kabelfarb Bno055

SCL

A5

Gelb

SCL

SDA

A4

Pink

SDA

5V

Braun

Vin

GND

Schwarz

GND

Pin

Mode

Funktion

Displayport-

Nano

D8

DigIn

Activate

GRÜN/weiss

blau

D9

PWM

Pitch servo

D10

PWM

Roll servo

weiss
schwarz

gelb

InMoov Marvin Page 5

Bending Wrist Pulley
Mittwoch, 23. Dezember 2015
09:55

offset

Nach der Drehung um 90 grad wickle ich somit die blaue Strecke ab
und wickle 1/4 Umfang + offset auf (grau)?
Dass sich hier auch eine neue X-Position ergibt ignoriere ich mal weil der Einfluss auf das Resultat
ziemlich klein ist (der Kanal der Seile ist ca. 150 mm vom Pulley entfernt)
Annäherungsmässig könnte ich auch sagen die blaue Strecke ist 1/4 Umfang - Offset?
Damit ergibt sich nach obiger Logik dann ungefähr:
Für links mit r = 15 und offset = 5 -> (3.14 * 15 / 2) - 5 = 18.55
Und für rechts -> 3.14*15/2 + 5 = 28.55
Ich könnte nun einfach im Excel verschiedene Werte für r und offset durchrechnen lassen um ein
passendes Verhältnis zu finden?
Das Verhältnis der Bogenlinien-Längen auf dem 1/4 Umfang welche durch den offset entstehen
man aber wohl genauer berechnen (blau/grün)?

InMoov Marvin Page 6

Hilfsmittel
Montag, 4. Januar 2016
16:15

Steckerteile (Muffen) auf Pins
Segor.de

InMoov Marvin Page 7

Bereiche Servos
Dienstag, 12. Januar 2016
18:06

Hand Rechts, 15.01.2015
Finger

Close

Open

Rest

Thumb

20

180

90

Index

0

163

90

Majeur

0

180

90

Ring

0

170

90

pinky

30

180

90

Wrist

InMoov Marvin Page 8

Arduino, Sensor
Freitag, 22. Januar 2016
19:31

Arduino Mega/Due 2580
ACHTUNG: auf dem InMoov Rücken ist der Arduino gedreht!

Arduino Nano für Hand, China 2 Euro
Orientation,
Adafruit BNo055, ca. CHF 40.-, einfache SW mit library
MPU 9250, GY-91, ca. 7 Euro AliExpress, programmierung?
