#include "Interface.h"
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

#define OLED_RESET 4  //Reset value for OLED
#define GPSSerial Serial3   //UART connected to GPS
#define GMT_OFFSET -5   //Time offset from UTCG
#define DST true    //Daylight savings time
#define HOUR_FORMAT 12    //12 or 24 hour time (Improperly executed)

//Define display object
Adafruit_SSD1306 display(OLED_RESET);

//Check to ensure the proper display has been selected
#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

// OLED Display Area Definitions
#define padding 1
#define line1 17
#define line2 29
#define line3 41
#define line4 53

//Motor Control Pin Definitions
#define pin_A1 22
#define pin_A2 24
#define pin_B1 26
#define pin_B2 28
#define en_A 6
#define en_B 7
#define X0_LIMIT 42       //Pin for min translational limit switch
#define X1_LIMIT 40       //Pin for max translational limit switch

//Cooling fan definitions
#define fanPin 11
#define FAN_DUTY 75       //Duty cycle (%)
#define fanPeriod 60      //Period of fan cycle (sec)
#define fanSpeed 75       //Speed of fan (%)

//Motor movement definitions
#define dutyCycle 80      //PWM Duty cycle for motors (%)
#define standbyDuty 60    //PWM Duty cycle for low hold power (%)
#define stepDelay_Calibrate 40   //Time between motor steps during calibration (uSec)
#define stepDelay_Sequence 200    //Time between motor steps during capture (uSec)
#define stepDelay_Move 1          //Time between motor steps during transport (uSec)

#define pwmRange 255
#define TABLE_SIZE 128 //Table Size = # of microsteps * 4

#define CALIBRATION_PAUSE 200 //Pause time for calibration direction changing (mSec)
#define CALIBRATION_OFFSET 25 //Distance to move carriage away from limit switch
#define X_COEFFICIENT 40      //Conversion from steps to (1" = 100 steps)
#define TIME_MULTIPLIER 1000  //Conversion from milliseconds to seconds
#define FREE_MULTIPLIER 100   //Multiplier applied to encoder for "Free Input" actions
#define MIN2SEC 60            //Conversion from minutes to seconds

//How often to update the timestamp
#define TIME_UPDATE 30000 //Clock update interval (milliseconds)

//Encoder Definition
#define A 18    //Encoder pin A
#define B 19    //Encoder pin B
#define S 2     //Encoder pushbutton
#define bounceTime 40   //Debouncing delay for encoder
#define selectBounceTime 400    //Debouncing delay for encoder button
#define resetTime 400           //Error checking for encoder direction

//Go Button Definition
#define G 3                     //Button pin 
#define BUTTON_LED 12           //Button LED pin
#define PULSE_PERIOD 50         //Loop cycles for pulsing the LED
#define buttonBounceTime 500   //Debouncing delay for button

#define TRIGGER_PIN 44  //Pin to trigger camera capture
#define FOCUS_PIN 45    //Pin to trigger camera focus

#define MAIN 0          //    0   Main
#define READY 1         //    1   Ready
#define MOVEMENT 2      //    2   Movement
#define DISPLACEMENT 3  //    3   Displacement
#define TIME 4          //    4   Time
#define DRIVE 5         //    5   Drive
#define TRANSLATION 6   //    6   Translation
#define ROTATION 7      //    7   Rotation
#define INIT_POS 8      //    8   Initial Position
#define TRAV_DIR 9      //    9   Travel Direction
#define CALIBRATE 10    //    10  Calibrate
#define CAMERA 11       //    11  Camera
#define MAN_CONTROL 12  //    12  Manual Control
#define FREE_MOVE 13    //    13  Free Move
#define FREE_ROTATE 14  //    14  Free Rotate
#define TOTAL_DISP 15   //    15  Total Displacement
#define INC_DISP 16     //    16  Incremental Displacement
#define TOTAL_ROT 17    //    17  Total Rotation
#define INC_ROT 18      //    18  Incremental Rotation
#define INIT_X 19       //    19  Initial X
#define INIT_R 20       //    20  Initial R
#define X_TRAVEL 21     //    21  X-Travel
#define R_TRAVEL 22     //    22  R-Travel
#define TIME_VAL 23     //    23  Time Value
#define IMAGES 24       //    24  Images
#define INTERVAL 25     //    25  Interval
#define TOTAL_TIME 26   //    26  Total Time
#define CAL_X 27        //    27  Calibrate X
#define CAL_R 28        //    28  Calibrate R
#define HOLD 29         //    29  Hold
#define NLL 30          //    30  Null

#define MENU_TYPE 1     //    Menu
#define OPTION_TYPE 2   //    Option
#define ACTION_TYPE 3   //    Action
#define FREE_TYPE 4     //    Free
#define NULL_TYPE 5     //    Null

int windUp[100];

//Define array holding text lines (Y-pixel positions) of OLED screen
static const int line[] = { line1, line2, line3, line4 };

Menu main = Menu();


//The following arrays (title, type, children, value) apply to menus displayed
//on the OLED screen.  The indices defined below apply throughout all arrays,
//ex. title[0], type[0], children[0], value[0] all apply to the main menu.
//(Note: values for menu types are placeholders only to fill the array and
//are unused)
//
//    0   Main
//    1   Ready
//    2   Movement
//    3   Displacement
//    4   Time
//    5   Drive
//    6   Translation
//    7   Rotation
//    8   Initial Position
//    9   Travel Direction
//    10  Calibrate
//    11  Camera
//    12  Manual Control
//    13  Free Move
//    14  Free Rotate
//    15  Total Displacement
//    16  Incremental Displacement
//    17  Total Rotation
//    18  Incremental Rotation
//    19  Initial X
//    20  Initial R
//    21  X-Travel
//    22  R-Travel
//    23  Time Value
//    24  Images
//    25  Interval
//    26  Total Time
//    27  Calibrate X
//    28  Calibrate R
//    29  Hold
//    30  Null

//Titles of menus
static const String title[] =
{ "Main",                   "Ready",                "Movement",
"Displacement",           "Time",                 "Drive",
"Translation",            "Rotation",             "Init Pos",
"Travel Dir",             "Calibrate",            "Camera",
"Manual Control",         "Free Move",            "Free Rotate",
"Total Disp",             "Inc Disp",             "Total Rot",
"Inc Rot",                "Initial X",            "Initial R",
"X-Travel",               "R-Travel",             "Time Value",
"Images",                 "Interval",             "Total Time",
"Calibrate X",            "Calibrate R",          "Hold",
"Null"
};

//Menu types (Null types are not displayed)
//   1 Menu
//   2 Option
//   3 Action
//   4 Free
//   5 Null
static const int type[] =
{ MENU_TYPE,        //    0
ACTION_TYPE,      //    1
MENU_TYPE,        //    2
MENU_TYPE,        //    3
MENU_TYPE,        //    4
MENU_TYPE,        //    5
MENU_TYPE,        //    6
MENU_TYPE,        //    7
MENU_TYPE,        //    8
MENU_TYPE,        //    9
MENU_TYPE,        //    10
MENU_TYPE,        //    11
MENU_TYPE,        //    12
FREE_TYPE,        //    13
FREE_TYPE,        //    14
OPTION_TYPE,      //    15
OPTION_TYPE,      //    16
OPTION_TYPE,      //    17
OPTION_TYPE,      //    18
OPTION_TYPE,      //    19
OPTION_TYPE,      //    20
OPTION_TYPE,      //    21
OPTION_TYPE,      //    22
OPTION_TYPE,      //    23
OPTION_TYPE,      //    24
OPTION_TYPE,      //    25
OPTION_TYPE,      //    26
ACTION_TYPE,      //    27
ACTION_TYPE,      //    28
OPTION_TYPE,      //    29
NULL_TYPE         //    30
};

//Menu children (only Menu types will display a list of children).  The parent menu is
//the only non-Null child of Option and Action types (Camera is an Option type) and
//is not displayed in favor of relevant information pertaining to the current menu.
static const int children[][4] =
{ { 1, 2, 4, 5 },       //Main Items - Ready, Movement, Time, Drive
{ 0, 30, 30, 30 },    //Ready Items - Main, Null, Null, Null
{ 3, 8, 9, 0 },       //Movement Items - Displacement, Init Pos, Travel Direction, Main
{ 6, 7, 2, 30 },      //Displacement Items - Translation, Rotation, Movement, Null
{ 26, 25, 11, 0 },    //Time Items - Total Time, Interval, Camera, Main
{ 10, 12, 29, 0 },    //Drive Items - Calibrate, Manual Control, Hold, Main
{ 15, 16, 3, 30 },    //Translation Items - Total Disp, Incremental Disp, Displacement, Null
{ 17, 18, 3, 30 },    //Rotation Items - Total Rotation, Incremental Rot, Displacement, Null
{ 19, 20, 2, 30 },    //Initial Position Items - Initial X, Initial R, Movement, Null
{ 21, 22, 2, 30 },    //Travel Direction Items - X-Travel, R-Travel, Movement, Null
{ 27, 28, 5, 30 },    //Calibrate Items - Calibrate X, Calibrate R, Drive, Null
{ 23, 24, 4, 30 },    //Camera Items - Time Value, Images, Time, Null
{ 13, 14, 5, 30 },    //Manual Control Items - Free Move, Free Rotate, Drive, Null
{ 12, 30, 30, 30 },   //Free Move Items - Manual Control, Null, Null, Null
{ 12, 30, 30, 30 },   //Free Rotate Items - Manual Control, Null, Null, Null
{ 6, 30, 30, 30 },    //Total Displacement Items - Translation, Null, Null, Null
{ 6, 30, 30, 30 },    //Incremental Displacement Items - Translation, Null, Null, Null
{ 7, 30, 30, 30 },    //Total Rotation Items - Rotation, Null, Null, Null
{ 7, 30, 30, 30 },    //Incremental Rotation Items - Rotation, Null, Null, Null
{ 8, 30, 30, 30 },    //Initial X Items - Initial Pos, Null, Null, Null
{ 8, 30, 30, 30 },    //Initial R Items - Initial Pos, Null, Null, Null
{ 9, 30, 30, 30 },    //X-Travel Items - Travel Direction, Null, Null, Null
{ 9, 30, 30, 30 },    //R-Travel Items - Travel Direction, Null, Null, Null
{ 11, 30, 30, 30 },   //Time Value Items - Camera, Null, Null, Null
{ 11, 30, 30, 30 },   //Images Items - Camera, Null, Null, Null
{ 4, 30, 30, 30 },    //Interval Items - Time, Null, Null, Null
{ 4, 30, 30, 30 },    //Total Time Items - Time, Null, Null, Null
{ 10, 30, 30, 30 },   //Calibrate X Items - Calibrate, Null, Null, Null
{ 10, 30, 30, 30 },   //Calibrate R Items - Calibrate, Null, Null, Null
{ 5, 30, 30, 30 },    //Hold Items - Drive, Null, Null, Null
{ 30, 30, 30, 30 },   //Null Items - Null, Null, Null, Null
};

//Menu value (values of menu types are arbitrary and unused)
int value[] =
{ 0,        //    0   Main
2,        //    1   Ready
0,        //    2   Movement
0,        //    3   Displacement
0,        //    4   Time
0,        //    5   Drive
0,        //    6   Translation
0,        //    7   Rotation
0,        //    8   Initial Position
1,        //    9   Travel Direction
0,        //    10  Calibrate
0,        //    11  Camera
0,        //    12  Manual Control
0,        //    13  Free Move
0,        //    14  Free Rotate
1150,     //    15  Total Displacement (millimeters)
5,        //    16  Incremental Displacement (millimeters)
5000,     //    17  Total Rotation (degrees)
50,       //    18  Incremental Rotation (degrees)
0,        //    19  Initial X (units)
0,        //    20  Initial R (units)
0,        //    21  X-Travel
0,        //    22  R-Travel
13,       //    23  Time Value (milliseconds)
20,       //    24  Images
3,        //    25  Interval  (seconds)
15,       //    26  Total Time (minutes)
0,        //    27  Calibrate X
0,        //    28  Calibrate R
0,        //    29  Hold
0         //    30  Null
};


//  0  Normal
//  1  ON/LOW/HIGH/AUTO
//  2  Fractional (1/4, etc..)
//  3  FWD/REV
//  4  Standard value with increment multiplier


static const String textMenu1[] = { "OFF", "LOW", "HIGH", "AUTO" };
static const String textMenu2[] = { "30", "25", "20", "15", "10", "5", "2", "1", "1/2", "1/3", "1/4", "1/5", "1/10", "1/20", "OFF" };
static const String textMenu3[] = { "FWD", "REV" };

static const int tv[] = { 30000, 25000, 20000, 15000, 10000, 5000, 2000, 1000, 500, 333, 250, 200, 100, 50, 0 };

//Direction of travel multiplier
static const int Direction[] = { 1,-1 };

static const int alternateMenu[] =
{ 0,        //    0   Main
0,        //    1   Ready
0,        //    2   Movement
0,        //    3   Displacement
0,        //    4   Time
0,        //    5   Drive
0,        //    6   Translation
0,        //    7   Rotation
0,        //    8   Initial Position
0,        //    9   Travel Direction
0,        //    10  Calibrate
0,        //    11  Camera
0,        //    12  Manual Control
0,        //    13  Free Move
0,        //    14  Free Rotate
4,        //    15  Total Displacement (1/100 inch)
0,        //    16  Incremental Displacement (1/100 inch)
0,        //    17  Total Rotation (degrees)
0,        //    18  Incremental Rotation (degrees)
0,        //    19  Initial X (units)
0,        //    20  Initial R (units)
3,        //    21  X-Travel
3,        //    22  R-Travel
2,        //    23  Time Value (milliseconds)
0,        //    24  Images
0,        //    25  Interval  (seconds)
0,        //    26  Total Time (minutes)
0,        //    27  Calibrate X
0,        //    28  Calibrate R
1,        //    29  Hold
0         //    30  Null
};

static const String subTitle[] =
{ "",        //    0   Main
"IMG",     //    1   Ready
"",        //    2   Movement
"",        //    3   Displacement
"",        //    4   Time
"",        //    5   Drive
"",        //    6   Translation
"",        //    7   Rotation
"",        //    8   Initial Position
"",        //    9   Travel Direction
"",        //    10  Calibrate
"",        //    11  Camera
"",        //    12  Manual Control
"Set",     //    13  Free Move
"Set",     //    14  Free Rotate
"",        //    15  Total Displacement (millimeters)
"",        //    16  Incremental Displacement (millimeters)
"",        //    17  Total Rotation (degrees)
"",        //    18  Incremental Rotation (degrees)
"",        //    19  Initial X (units)
"",        //    20  Initial R (units)
"",        //    21  X-Travel
"",        //    22  R-Travel
"",        //    23  Time Value (milliseconds)
"",        //    24  Images
"",        //    25  Interval  (seconds)
"",        //    26  Total Time (minutes)
"",        //    27  Calibrate X
"",        //    28  Calibrate R
"",        //    29  Hold
""         //    30  Null
};

//Least to most significant:
//    0   A
//    1   B
//    2   Aval
//    3   Bval
//    4   encFlag (needed?)
//    5   selectFlag
//    6   startFlag
byte interruptFlag = B00000000;

//Least to most significant:
//    0   Images
//    1   Incremental Displacement
//    2   Total Displacement
//    3   Incremental Rotation
//    4   Total Rotation
//    5   Interval
//    6   Total Time
//    7
//Second most recent
//    8   Images
//    9   Incremental Displacement
//    10  Total Displacement
//    11  Incremental Rotation
//    12  Total Rotation
//    13  Interval
//    14  Total Time
//    15
word mostRecent = 0;
static const int mR[] =
{ 7,        //    0   Main
7,        //    1   Ready
7,        //    2   Movement
7,        //    3   Displacement
7,        //    4   Time
7,        //    5   Drive
7,        //    6   Translation
7,        //    7   Rotation
7,        //    8   Initial Position
7,        //    9   Travel Direction
7,        //    10  Calibrate
7,        //    11  Camera
7,        //    12  Manual Control
7,        //    13  Free Move
7,        //    14  Free Rotate
2,        //    15  Total Displacement (millimeters)
1,        //    16  Incremental Displacement (millimeters)
4,        //    17  Total Rotation (degrees)
3,        //    18  Incremental Rotation (degrees)
7,        //    19  Initial X (units)
7,        //    20  Initial R (units)
7,        //    21  X-Travel
7,        //    22  R-Travel
7,        //    23  Time Value (milliseconds)
0,        //    24  Images
5,        //    25  Interval  (seconds)
6,        //    26  Total Time (minutes)
7,        //    27  Calibrate X
7,        //    28  Calibrate R
7,        //    29  Hold
7         //    30  Null
};

volatile long timeLast;   //Last time an event was triggered
int pulseDutyCycle = 0;
long beginPulse = 0;
bool pulsePause = false;

String gpsData;
String dataType;
String timeStamp;
long lastUpdate = 0;
long lastTimeUpdate = -30000;
long positionLast = 0;
int dstShift;
int currentMenu = 0;
int selector = 0;
int displayRange[] = { 0, 3 };

int direc;
int pwmATable[TABLE_SIZE];
int pwmBTable[TABLE_SIZE];
static const int halfSize = TABLE_SIZE / 2;
unsigned long Position = 0;
unsigned long X_MAX;
bool flag = true;
//short stepTable [TABLE_SIZE][4];
//static const int stepWord = B1001101001100101;
int stepWord[4][4] = {
	{ 1,0,1,0 },
	{ 0,1,1,0 },
	{ 0,1,0,1 },
	{ 1,0,0,1 }
};
int indexer[TABLE_SIZE];
int stepCounter = 0;
int drive = (dutyCycle / 200.0) * pwmRange;
int hold = (standbyDuty / 100.0) * pwmRange;
long fanEpoch;
byte fanState;
int fanDuty = FAN_DUTY;

void setup() {
	TCCR1B = (TCCR1B & 0b11111000) | 0x04;  //timer 1 (controls pin 12, 11)
	TCCR2B = (TCCR2B & 0b11111000) | 0x01;  //timer 2 (controls pin 10, 9)
	TCCR4B = (TCCR4B & 0b11111000) | 0x01;  //timer 4 (controls pin 8, 7, 6)

	pinMode(A, INPUT_PULLUP);
	pinMode(B, INPUT_PULLUP);
	pinMode(S, INPUT_PULLUP);
	pinMode(G, INPUT_PULLUP);
	pinMode(X0_LIMIT, INPUT_PULLUP);
	pinMode(X1_LIMIT, INPUT_PULLUP);

	attachInterrupt(digitalPinToInterrupt(A), pin_A, CHANGE);
	attachInterrupt(digitalPinToInterrupt(B), pin_B, CHANGE);
	attachInterrupt(digitalPinToInterrupt(S), select, FALLING);
	attachInterrupt(digitalPinToInterrupt(G), hitGo, FALLING);

	pinMode(BUTTON_LED, OUTPUT);

	pinMode(fanPin, OUTPUT);
	pinMode(en_A, OUTPUT);
	pinMode(en_B, OUTPUT);
	pinMode(pin_A1, OUTPUT);
	pinMode(pin_A2, OUTPUT);
	pinMode(pin_B1, OUTPUT);
	pinMode(pin_B2, OUTPUT);

	pinMode(FOCUS_PIN, OUTPUT);
	pinMode(TRIGGER_PIN, OUTPUT);
	digitalWrite(FOCUS_PIN, HIGH);
	digitalWrite(TRIGGER_PIN, HIGH);

	analogWrite(fanPin, 255);
	//Serial.println("Build Step Table");
	buildStepTable(TABLE_SIZE);

	while (!Serial);
	// 9600 baud is the default rate for the Ultimate GPS
	GPSSerial.begin(9600);
	Serial.begin(250000);

	if (DST) dstShift = 1;
	else dstShift = 0;

	//Start OLED display
	Serial.println("Init Display");
	display.begin(SSD1306_SWITCHCAPVCC, 0x3C);    //Initialize with I2C address 0x3C

	for (int i = 0; i < 100; i++) {
		windUp[i] = 200 - i * 2;
	}
	Serial.println("Init Calibration Sequence");
	calibrateX();
	Serial.println("Make Menu");
	bitSet(mostRecent, mR[TOTAL_TIME]);
	updateValues();
	makeMenu(currentMenu);    //Display the main menu
}

int n = 0;

void loop() {
	getTime();
	makeMenu(currentMenu);
	buttonPulse();
	encoderEvent();
	checkSelect();
	checkGo();
	cycleFan();
}

//Button LED Pulsing
void buttonPulse() {
	if (beginPulse > PULSE_PERIOD) {
		beginPulse = 0;
		pulsePause = !pulsePause;
	}
	pulseDutyCycle = 255 * sin((PI / PULSE_PERIOD) * beginPulse);
	beginPulse++;
	if (!pulsePause) {
		analogWrite(BUTTON_LED, pulseDutyCycle);
	}
}

//Caller for timed event menu updating
//Implimentation pending a functioning event timer.
void menUpdater() {
	makeMenu(currentMenu);
	return;
}

void calibrationScreen() {
	display.clearDisplay();
	display.setTextSize(2);
	display.setTextColor(WHITE);
	display.setCursor(padding + 6, line1);
	display.println("CALIBRATE");
	display.display();
}

void makeMenu(int menuNum) {
	display.clearDisplay();
	display.setTextSize(1);
	display.setTextColor(WHITE);
	display.setCursor(padding, 0);
	display.println(title[menuNum]);
	display.setCursor(96, 0);
	display.println(timeStamp);
	switch (type[menuNum])
	{
	case 1:
		if (selector > displayRange[1]) {
			displayRange[1] = selector;
			if (displayRange[1] - 3 >= 0) {
				displayRange[0] = displayRange[1] - 3;
			}
			else {
				displayRange[0] =
			}
		}
		if (selector < displayRange[0]) {
			displayRange[0] = selector;
			displayRange[1] = displayRange[0] + 3;
		}
		for (int i = displayRange[0]; i <= displayRange[1]; i++) {
			if (title[children[menuNum][i]] != "Null") {
				if (i == selector) {
					display.setTextColor(BLACK);
					for (int t = 0; t < 9; t++) {
						display.drawFastHLine(0, line[i] - 1 + t, title[children[menuNum][i]].length() * 6 + 1, 1);
					}
				}
				else {
					display.setTextColor(WHITE);
				}
				display.setCursor(padding, line[i]);
				display.println(title[children[menuNum][i]]);
			}
		}
		break;
	case 2:
		switch (alternateMenu[menuNum])
		{
		case 0:
			//Normal Value Display
			display.setCursor(padding, 30);
			display.setTextSize(2);
			display.println(value[menuNum]);
			break;
		case 1:
			//Hold power value display
			display.setCursor(padding, 30);
			display.setTextSize(2);
			display.println(textMenu1[value[menuNum]]);
			break;
		case 2:
			//Shutter speed value display
			display.setCursor(padding, 30);
			display.setTextSize(2);
			display.println(textMenu2[value[menuNum]]);
			break;
		case 3:
			//Travel direction value display
			display.setCursor(padding, 30);
			display.setTextSize(2);
			display.println(textMenu3[value[menuNum]]);
			break;
		case 4:
			//Normal Value Display with Multiplier
			display.setCursor(padding, 30);
			display.setTextSize(2);
			display.println(value[menuNum]);
			break;
		default:
			//This shit shouldn't happen
			display.setCursor(padding, 30);
			display.setTextSize(2);
			display.println("ERROR");
			break;
		}
		break;
	case 3:
		display.setCursor(padding, line1);
		display.setTextSize(2);
		display.println(subTitle[menuNum]);
		display.setCursor(64, line1);
		display.println(value[menuNum]);
		display.setCursor(padding, line3);
		display.println("Pos");
		display.setCursor(64, line3);
		display.println(Position / X_COEFFICIENT);
		break;
	case 4:
		display.setCursor(padding, line1);
		display.setTextSize(2);
		display.println(subTitle[menuNum]);
		display.setCursor(64, line1);
		display.println(value[menuNum]);
		display.setCursor(padding, line3);
		display.println("Pos");
		display.setCursor(64, line3);
		display.println(Position / X_COEFFICIENT);
		break;
	default:
		//This shit shouldn't happen
		break;
	}
	display.display();
}

void pin_A() {
	if (millis() - timeLast < bounceTime) return;
	timeLast = millis();
	if (bitRead(interruptFlag, 4))return;
	bitSet(interruptFlag, 4);
	bitSet(interruptFlag, 0);
	bitWrite(interruptFlag, 2, digitalRead(A));
	bitWrite(interruptFlag, 3, digitalRead(B));
}

void pin_B() {
	if (millis() - timeLast < bounceTime) return;
	timeLast = millis();
	if (bitRead(interruptFlag, 4))return;
	bitSet(interruptFlag, 4);
	bitSet(interruptFlag, 1);
	bitWrite(interruptFlag, 2, digitalRead(A));
	bitWrite(interruptFlag, 3, digitalRead(B));
}

void encoderEvent() {
	if (!bitRead(interruptFlag, 4)) return;
	bool Aint = bitRead(interruptFlag, 0);
	bool Bint = bitRead(interruptFlag, 1);
	bool Aval = bitRead(interruptFlag, 2);
	bool Bval = bitRead(interruptFlag, 3);
	clearInterruptBits();
	if (Aint) {
		if ((Aval && !Bval) || (!Aval && Bval)) {
			selectorInc(1);
			Aint = 0;
			Bint = 0;
			Aval = 0;
			Bval = 0;
			return;
		}
		else if ((Aval && Bval) || (!Aval && !Bval)) {
			selectorInc(-1);
			Aint = 0;
			Bint = 0;
			Aval = 0;
			Bval = 0;
			return;
		}
	}
	if (Bint) {
		if ((Aval && Bval) || (!Aval && !Bval)) {
			selectorInc(1);
			Aint = 0;
			Bint = 0;
			Aval = 0;
			Bval = 0;
			return;
		}
		else if ((Aval && !Bval) || (!Aval && Bval)) {
			selectorInc(-1);
			Aint = 0;
			Bint = 0;
			Aval = 0;
			Bval = 0;
			return;
		}
	}
}

void clearInterruptBits() {
	for (int i = 0; i < 8; i++) {
		bitClear(interruptFlag, i);
	}
}

void selectorInc(int dir) {
	if (type[currentMenu] == MENU_TYPE) {
		int validMenus = -1;
		for (int k = 0; k < 4; k++) {
			if (type[children[currentMenu][k]] != NULL_TYPE) {
				validMenus += 1;
			}
		}
		if (selector + dir >= 0 && selector + dir <= validMenus) {
			selector += dir;
			return;
		}
		else if (selector + dir < 0) {
			selector = validMenus;
			return;
		}
		else {
			selector = 0;
			return;
		}
	}
	if (type[currentMenu] == OPTION_TYPE) {
		if (value[currentMenu] + dir >= 0) {
			switch (alternateMenu[currentMenu]) {
			case 0:
				value[currentMenu] += dir;
				break;
			case 1:
				//Increment Hold Values
				if (value[currentMenu] + dir < sizeof(textMenu1) / sizeof(String)) {
					value[currentMenu] += dir;
				}
				else {
					value[currentMenu] = 0;
				}
				break;
			case 2:
				//Increment Shutter Speed Values
				if (value[currentMenu] + dir < sizeof(textMenu2) / sizeof(String)) {
					value[currentMenu] += dir;
				}
				else {
					value[currentMenu] = 0;
				}
				break;
			case 3:
				//Increment Direction Values
				if (value[currentMenu] + dir < sizeof(textMenu3) / sizeof(String)) {
					value[currentMenu] += dir;
				}
				else {
					value[currentMenu] = 0;
				}
				break;
			case 4:
				//Increment Value with Multiplier
				if (value[currentMenu] + dir * FREE_MULTIPLIER >= 0) {
					value[currentMenu] += dir * FREE_MULTIPLIER;
				}
				break;
			default:
				//This shit shouldn't happen
				display.setCursor(padding, 30);
				display.setTextSize(2);
				display.println("ERROR");
				break;
			}
			//value[currentMenu] += dir;
			return;
		}
		else {
			value[currentMenu] = 0;
		}
	}
	if (type[currentMenu] == FREE_TYPE) {
		if (value[currentMenu] + dir * FREE_MULTIPLIER >= 0 && value[currentMenu] + dir * FREE_MULTIPLIER <= X_MAX) {
			value[currentMenu] += dir * FREE_MULTIPLIER;
			return;
		}
	}
}

void select() {
	if (millis() - timeLast < selectBounceTime) return;
	timeLast = millis();
	//if (bitRead(interruptFlag, 5))return;
	bitSet(interruptFlag, 5);
}

void checkSelect() {
	if (!bitRead(interruptFlag, 5))return;
	clearInterruptBits();
	bitSet(mostRecent, mR[currentMenu]);
	updateValues();
	currentMenu = children[currentMenu][selector];
	selector = 0;
}

void updateValues() {
	if (bitRead(mostRecent, 0) || bitRead(mostRecent, 1)) {
		value[TOTAL_DISP] = value[IMAGES] * value[INC_DISP];
		bitWrite(mostRecent, 8, bitRead(mostRecent, 0));
		bitWrite(mostRecent, 9, bitRead(mostRecent, 1));
		bitClear(mostRecent, 0);
		bitClear(mostRecent, 1);
	}

	if (bitRead(mostRecent, 0) || bitRead(mostRecent, 2)) {
		value[INC_DISP] = value[TOTAL_DISP] / value[IMAGES];
		bitWrite(mostRecent, 8, bitRead(mostRecent, 0));
		bitWrite(mostRecent, 10, bitRead(mostRecent, 2));
		bitClear(mostRecent, 0);
		bitClear(mostRecent, 2);
	}

	if (bitRead(mostRecent, 0) || bitRead(mostRecent, 3)) {
		value[TOTAL_ROT] = value[IMAGES] * value[INC_ROT];
		bitWrite(mostRecent, 8, bitRead(mostRecent, 0));
		bitWrite(mostRecent, 11, bitRead(mostRecent, 3));
		bitClear(mostRecent, 0);
		bitClear(mostRecent, 3);
	}

	if (bitRead(mostRecent, 0) || bitRead(mostRecent, 4)) {
		value[INC_ROT] = value[TOTAL_ROT] / value[IMAGES];
		bitWrite(mostRecent, 8, bitRead(mostRecent, 0));
		bitWrite(mostRecent, 12, bitRead(mostRecent, 4));
		bitClear(mostRecent, 0);
		bitClear(mostRecent, 4);
	}

	if (bitRead(mostRecent, 5) || bitRead(mostRecent, 6)) {
		if (value[INTERVAL] > 0) {
			value[IMAGES] = (value[TOTAL_TIME] * MIN2SEC) / value[INTERVAL];
		}
		else {
			value[IMAGES] = value[TOTAL_TIME] * MIN2SEC;
		}
		value[TOTAL_DISP] = value[IMAGES] * value[INC_DISP];
		value[TOTAL_ROT] = value[IMAGES] * value[INC_ROT];
		bitWrite(mostRecent, 13, bitRead(mostRecent, 5));
		bitWrite(mostRecent, 14, bitRead(mostRecent, 6));
		bitClear(mostRecent, 5);
		bitClear(mostRecent, 6);
	}

	if (bitRead(mostRecent, 5) || bitRead(mostRecent, 0)) {
		value[TOTAL_TIME] = (value[TOTAL_TIME] * value[INTERVAL]) * MIN2SEC;
		value[TOTAL_DISP] = value[IMAGES] * value[INC_DISP];
		value[TOTAL_ROT] = value[IMAGES] * value[INC_ROT];
		bitWrite(mostRecent, 13, bitRead(mostRecent, 5));
		bitWrite(mostRecent, 8, bitRead(mostRecent, 0));
		bitClear(mostRecent, 5);
		bitClear(mostRecent, 0);
	}

	value[READY] = value[IMAGES];
	setHold(value[HOLD]);
}

void hitGo() {
	if (millis() - timeLast < buttonBounceTime) return;
	timeLast = millis();
	//if (bitRead(interruptFlag, 6))return;
	bitSet(interruptFlag, 6);
	bitClear(interruptFlag, 7);
}

void checkGo() {
	if (!bitRead(interruptFlag, 6))return;
	clearInterruptBits();
	updateValues();
	if (type[currentMenu] != ACTION_TYPE && type[currentMenu] != FREE_TYPE) {
		currentMenu = READY;
		selector = 0;
	}
	else {
		runSequence(currentMenu);
		Serial.println("Exit 3");
	}
}

void runSequence(int menu) {
	switch (menu) {
	case 1:
		//Capture sequence
		bitSet(interruptFlag, 7);
		captureSequence();
		Serial.println("Exit 2");
		break;
	case 13:
		//Translate to target
		freeMove();
		break;
	case 14:
		//Rotate to target
		break;
	case 27:
		//Calibrate translation
		calibrateX();
		break;
	case 28:
		//Calibrate rotation
		break;
	default:
		//Do something to indicate erroneous sequence entry
		break;
	}
}

void captureSequence() {
	setFan(80);
	moveTo(value[INIT_X] * X_COEFFICIENT);
	for (int i = value[IMAGES]; i > 0; i--) {
		unsigned long sleepStart = millis();
		if (!bitRead(interruptFlag, 7)) {
			clearInterruptBits();
			Serial.println("Exit 1");
			break;
		}
		if (value[INTERVAL]) {
			capture();
		}
		moveSteps(value[INC_DISP] * X_COEFFICIENT);
		while (millis() - sleepStart < value[INTERVAL] * TIME_MULTIPLIER && value[INTERVAL] > 0) {
			cycleFan();
			getTime();
			menUpdater();
			if (!bitRead(interruptFlag, 7)) {
				break;
			}
		}
	}
	value[READY] = value[IMAGES];
	setFan(80);
	moveTo(value[INIT_X] * X_COEFFICIENT);
}

void capture() {
	buttonLED(1);
	focusTrigger(0);
	delay(10);
	camTrigger(0);
	unsigned long exposureStart = millis();
	while (millis() - exposureStart < tv[value[TIME_VAL]]);
	buttonLED(0);
	focusTrigger(1);
	camTrigger(1);
	value[READY] -= 1;
	menUpdater();
	delay(50);
}

void freeMove() {
	moveTo(long(value[currentMenu])*X_COEFFICIENT);
}

void buttonLED(int state) {
	digitalWrite(BUTTON_LED, state);
}

void camTrigger(int state) {
	digitalWrite(TRIGGER_PIN, state);
}

void focusTrigger(int state) {
	digitalWrite(FOCUS_PIN, state);
}

void getTime() {
	if (millis() - lastTimeUpdate > TIME_UPDATE) {
		int invalid = -1;
		int last = -1;
		int comma1;
		int comma2;
		int comma3;
		int comma4;
		int comma5;
		int comma6;
		int comma7;
		//GPS Time Parsing
		while (invalid < 0) {
			while (GPSSerial.available()) {
				char c = GPSSerial.read();
				if (c == '$') gpsData = "";
				gpsData += c;
			}
			invalid = gpsData.indexOf('$');
		}

		while (last < 0) {
			while (GPSSerial.available()) {
				char c = GPSSerial.read();
				gpsData += c;
			}
			last = gpsData.indexOf('*');
		}

		comma1 = gpsData.indexOf(',');
		comma2 = gpsData.indexOf(',', comma1 + 1);
		dataType = gpsData.substring(0, comma1);

		if (dataType == "$GPRMC") {
			String gpsTime = gpsData.substring(comma1 + 1, comma2);
			String rawHour = gpsTime.substring(0, 2);
			int hour = (rawHour.toInt() + GMT_OFFSET + dstShift) % HOUR_FORMAT;
			if (hour < 0) hour = 12 + hour;
			if (hour == 0) hour = 12;
			String minute = gpsTime.substring(2, 4);
			//String second = gpsTime.substring(4, 6);
			timeStamp = String(hour) + ":" + minute;  // + ":" + second;
		}
	}
}

void calibrateX() {
	setFan(fanSpeed);
	calibrationScreen();
	long t;
	while (!digitalRead(X0_LIMIT)) {
		setStepper(-1);
		t = micros();
		while (micros() - t < stepDelay_Calibrate);
	}
	delay(CALIBRATION_PAUSE);
	for (int i = 0; i < CALIBRATION_OFFSET * X_COEFFICIENT; i++) {
		setStepper(1);
		t = micros();
		while (micros() - t < stepDelay_Calibrate);
	}
	Position = 0;
	Serial.println("Position Zero");
	while (!digitalRead(X1_LIMIT)) {
		setStepper(1);
		Position += 1;
		t = micros();
		while (micros() - t < stepDelay_Calibrate);
	}
	delay(CALIBRATION_PAUSE);
	for (int i = 0; i < CALIBRATION_OFFSET * X_COEFFICIENT; i++) {
		setStepper(-1);
		Position -= 1;
		t = micros();
		while (micros() - t < stepDelay_Calibrate);
	}
	X_MAX = Position / X_COEFFICIENT;
	Serial.print("Position max: ");
	Serial.println(Position);
	moveTo(Position / 2);
}

void buildStepTable(int tableSize) {

	for (int i = 0; i < (tableSize / 4) * 1; i++) {
		indexer[i] = 0;
	}

	for (int i = (tableSize / 4) * 1; i < (tableSize / 4) * 2; i++) {
		indexer[i] = 1;
	}

	for (int i = (tableSize / 4) * 2; i < (tableSize / 4) * 3; i++) {
		indexer[i] = 2;
	}

	for (int i = (tableSize / 4) * 3; i < (tableSize / 4) * 4; i++) {
		indexer[i] = 3;
	}

	for (int i = 0; i < tableSize; i++) {
		pwmATable[i] = fabs(cos(i * (PI / halfSize))) * drive + 255 * 0.45;
		pwmBTable[i] = fabs(sin(i * (PI / halfSize))) * drive + 255 * 0.45;
	}
}

void setStepper(int dir) {
	if (flag)
	{
		if (stepCounter + dir < TABLE_SIZE) {
			if (stepCounter + dir >= 0) {
				stepCounter += dir;
			}
			else {
				stepCounter = TABLE_SIZE - 1;
			}
		}
		else {
			stepCounter = 0;
		}
		analogWrite(en_A, pwmATable[stepCounter]);
		digitalWrite(pin_A1, stepWord[indexer[stepCounter]][0]);
		digitalWrite(pin_A2, stepWord[indexer[stepCounter]][1]);
		analogWrite(en_B, pwmBTable[stepCounter]);
		digitalWrite(pin_B1, stepWord[indexer[stepCounter]][2]);
		digitalWrite(pin_B2, stepWord[indexer[stepCounter]][3]);
	}
}

void moveTo(long target) {
	long pos = Position;
	int stepDelay = stepDelay_Calibrate;

	if (target < 0) {
		value[X_TRAVEL] = 0;
		return;
	}
	if (target / X_COEFFICIENT > X_MAX) {
		value[X_TRAVEL] = 1;
		return;
	}
	if (target == Position) return;
	if (Position < target) direc = 1;
	else direc = (-1);

	if (fabs(target - pos) / X_COEFFICIENT <= 100) {
		stepDelay = stepDelay_Sequence;
	}

	if (fabs(target - pos) / X_COEFFICIENT > 1000) {
		stepDelay = stepDelay_Move;
	}
	unsigned long range = fabs(target - pos);

	for (unsigned long i = 0; i < range; i++)
	{
		setStepper(direc);
		Position += direc;

		if (i < range / 4 && i < 1000) {
			delayMicroseconds(stepDelay + windUp[i / 10]);
		}
		else if (range - i < range / 4 && range - i < 1000) {
			delayMicroseconds(stepDelay + windUp[(range - i) / 10]);
		}
		else {
			delayMicroseconds(stepDelay);
		}
		if (digitalRead(X0_LIMIT) || digitalRead(X1_LIMIT)) break;
	}
	analogWrite(en_A, hold);
	analogWrite(en_B, hold);
}

void setHold(int level) {
	int holdA;
	int holdB;
	switch (level) {
	case 0:
		//Turn it off
		hold = 0;
		holdA = hold;
		holdB = hold;
		fanDuty = 10;
		break;
	case 1:
		//Low Power
		hold = (50 / 100.0) * pwmRange;
		holdA = hold;
		holdB = hold;
		break;
	case 2:
		//High Power
		hold = (standbyDuty / 100.0) * pwmRange;
		holdA = hold;
		holdB = hold;
		fanDuty = FAN_DUTY;
		break;
	default:
		//Something else (Including Auto until accelerometer is mounted
		holdA = pwmATable[stepCounter];
		holdB = pwmBTable[stepCounter];
		break;
	}
	analogWrite(en_A, holdA);
	analogWrite(en_B, holdB);
}

void moveSteps(long distance) {
	moveTo(Position + distance * Direction[value[X_TRAVEL]]);
}

void motorOff() {
	analogWrite(en_A, 0);
	analogWrite(en_B, 0);
}

void setFan(int dC) {
	int fnspd = pwmRange - (dC / 100.0) * pwmRange;
	analogWrite(fanPin, fnspd);
}

void cycleFan() {
	if (fanState == B0)
	{
		if (millis() - fanEpoch > (1.0 - fanDuty / 100.0) * fanPeriod * 1000)
		{
			setFan(fanSpeed);
			fanState = B1;
			fanEpoch = millis();
		}
	}
	else
	{
		if (millis() - fanEpoch > fanDuty / 100.0 * fanPeriod * 1000)
		{
			setFan(0);
			fanState = B0;
			fanEpoch = millis();
		}
	}
}