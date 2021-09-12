/***********************************************************************************************
|  ____    _                        _           ____                       __     __  _
| / ___|  | |_    ___    __ _    __| |  _   _  |  _ \    __ _   ___   ___  \ \   / / / |
| \___ \  | __|  / _ \  / _` |  / _` | | | | | | |_) |  / _` | / __| / __|  \ \ / /  | |
|  ___) | | |_  |  __/ | (_| | | (_| | | |_| | |  __/  | (_| | \__ \ \__ \   \ V /   | |
| |____/   \__|  \___|  \__,_|  \__,_|  \__, | |_|      \__,_| |___/ |___/    \_/    |_|
|                                        |___/
|
************************************************************************************************
  ---|FILE: SteadyPassV1.ino
  ---|Initial document Created By: Dimitry
  ---|Description: This is the code used to make everything work
  ---|  Change History
  ---| <17.xx - Dimitry   - All versions Prior to 17.00 have been solely created by Dimitry
  ---|  17.xx - Sam James - Renamed File from "Limiter_16_53.ino", Added Title and Change History, Added Function comment and Descriptions, Minor code changes made that will not effect the main code, Remade speed and RPM calc into one function, Created knob and button switches. Long press, short press,knob up, and knob down. Added basic scroll menu.
  ---|  18.00 - Sam James - New Menu, PID functioning correctly, optimizations
  ---|  18.01 - ''''''''' - Screen redraw fix, changed description on min/max servo, Kpid settings default text changed, exit from any >>>
  ---|  18.02 - ''''''''' - menu with long press, PID now pulls range change without reboot, created settings initialization on first boot
  ---|  18.03 - ''''''''' - Fixed Time Zone on boot and in menu setting, fixed servo pos change on setting change
  ---|  18.04 - ''''''''' - BETA RELEASE
  ---|  18.05 - ''''''''' - Fixed Target speed displaying MPH numbers while in KPH mode, changes Initial boot setting to P=.37 I=.25 D=.08 for faster initial end user tuning.
  ---|  18.06 - ''''''''' - Fixed switching MPH to RPM mode automatic Kd set to 0.00
  ---|  18.08 - ''''''''' - Added Water, Air, and voltage readings back in and working
  ---|  18.10 - ''''''''' - Slimmed down menu. GPS directional framework added, added RPM ignition coil mode(1 cyl)
  ---|  18.11 - ''''''''' - Added first GPS Degree and Directional on display
  ---|  18.12 - ''''''''' - Added 1 stage acceleration profile
  ---|  18.13 - ''''''''' - removed screen flip setting
  ---|  18.14 - ''''''''' - Added menu text to clarify long and short press (cancel and save),fixed startup speed rounding off issue,fixed KP and KI gain mixup, set default servo values to 950 and 1950, Set contrast limits, Added more GPS data variables
  ---|  18.15 - ''''''''' - Added In Motion simplified display framework (commented out), repaired fix issues in 18.14
  ---|  18.16 - ''''''''' - Added Target offset, Added menu options for accel control
  ---|  18.17 - ''''''''' - Commenting out Accel Control and Target Offset, Changed PID routine to P_ON_M
  ---|  19.01 - ''''''''' - calc in (P_ON_M) mode
  ---|  19.02 - ''''''''' - calc in (P_ON_E) mode
  ---|  19.03 - ''''''''' - Change calculations from PonM to PonE on the fly when target reached. goes back to PonM when 5mph off target
  ---|  20.00 - ''''''''' - Fixed Issue with calculation delay. Set new tune defaults to PonM| P-.25 I-.50 D-.05 and PonE|P-.25 I-.25 D-.05 PonE is not tunable during operation

*/

//******LIBRARYS******
#define _Digole_Serial_SPI_
#include <DigoleSerial.h>
#include <TinyGPS++.h>
#include <Servo.h>
#include <SPI.h>
#include <OneWire.h>
#include <EEPROM.h>
#include <PID_v1.h>
#include <DallasTemperature.h>

//*************INITIALIZING DEFINITIONS*************
byte menuItemSelected = 0;
bool firstLoopOnStart = true;
bool stillSelecting = true;
double Setpoint, Input, Output, gap;
double PonMKp = 0.25, PonMKi = 0.50, PonMKd = 0.05, PonEKp = 0.30, PonEKi = 0.25, PonEKd = 0.05;
double KpInput = PonMKp;
double KiInput = PonMKi;
double KdInput = PonMKd;
PID myPID(&Input, &Output, &Setpoint, KpInput, KiInput, KdInput, P_ON_M, REVERSE);

TinyGPSPlus gps;                                //required for TinyGPSplus Library
DigoleSerialDisp mydisp(9, 8, 10);              //Pin Config SPI | 9: data | 8:clock | 10: SS | you can assign 255 to SS, and hard ground SS pin on module
float voltage = 0;				                //Voltage value
byte cylCoeff = 6;                              //Number of Cylinders
byte mode = 0;                                  //Calculations mode 0=off 1=MPH/KPH 2=RPM 
byte printLine = 0;

//*****PIN CONFIGURATION******
//0 and 1 = GPS

const byte PinSW = 4;              //encoder button
const byte servoPin = 5;           //servo PWM connected here

#if defined(__AVR_ATmega2560__)
  //Code in here will only be compiled if an Arduino Mega is used.
	const byte rpmPin = 20;        //Engine RPM input connected here
	const byte PinCLK = 3;         //encoder second pin
	const byte PinDT = 2;          // Used for reading DT signal of encoder
#else
  //Code in here will only be compiled if an Arduino Leonardo is used.
	const byte rpmPin = 7;         //Engine RPM input connected here
	const byte PinCLK = 2;         //encoder second pin
	const byte PinDT = 3;          // Used for reading DT signal of encoder
#endif

//8 and 9 - LCD, defined above
//open - 10 - LCD SS?
const byte tempWtrPin = 11;        //temperature water
const byte tempAirPin = 12;        //temperature air
const byte voltPin = A0;           //battery voltage divider
//OPEN A1, A2, A3, A4, A5



//***TEMP SENSOR INITIALIZATION***
OneWire  wtr(tempWtrPin);
OneWire  air(tempAirPin);
DallasTemperature wtrSensor(&wtr);
DallasTemperature airSensor(&air);
byte i;
byte airTemp, wtrTemp;
bool celsius = true;

//*************SPEED INITIALIZATION*************
int speedValue = 0;
int TargetSpeedInt, target100 = 500;
int SpeedInt = 0;
int speed100, pos100;
int accel100 = 500;
bool mph = true;
unsigned long delayCheck = 0;
unsigned long throttleCheck = 0;
unsigned long temperatureCheck = 0;
byte targetSpeedWhole = TargetSpeedInt / 10;
byte targetSpeedDecimal = TargetSpeedInt % 10;
byte speedBreakdown = 0;
byte decimalBreakdown = 0;
byte Contrast;
byte menuItem = 1;
//*********RIVER MODE INITIALIZATION**********
bool riverModeEnabled = false;
bool downRiverCourse = false;
byte riverModeHeadingOffset = 60;
int riverModeSpeedOffset = 0;
int riverModeHeading = 0;
int riverModeDelay = 5;
int gpsDegree;
unsigned long startedDownRiver = 0;
//***********LIMITS INITIALIZATION************
byte limitSpeed = 0;
int speedLimit100 = 2000;
const byte Off = 0;
const byte On = 1;
const byte Always = 2;
//*************RPM INITIALIZATION*************
volatile unsigned long duration = 0; // accumulates pulse width
volatile unsigned long pulsecount = 0; //incremented by interrupt
volatile unsigned long previousMicros = 0;
int targetRPM = 0;
byte CalcMode = 0;
byte limitRPM = 0;
int currentRPM = 0;
byte RPMLimit = 0;
//*************SERVO INITIALIZATION*************
Servo myservo;
int minServo = 950;
int maxServo = 1950;
int pos = minServo;
int S = 10;
float throttlePer = 100;

//*************ENCODER INITIALIZATION*************
volatile bool TurnDetected = false;
byte buttonTimes = 0;
byte inMenuPress = 0;
byte buttonTarget = 5;
bool mainDisplay = true;
volatile long encoder = 0;
int encoderChange = 0;
int hours, minutes, seconds, hourOffset = 12;
unsigned long startMillis = 0;
unsigned long currentMillis;
int menuTimeout = 10000;

//*************DISPLAY ITEMS INITIALIZATION*************
bool displayThrottle = true;
bool displayTime = true;
bool displayVoltage = true;
bool displayCourse = true;
bool displayWaterTemp = true;
bool displayAirTemp = true;
bool displayRPM = true;
bool warnDrainPlug = true;
bool twentyFourHourClockMode = false;
int voltageWarning100 = 1600;

//*************FUNCTION INITIALIZATION*************
int   readRpm();                              //read RPM and reset counters
void  rpmIntHandler();                        //called by interrupt, increments counters
static void smartDelay(unsigned long ms);     //Reads Data from GPS device
void  isr();                                  //encoder - Interrupt service routine is executed when a HIGH to LOW transition is detected on CLK
void  PIDCalculations();                      //Calculate Speed Adjustments based on MPH or RPM depending on mode
void  MainDisplay();                          //Runs the Main LCD screen
void  Menu();                                 //runs the USER menu options
int read_encoder();                           //Reads knob up/down/press/long press for menu cases
void updateTemperatureVoltRead();
void PrintEZ(const char* text, int fontSize, int posX, int posY, int offsetX, int offsetY, bool inverted, int delayAfter, int spacesAfter);
void PrintEZ(double text, int fontSize, int posX, int posY, int offsetX, int offsetY, bool inverted, int delayAfter, int spacesAfter);
void PrintEZ(int text, int fontSize, int posX, int posY, int offsetX, int offsetY, bool inverted, int delayAfter, int spacesAfter);
void PrintEZ(byte text, int fontSize, int posX, int posY, int offsetX, int offsetY, bool inverted, int delayAfter, int spacesAfter);

const unsigned char startupScreenData[] PROGMEM = {
  'C', 'L', //clear screen
  'G', 'P', 0, 0,  //set display position at 0,0
  'D', 'I', 'M', 128, 64,  //draw a 128x64 mono image, following is 128x64 standard image data, can be used on any Digole serial modules
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 192, 0, 0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 15, 240, 0, 0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 31, 248, 0, 0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 63, 252, 0, 0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 63, 252, 0, 224,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 127, 254, 1, 240,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 127, 254, 3, 248,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 127, 254, 7, 248,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 127, 254, 15, 248,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 0, 63, 252, 31, 240,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 0, 63, 252, 63, 224,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 15, 0, 31, 248, 127, 192,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 63, 0, 15, 240, 255, 128,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 127, 0, 7, 225, 255, 0,0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 254, 0, 0, 3, 254, 0,0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 207, 255, 255, 255, 252, 0,0, 0, 0, 0, 0, 0, 0, 0, 0, 15, 159, 255, 255, 255, 248, 0,255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 31, 255, 255, 255, 240, 0,255, 255, 255, 255, 255, 255, 255, 255, 255, 252, 31, 255, 255, 255, 224, 0,0, 0, 0, 0, 0, 0, 0, 0, 0, 30, 31, 255, 255, 255, 192, 0,0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 191, 255, 255, 255, 128, 0,0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 252, 0, 31, 255, 0, 0,31, 224, 0, 0, 0, 0, 7, 128, 0, 1, 252, 0, 63, 254, 0, 0,63, 240, 0, 0, 0, 0, 31, 128, 0, 0, 252, 0, 63, 252, 0, 0,127, 240, 112, 0, 0, 0, 15, 128, 0, 0, 124, 0, 127, 248, 0, 0,124, 241, 240, 0, 0, 0, 15, 128, 0, 0, 56, 0, 127, 248, 0, 0,126, 3, 254, 62, 7, 224, 255, 159, 252, 0, 56, 0, 255, 240, 0, 0,127, 199, 254, 255, 31, 241, 255, 159, 252, 0, 0, 1, 255, 240, 0, 0,63, 225, 240, 247, 157, 243, 239, 143, 220, 0, 0, 1, 255, 224, 0, 0,15, 241, 241, 255, 129, 243, 239, 135, 220, 0, 0, 3, 255, 224, 0, 0,119, 241, 241, 255, 143, 243, 239, 135, 248, 0, 0, 3, 255, 192, 0, 0,121, 241, 241, 240, 63, 243, 239, 131, 248, 0, 0, 7, 255, 192, 0, 0,127, 241, 241, 249, 188, 251, 239, 131, 240, 0, 0, 31, 255, 128, 0, 0,63, 225, 254, 255, 191, 249, 255, 193, 240, 0, 0, 63, 255, 128, 0, 0,31, 128, 252, 126, 31, 240, 255, 159, 240, 0, 0, 127, 255, 0, 0, 0,0, 0, 0, 0, 0, 0, 0, 31, 224, 0, 0, 255, 255, 0, 0, 0,0, 0, 0, 0, 0, 0, 0, 31, 192, 0, 1, 255, 255, 0, 0, 0,0, 0, 0, 0, 0, 0, 0, 15, 128, 0, 1, 255, 255, 0, 0, 0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 255, 127, 0, 0, 0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 254, 127, 0, 0, 0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 252, 127, 128, 0, 0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 252, 127, 128, 0, 0,0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 253, 252, 127, 192, 0, 0,0, 7, 254, 0, 0, 0, 0, 0, 0, 63, 255, 252, 63, 192, 0, 0,0, 7, 255, 0, 0, 0, 0, 0, 0, 127, 255, 255, 63, 224, 0, 0,0, 3, 239, 128, 0, 0, 0, 0, 0, 127, 255, 255, 255, 224, 0, 0,0, 3, 239, 128, 0, 0, 0, 0, 0, 127, 255, 255, 255, 240, 0, 0,0, 3, 239, 135, 225, 252, 127, 0, 0, 255, 255, 255, 255, 248, 0, 0,0, 3, 239, 159, 243, 254, 255, 128, 0, 255, 255, 255, 255, 248, 0, 0,0, 3, 255, 29, 243, 238, 251, 128, 0, 127, 255, 255, 255, 248, 0, 0,0, 3, 254, 1, 243, 240, 252, 0, 0, 63, 255, 255, 255, 254, 0, 0,0, 3, 224, 15, 241, 252, 127, 0, 0, 31, 255, 255, 255, 255, 0, 0,0, 3, 224, 63, 240, 254, 63, 128, 0, 7, 255, 255, 255, 255, 192, 0,0, 3, 224, 60, 251, 190, 239, 128, 0, 3, 255, 255, 255, 255, 224, 0,0, 7, 240, 63, 251, 254, 255, 128, 0, 1, 255, 255, 255, 255, 240, 0,0, 7, 240, 31, 241, 252, 127, 0, 0, 0, 127, 255, 255, 255, 252, 0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 31, 255, 255, 255, 254, 0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 15, 255, 255, 255, 255, 0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 255, 255, 255, 255, 128,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 255, 255, 255, 255, 128,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 31, 255, 255, 255, 0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 255, 255, 255, 0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 63, 255, 254, 0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 255, 248, 0,
  'S','C',1,  //set color white
  'T', 'P', 0, 0,  //set text position at 0,0
  255,255 //255 is indicate of ending this command set
};



//************************************************************************************************|
//*********** PERFORM STARTUP TASKS | DISPLAY STARTUP | GPS CONFIGURATION | READ MEMORY **********|
//************************************************************************************************|
void setup()
{
	//for debug---------
	Serial.begin(115200);
	delay(100);
	//------------------
	Serial1.begin(9600);   //GPS device
	delay(3000);
	mydisp.begin();
	
	if (EEPROM.read(200) != 22)
	{
		PrintEZ(" INITIALIZE MEM", 30, 0, 3, 0, 0, false, 2000, 2);
		initializeEEPROMValues(false);
		mydisp.clearScreen();
		delay(500);
		PrintEZ("UPDATING TUNING", 30, 0, 3, 0, 0, false, 2000, 0);
		initializePIDValues(false);
		mydisp.clearScreen();
		delay(500);
		mydisp.displayConfig(0);  //Turns off Digole display configuration data on start screen.
		mydisp.downloadStartScreen(sizeof(startupScreenData), startupScreenData);	//Change startup screen to SteadyPass logo.
	}

	mydisp.displayStartScreen(1);
	PrintEZ("Version: 22", 10, 0, 0, 0, 0, false, 2000, 0);
	mydisp.clearScreen();
	delay(500);


	//******************LOADING SAVED SETTINGS*******************
	readPIDValues();
	readEEPROMValues();
	if (warnDrainPlug) {
		for (int i = 64; i > 0; i--)
		{
			mydisp.drawLine(0, i, 128, i);
			PrintEZ("DRAIN PLUG IN?", 30, 1, 2, 0, 0, false, 45, 0);
		}
		mydisp.clearScreen();
	}
	PrintEZ("ACQUIRING SIGNAL", 30, 0, 1, 0, 0, false, 0, 0);
	PrintEZ("  PLEASE WAIT ", 30, 0, 3, 0, 0, false, 20, 0);
	pinMode(PinCLK, INPUT);										            // rotary encoder
	pinMode(PinDT, INPUT);													// rotary encoder
	pinMode(PinSW, INPUT_PULLUP);								            // rotary encoder button
	attachInterrupt(0, isr, RISING);										// interrupt 0 pin 3 for encoder
	pinMode(rpmPin, INPUT);													// RPM pin, high when no pulse.
	attachInterrupt(digitalPinToInterrupt(rpmPin), rpmIntHandler, CHANGE);  // RPM pin, is grounded on pulse
	myservo.attach(servoPin);								                // attaches the servo on pin  to the servo object
	//PID Variables//
	Input = speed100;
	Setpoint = target100;
	//Turn on PID//
	myPID.SetMode(AUTOMATIC);
	myPID.SetSampleTime(300);
	myPID.SetOutputLimits(minServo, maxServo);
	myservo.writeMicroseconds(minServo);

	//******************LOADING GPS CONFIGURATION*******************
	Serial1.println("$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29");     //Sets GPS to RMC only
	delay(100);
	Serial1.println("$PMTK220,200*2C");  //Set GPS to 5hz
	delay(100);
}

//**RUN MAIN**|
void loop()
{
	smartDelay(50);

	if (firstLoopOnStart) {
		mydisp.clearScreen();
		delay(100);
		mydisp.setContrast(Contrast);
		firstLoopOnStart = false;
	}
	if (gps.speed.isValid()) {
		speed100 = 100 * gps.speed.mph();
	}

	//UPDATE TEMPS/VOLTAGE CHECK
	if (temperatureCheck < millis()) {
		temperatureCheck = millis() + 5000;
		updateTemperatureVoltRead();
	}

	//RUN CURRENT CALCULATION MODE//
	if (mode == 0) {
		if (limitRPM == Always || limitSpeed == Always){
			if ((currentRPM > RPMLimit * 100) || ((speedValue / 10) > speedLimit100)) {
				// Speed or RPM is over limit extend throttle position to slow engine down
				pos = pos + 3;
				if (pos > maxServo) pos = maxServo;
			}
			else {
				// Limit is in effect but speed/rpm isn't exceeding limits.
				// Increase throttle so that we either exceed speed and get caught by above if statement or are at minimum throttle position.
				pos--;
				if (pos < minServo) pos = minServo;
			}
		}
		else {
			// Mode is off and no limits are set to always on.
			pos = minServo;
		}
	}
	else {
		if ((limitRPM != Off && currentRPM > RPMLimit * 100) || (limitSpeed != Off && (speedValue / 10) > speedLimit100)) {
			// Speed or RPM is over limit extend throttle position to slow engine down
			pos = pos + 3;
			if (pos > maxServo) pos = maxServo;
		}
		else {
			if (mode == 1) {  //Speed Mode
				PIDCalculations();
			}
			else { // RPM Mode
				if (currentRPM > targetRPM + 250) {
					pos += 3;//Really Too fast
				}
				else if (currentRPM > targetRPM + 50) {
					pos += 1; //Too Fast
				}
				else if (currentRPM < targetRPM - 250) {
					pos -= 3; //Really Too Slow
				}
				else if (currentRPM < targetRPM - 50) {
					pos -= 1; //Too Slow
				}

				if (pos > maxServo) {
					pos = maxServo;
				}
				else if (pos < minServo) {
					pos = minServo;
				}
			}
		}
	}
	myservo.writeMicroseconds(pos);

	//DISPLAY MAIN OUTPUT section
	TargetSpeedInt = target100 / 10;
	speedValue = speed100 / 10;
	if (!mph) speedValue *= 1.61;

	if (mainDisplay) MainDisplay();  //DISPLAY MAIN OUTPUT

	if (TurnDetected) {
		switch (mode) {
		case 1:
			target100 += -10 * encoder;
			if (target100 < 0) target100 = 0;
			break;
		case 2:
			targetRPM += -10 * encoder;
			if (targetRPM < 0) targetRPM = 0;
			break;
		}
		encoder = 0;
		TurnDetected = false;
	}

	//menu enter
	if (digitalRead(PinSW) == LOW) {
		buttonTimes++;
	}
	else {
		if (buttonTimes > 0) {
			mode++;
			if (mode > 2) mode = 0;
		}
		buttonTimes = 0;
	}

	if (buttonTimes > buttonTarget) {
		mydisp.clearScreen();
		mydisp.setFont(20);
		mydisp.setPrintPos(0, 0);
		delay(500);
	}
	while (buttonTimes > buttonTarget) {
		mydisp.print("Main Menu");
		delay(500);
		mydisp.clearScreen();
		Menu();
	}
}

//*******************************************************************
/* FUNCTION: smartDelay()
   INPUT: N/A
   RETURN: N/A
   DESCRIPTION: Reads Data from GPS device, Loops whiles data is transferring over Serial, ends loop once data is read.
*/
static void smartDelay(unsigned long ms)
{
	unsigned long start = millis();
	do
	{
		while (Serial1.available())
		{
			gps.encode(Serial1.read());
		}
	} while ((!gps.speed.isUpdated()));
}

//*******************************************************************
/* FUNCTION: readRPM()
   INPUT: duration | pulsecount
   RETURN:freq
   DESCRIPTION: Takes INPUTs and runs an equation to turn it into an RPM reading
*/
int readRpm()
{
	unsigned long _duration = duration;
	unsigned long _pulsecount = pulsecount;
	duration = 0; // clear counters
	pulsecount = 0;
	long rpmOut = 60 * 1e5 * _pulsecount / _duration / cylCoeff;
	rpmOut *= 10;
	return (rpmOut);
}

//*******************************************************************
/* FUNCTION: rpmIntHandler()
   INPUT: micros()
   RETURN: N/A
   DESCRIPTION: it's a interrupt handler for RPMs, need better knowledge for better description
*/
void rpmIntHandler()
{
	unsigned long currentMicros = micros();
	duration += currentMicros - previousMicros;
	previousMicros = currentMicros;
	pulsecount++;
}

//*******************************************************************
/* FUNCTION: isr()
   INPUT: digitalread
   RETURN: increments encoder up or down
   DESCRIPTION: encoder - Interrupt service routine is executed when a HIGH to LOW transition is detected on CLK
*/
void isr()
{
	if (digitalRead(PinCLK) == HIGH)
	{
		encoder--;
	}
	else
	{
		encoder++;
	}
	TurnDetected = true;
}

//*******************************************************************
/* FUNCTION: PIDCalculations
   INPUT: Speed100 or current RPM | target100 or targetRPM
   RETURN: pos
   DESCRIPTION: Runs an equation to calculate Change in speed based off MPH | needs better description later
*/
void PIDCalculations()
{
	gap = abs(Setpoint - Input);
	
	//PID INPUTS//

	if (mode == 1) {
		if (gap > accel100) {
			if (CalcMode != P_ON_M) {
				KpInput = PonMKp;
				KiInput = PonMKi;
				KdInput = PonMKd;
				CalcMode = P_ON_M;
			}
		}
		else {
			if (CalcMode != P_ON_E) {
				KpInput = PonEKp;
				KiInput = PonEKi;
				KdInput = PonEKd;
				CalcMode = P_ON_E;
			}
		}
		myPID.SetTunings(KpInput, KiInput, KdInput, CalcMode);
		Input = speed100;
		Setpoint = target100;
		if (riverModeEnabled && downRiverCourse) {
			Setpoint += riverModeSpeedOffset;
		}
	}
	else {
		Input = currentRPM;
		Setpoint = targetRPM;
		PonEKd = 0.00;
		myPID.SetTunings(PonEKp, PonEKi, PonEKd, P_ON_E);
	}
	myPID.Compute();
	pos = Output;
}

//*******************************************************************
/* FUNCTION:  MainDisplay()
   DESCRIPTION: This displays everything that you see
*/
void MainDisplay()
{
	//Throttle percent update
	if (delayCheck < millis()) {
		delayCheck = millis() + 500;
		throttlePer = maxServo - pos;
		throttlePer = throttlePer / (maxServo - minServo);
		throttlePer *= 100;
	}

	//print word "GPS"
	PrintEZ("GPS", 10, 18, 3, 2, -2, false, 0, 0);

	//Print word "MPH" or "KPH"
	mydisp.setPrintPos(18, 4);
	mydisp.setTextPosOffset(2, -3);
	if (mph) mydisp.print("MPH"); else mydisp.print("KPH");

	printLine = 0;
	if (displayThrottle) {
		//Print throttle percentage
		mydisp.setPrintPos(0, printLine);
		mydisp.print("P:");
		mydisp.print((int)(throttlePer));
		if (mode > 0) {
			if (CalcMode == P_ON_M) {
				mydisp.print("A  ");
			}
			else {
				mydisp.print("T  ");
			}
		}
		else {
			mydisp.print("  ");
		}
		printLine++;
	}

	if (displayTime) {
		//Time
		hours = gps.time.hour() + hourOffset - 12;
		if (hours < 0) hours += 24;
		if (hours > 24) hours -= 24;
		if (!twentyFourHourClockMode) {
			if (hours > 12) hours -= 12;
		}
		minutes = gps.time.minute();
		seconds = gps.time.second();
		mydisp.setFont(10);
		mydisp.setPrintPos(0, printLine);
		mydisp.print(hours); mydisp.print(":"); if (minutes < 10) mydisp.print("0"); mydisp.print(minutes); mydisp.print(":"); if (seconds < 10) mydisp.print("0"); mydisp.print(seconds); mydisp.print("  ");
		printLine++;
	}

	if (displayVoltage) {
		//Prints Voltage
		if ((voltage < ((float)voltageWarning100 / 100)) && voltage > 6) {
			PrintEZ("V ", 10, 0, printLine, 0, 0, true, 0, 0);
			PrintEZ(voltage, 10, -1, -1, 0, 0, true, 0, 0);
		}
		else
		{
			PrintEZ("V ", 10, 0, printLine, 0, 0, false, 0, 0);
			PrintEZ(voltage, 10, -1, -1, 0, 0, false, 0, 0);
		}
		printLine++;
	}

	gpsDegree = gps.course.deg();
	if (riverModeEnabled) {
		if (headingError(gpsDegree, riverModeHeading) <= riverModeHeadingOffset) {
			if (startedDownRiver == 0) startedDownRiver = millis();
			if (millis() - startedDownRiver >= riverModeDelay * 1000) {
				downRiverCourse = true;
			}
		}
		else {
			downRiverCourse = false;
			startedDownRiver = 0;
		}
	}

	if (displayCourse) {
		//print GPS directional
		PrintEZ("DIR    ", 10, 0, printLine, 0, 0, false, 0, 0);
		mydisp.setPrintPos(4, printLine);
		PrintEZ(TinyGPSPlus::cardinal(gpsDegree), 10, -1, -1, 0, 0, downRiverCourse && mode == 1, 20, 0);
		printLine++;
	}

	if (displayWaterTemp && wtrTemp > 0) {
		// Prints Water Temp
		mydisp.setPrintPos(0, printLine);
		mydisp.print("WTR "); mydisp.print(wtrTemp); if (celsius)  mydisp.print("c  "); else mydisp.print("F  ");
		printLine++;
	}

	if (displayAirTemp && airTemp > 0 ) {
		// Prints Air Temp
		mydisp.setPrintPos(0, printLine);
		mydisp.print("AIR "); mydisp.print(airTemp); if (celsius)  mydisp.print("c  "); else mydisp.print("F  ");
		printLine++;
	}

	currentRPM = readRpm();
	if (displayRPM && currentRPM > 0) {
		//Prints current RPM reading
		mydisp.setPrintPos(0, printLine);
		mydisp.print("RPM "); mydisp.print(currentRPM); mydisp.print("    ");
	}

	//Print target MPH or KPH depending on mode selection
	if (downRiverCourse) {
		targetSpeedWhole = SpeedBreakdown(target100 + riverModeSpeedOffset, true);
		targetSpeedDecimal = SpeedBreakdown(target100 + riverModeSpeedOffset, false);
	}
	else {
		targetSpeedWhole = SpeedBreakdown(target100, true);
		targetSpeedDecimal = SpeedBreakdown(target100, false);
	}
	mydisp.setFont(18);
	mydisp.setPrintPos(6, 0);

	switch (mode) {
	case 0:
		mydisp.print("   OFF  ");
		break;
	case 1:
		if (targetSpeedWhole < 10) {
			mydisp.print("  ");
		}
		else {
			mydisp.print(" ");
		}
		mydisp.print(abs(targetSpeedWhole)); mydisp.print(".");
		mydisp.print(abs(targetSpeedDecimal));
		if (mph) mydisp.print("MPH"); else mydisp.print("KPH");
		break;
	case 2:
		mydisp.print(" ");
		mydisp.print(targetRPM);
		mydisp.print("RPM");
		break;
	}

	//Prints the Main Speed value On Display
	PrintEZ("", 203, 0, 0, 46, 22, false, 0, 0);
	if (speedValue < 100) mydisp.print('0');
	mydisp.print(speedValue / 10);
	
	//Prints the decimal value of the Main Speed
	PrintEZ(speedValue % 10, 201, 0, 0, 111, 43, false, 0, 0);
}

int read_encoder() {
#define btnUp         1
#define btnDown       2
#define btnPress      4
#define btnLongPress  8
#define btnNull       16

	if (TurnDetected) {
		TurnDetected = false;
		if (encoder > 0) {
			encoderChange = encoder;
			encoder = 0;
			return btnUp;
		}
		else if (encoder < 0) {
			encoderChange = encoder;
			encoder = 0;
			return btnDown;
		}
		else {
			encoderChange = 0;
			return btnNull;
		}
	}
	else {
		encoderChange = 0;
		if (digitalRead(PinSW) == LOW)
		{
			inMenuPress++;
			delay(200);
		}
		else if (inMenuPress > 0) {
			inMenuPress = 0;
			mydisp.clearScreen();
			return btnPress;
		}
		else {
			return btnNull;
		}

		if (inMenuPress > buttonTarget) {
			inMenuPress = 0;
			mydisp.clearScreen();
			PrintEZ("BACK", 20, 0, 0, 0, 0, false, 0, 0);
			while (digitalRead(PinSW) == LOW) {};
			mydisp.clearScreen();
			return btnLongPress;
		}
	}
}

//*******************************************************************
/* FUNCTION: Menu()
   DESCRIPTION: Displays the main settings menu
*/
void Menu() {
	//Reset selectors to false
	menuItem = 1;
	startMillis = millis();
	do {
		stillSelecting = true;

		PrintEZ(" Tuning ", 20, 0, 0, 0, 0, menuItem == 1, 20, 0);
		PrintEZ(" Preferences ", 20, 0, 1, 0, 0, menuItem == 2, 20, 0);
		PrintEZ(" Setup ", 20, 0, 2, 0, 0, menuItem == 3, 20, 0);
		PrintEZ(" Configuration ", 20, 0, 3, 0, 0, menuItem == 4, 20, 0);
		PrintEZ(" Misc ", 20, 0, 4, 0, 0, menuItem == 5, 20, 0);
		switch (read_encoder()) {
		case 1:  // ENCODER UP
			startMillis = millis();
			menuItem--;
			if (menuItem < 1) menuItem = 5;
			break;
		case 2:  //ENCODER DOWN
			startMillis = millis();
			menuItem++;
			if (menuItem > 5) menuItem = 1;
			break;
		case 4:  // ENCODER BUTTON SHORT PRESS
			if (menuItem == 1) tuningMenu();
			if (menuItem == 2) preferencesMenu();
			if (menuItem == 3) setupMenu();
			if (menuItem == 4) configurationMenu();
			if (menuItem == 5) miscMenu();
			break;
		case 8:  // ENCODER BUTTON LONG PRES
			stillSelecting = false;
			returnToMainDisp();
			break;
		case 16:  // ENCODER BUTTON NULL
			currentMillis = millis();
			if ((currentMillis - startMillis) >= menuTimeout * 1000) {
				mydisp.clearScreen();
				returnToMainDisp();
			}
			break;
		}
	} while (stillSelecting);
}


void tuningMenu() {
	menuInitialize();
	do {

		PrintEZ("Target P: ", 10, 0, 0, 0, 0, menuItem == 1, 0, 0);
		PrintEZ(PonEKp, 10, 11, 0, 0, 0, menuItemSelected == 1, 25, 0);

		PrintEZ("Target I: ", 10, 0, 1, 0, 0, menuItem == 2, 0, 0);
		PrintEZ(PonEKi, 10, 11, 1, 0, 0, menuItemSelected == 2, 25, 0);

		PrintEZ("Target D: ", 10, 0, 2, 0, 0, menuItem == 3, 0, 0);
		PrintEZ(PonEKd, 10, 11, 2, 0, 0, menuItemSelected == 3, 25, 0);

		PrintEZ("Transition: ", 10, 0, 3, 0, 0, menuItem == 4, 0, 0);
		PrintEZ(SpeedBreakdown(accel100, true), 10, 13, 3, 0, 0, menuItemSelected == 4, 0, 0);
		PrintEZ(".", 10, -1, -1, 0, 0, menuItemSelected == 4, 0, 0);
		PrintEZ(SpeedBreakdown(accel100, false), 10, -1, -1, 0, 0, menuItemSelected == 4, 0, 0);
		PrintSpeedUnit(menuItemSelected == 4);

		PrintEZ("Accel P: ", 10, 0, 4, 0, 0, menuItem == 5, 0, 0);
		PrintEZ(PonMKp, 10, 10, 4, 0, 0, menuItemSelected == 5, 25, 0);

		PrintEZ("Accel I: ", 10, 0, 5, 0, 0, menuItem == 6, 0, 0);
		PrintEZ(PonMKi, 10, 10, 5, 0, 0, menuItemSelected == 6, 25, 0);

		PrintEZ("Accel D: ", 10, 0, 6, 0, 0, menuItem == 7, 0, 0);
		PrintEZ(PonMKd, 10, 10, 6, 0, 0, menuItemSelected == 7, 25, 0);

		switch (read_encoder()) {
		case 4:  // ENCODER BUTTON SHORT PRESS
			startMillis = millis();
			if (menuItemSelected == 0) {
				menuItemSelected = menuItem;
			}
			else {
				menuItemSelected = 0;
			}
			break;
		case 8:  // ENCODER BUTTON LONG PRESS
			Menu();
			break;
		case 16:  // ENCODER BUTTON NULL
			currentMillis = millis();
			if ((currentMillis - startMillis) >= menuTimeout * 1000) {
				mydisp.clearScreen();
				stillSelecting = false;
				returnToMainDisp();
			}
			break;
		default:  // ENCODER TURNED
			startMillis = millis();

			switch (menuItemSelected) {
			case 1: // Target P
				PonEKp = adjustPIDValue(PonEKp);
				break;
			case 2: // Target I
				PonEKi = adjustPIDValue(PonEKi);
				break;
			case 3: // Target D
				PonEKd = adjustPIDValue(PonEKd);
				break;
			case 4: // Accel to Target
				accel100 -= encoderChange * 10;
				if (accel100 < 100) accel100 = 100;
				if (accel100 >= 990) accel100 = 990;
				break;
			case 5: // Accel P
				PonMKp = adjustPIDValue(PonMKp);
				break;
			case 6: // Accel I
				PonMKi = adjustPIDValue(PonMKi);
				break;
			case 7: // Accel D
				PonMKd = adjustPIDValue(PonMKd);
				break;
			default: //No menu item selected
				changeMenuItem(7);
				break;
			}
			break;
		}
	} while (stillSelecting);
}

void preferencesMenu() {
	menuInitialize();
	do {
		PrintEZ(" Start Spd: ", 10, 0, 0, 0, 0, menuItem == 1, 0, 0);
		PrintEZ(SpeedBreakdown(target100, true), 10, 13, 0, 0, 0, menuItemSelected == 1, 0, 0);
		PrintEZ(".", 10, -1, -1, 0, 0, menuItemSelected == 1, 0, 0);
		PrintEZ(SpeedBreakdown(target100, false) , 10, -1, -1, 0, 0, menuItemSelected == 1, 0, 0);
		PrintSpeedUnit(menuItemSelected == 1);
		mydisp.print(" ");

		PrintEZ(" Start RPM: ", 10, 0, 1, 0, 0, menuItem == 2, 0, 0);
		PrintEZ(targetRPM, 10, 13, 1, 0, 0, menuItemSelected == 2, 20, 0);

		PrintEZ(" Volts Warning: ", 10, 0, 2, 0, 0, menuItem == 3, 0, 0);
		PrintEZ(DecimalBreakdown(voltageWarning100, true), 10, 17, 2, 0, 0, menuItemSelected == 3, 0, 0);
		PrintEZ(".", 10, -1, -1, 0, 0, menuItemSelected == 3, 0, 0);
		PrintEZ(DecimalBreakdown(voltageWarning100, false), 10, -1, -1, 0, 0, menuItemSelected == 3, 0, 0);

		PrintEZ(" Plug Warning: ", 10, 0, 3, 0, 0, menuItem == 4, 0, 0);
		PrintShowHide(warnDrainPlug, menuItemSelected == 4);

		PrintEZ(" Menu Timeout: ", 10, 0, 4, 0, 0, menuItem == 5, 0, 0);
		PrintEZ(menuTimeout, 10, 16, 4, 0, 0, menuItemSelected == 5, 20, 1);
				
		switch (read_encoder()) {
		case 4:  // ENCODER BUTTON SHORT PRESS
			if (menuItemSelected == 0) {
				menuItemSelected = menuItem;
			}
			else {
				menuItemSelected = 0;
			}
			break;
		case 8:  // ENCODER BUTTON LONG PRESS
			Menu();
			break;
		case 16:  // ENCODER BUTTON NULL
			currentMillis = millis();
			if ((currentMillis - startMillis) >= menuTimeout * 1000) {
				mydisp.clearScreen();
				returnToMainDisp();
			}
			break;
		default:  // ENCODER TURNED
			startMillis = millis();

			switch (menuItemSelected) {
			case 1: // Start Speed
				target100 -= encoderChange * 10;
				if (target100 < 500) target100 = 500;
				if (target100 >= 5000) target100 = 5000;
				break;
			case 2: // Start RPM
				targetRPM -= encoderChange * 10;
				if (targetRPM < 1000) targetRPM = 1000;
				if (targetRPM >= 9900) targetRPM = 9900;
				break;
			case 3: // Voltage Warning
				voltageWarning100 -= encoderChange * 10;
				if (voltageWarning100 < 1000) voltageWarning100 = 1000;
				if (voltageWarning100 >= 1600) voltageWarning100 = 1600;
				break;
			case 4: // Drain Plug Warning
				if (encoderChange || 0) warnDrainPlug = !warnDrainPlug;
				break;
			case 5: // Menu Timeout
				menuTimeout -= encoderChange;
				if (menuTimeout < 5)  menuTimeout = 5;
				if (menuTimeout > 99) menuTimeout = 99;
				break;
			default:
				changeMenuItem(5);
				break;
			}
			break;
		}
	} while (stillSelecting);
}

void setupMenu()
{
	menuInitialize();
	do {
		PrintEZ("Engine Cyl: ", 10, 0, 0, 0, 0, menuItem == 1, 0, 0);
		PrintEZ(cylCoeff, 10, 13, 0, 0, 0, menuItemSelected == 1, 20, 0);

		PrintEZ("Clock Offset: ", 10, 0, 1, 0, 0, menuItem == 2, 0, 0);
		PrintEZ(hourOffset - 12, 10, 15, 1, 0, 0, menuItemSelected == 2, 20, 1);
		
		PrintEZ("Hour Mode: ", 10, 0, 2, 0, 0, menuItem == 3, 0, 0);
		if (twentyFourHourClockMode) {
			PrintEZ("24HR", 10, 12, 2, 0, 0, menuItemSelected == 3, 20, 0);
		}
		else {
			PrintEZ("12HR", 10, 12, 2, 0, 0, menuItemSelected == 3, 20, 0);
		}

		PrintEZ("Speed Units: ", 10, 0, 3, 0, 0, menuItem == 4, 0, 0);
		PrintSpeedUnit(menuItemSelected == 4);

		PrintEZ("Temp Units: ", 10, 0, 4, 0, 0, menuItem == 5, 0, 0);
		if (celsius) {
			PrintEZ("C", 10, 13, 4, 0, 0, menuItemSelected == 5, 20, 0);
		}
		else {
			PrintEZ("F", 10, 13, 4, 0, 0, menuItemSelected == 5, 20, 0);
		}

		PrintEZ("Contrast: ", 10, 0, 5, 0, 0, menuItem == 6, 0, 0);
		PrintEZ(Contrast, 10, 11, 5, 0, 0, menuItemSelected == 6, 20, 0);
		
		switch (read_encoder()) {
		case 4:  // ENCODER BUTTON SHORT PRESS
			if (menuItemSelected == 0) {
				menuItemSelected = menuItem;
			}
			else {
				menuItemSelected = 0;
			}
			break;
		case 8:  // ENCODER BUTTON LONG PRESS
			Menu();
			break;
		case 16:  // ENCODER BUTTON NULL
			currentMillis = millis();
			if ((currentMillis - startMillis) >= menuTimeout * 1000) {
				mydisp.clearScreen();
				returnToMainDisp();
			}
			break;
		default:  // ENCODER TURNED
			startMillis = millis();

			switch (menuItemSelected) {
			case 1: // Engine Cylinders
				if (encoderChange > 0) {
					if (cylCoeff > 4) {
						cylCoeff = cylCoeff - 2;
					}
					else {
						cylCoeff = 1;
					}
				}
				else if (encoderChange < 0) {
					if (cylCoeff == 1) {
						cylCoeff = 4;
					}
					else if (cylCoeff < 8) {
						cylCoeff = cylCoeff + 2;
					}
					else {
						cylCoeff = 8;
					}
				}
				break;
			case 2: // Clock Offset
				hourOffset -= encoderChange;
				if (hourOffset < 0) hourOffset = 24;
				if (hourOffset > 24) hourOffset = 0;
				break;
			case 3: // Hour Mode
				if (encoderChange || 0) twentyFourHourClockMode = !twentyFourHourClockMode;
				break;
			case 4: // Speed Units
				if (encoderChange || 0) mph = !mph;
				break;
			case 5: // Temp Units
				if (encoderChange || 0) celsius = !celsius;
				break;
			case 6: // Contrast
				Contrast -= encoderChange;
				if (Contrast >= 40) Contrast = 40;
				if (Contrast <= 20) Contrast = 20;
				mydisp.setContrast(Contrast);
				break;
			default:
				changeMenuItem(6);
				break;
			}
			break;
		}
	} while (stillSelecting);

}

void configurationMenu()
{
	menuInitialize();
	do {
		PrintEZ(" Servo ", 20, 0, 0, 0, 0, menuItem == 1, 20, 0);
		PrintEZ(" Display Items ", 20, 0, 1, 0, 0, menuItem == 2, 20, 0);
		PrintEZ(" Limits ", 20, 0, 2, 0, 0, menuItem == 3, 20, 0);
		PrintEZ(" River Mode ", 20, 0, 3, 0, 0, menuItem == 4, 20, 0);
		switch (read_encoder()) {
		case 1:  // ENCODER UP
			startMillis = millis();
			menuItem--;
			if (menuItem < 1) menuItem = 4;
			break;
		case 2:    //ENCODER DOWN
			startMillis = millis();
			menuItem++;
			if (menuItem > 4) menuItem = 1;
			break;
		case 4:  // ENCODER BUTTON SHORT PRESS
			if (menuItem == 1) servoMenu();
			if (menuItem == 2) displayMenu();
			if (menuItem == 3) limitsMenu();
			if (menuItem == 4) riverMenu();
			break;
		case 8:  // ENCODER BUTTON LONG PRES
			stillSelecting = false;
			Menu();
			break;
		case 16:  // ENCODER BUTTON NULL
			currentMillis = millis();
			if ((currentMillis - startMillis) >= menuTimeout * 1000) {
				mydisp.clearScreen();
				Menu();
			}
			break;
		}
	} while (stillSelecting);
}

void displayMenu() {
	menuInitialize();
	do {

		PrintEZ(" Throttle: ", 10, 0, 0, 0, 0, menuItem == 1, 0, 0);
		PrintShowHide(displayThrottle, menuItemSelected == 1);
		
		PrintEZ(" Time: ", 10, 0, 1, 0, 0, menuItem == 2, 0, 0);
		PrintShowHide(displayTime, menuItemSelected == 2);
		
		PrintEZ(" Voltage: ", 10, 0, 2, 0, 0, menuItem == 3, 0, 0);
		PrintShowHide(displayVoltage, menuItemSelected == 3);

		PrintEZ(" Course: ", 10, 0, 3, 0, 0, menuItem == 4, 0, 0);
		PrintShowHide(displayCourse, menuItemSelected == 4);

		PrintEZ(" Water Temp: ", 10, 0, 4, 0, 0, menuItem == 5, 0, 0);
		PrintShowHide(displayWaterTemp, menuItemSelected == 5);

		PrintEZ(" Air Temp: ", 10, 0, 5, 0, 0, menuItem == 6, 0, 0);
		PrintShowHide(displayAirTemp, menuItemSelected == 6);

		PrintEZ(" RPM: ", 10, 0, 6, 0, 0, menuItem == 7, 0, 0);
		PrintShowHide(displayRPM, menuItemSelected == 7);

		switch (read_encoder()) {
		case 4:  // ENCODER BUTTON SHORT PRESS
			if (menuItemSelected == 0) {
				menuItemSelected = menuItem;
			}
			else {
				menuItemSelected = 0;
			}
			break;
		case 8:  // ENCODER BUTTON LONG PRESS
			configurationMenu();
			break;
		case 16:  // ENCODER BUTTON NULL
			currentMillis = millis();
			if ((currentMillis - startMillis) >= menuTimeout * 1000) {
				mydisp.clearScreen();
				configurationMenu();
			}
			break;
		default:  // ENCODER TURNED
			startMillis = millis();

			if (encoderChange || 0) {
				switch (menuItemSelected) {
				case 1: // Throttle
					displayThrottle = !displayThrottle;
					break;
				case 2: // Time
					displayTime = !displayTime;
					break;
				case 3: // Voltage
					displayVoltage = !displayVoltage;
					break;
				case 4: // Course
					displayCourse = !displayCourse;
					break;
				case 5: // Water Temp
					 displayWaterTemp = !displayWaterTemp;
					break;
				case 6: // Air Temp
					displayAirTemp = !displayAirTemp;
					break;
				case 7: // RPM
					displayRPM = !displayRPM;
					break;
				default:
					changeMenuItem(7);
				}
				break;
			}
		}
	} while (stillSelecting);
}

void limitsMenu() {
	menuInitialize();
	do {
		PrintEZ(" RPM: ", 10, 0, 0, 0, 0, menuItem == 1, 20, 0);
		PrintSetting(limitRPM, menuItemSelected == 1);
		mydisp.print(" ");

		PrintEZ(" Speed: ", 10, 0, 1, 0, 0, menuItem == 2, 20, 0);
		PrintSetting(limitSpeed, menuItemSelected == 2);
		mydisp.print(" ");

		PrintEZ(" Max RPM: ", 10, 0, 2, 0, 0, menuItem == 3, 20, 0);
		PrintEZ(RPMLimit * 100, 10, 11, 2, 0, 0, menuItemSelected == 3, 20, 5);

		PrintEZ(" Max Speed: ", 10, 0, 3, 0, 0, menuItem == 4, 20, 0);
		PrintEZ(SpeedBreakdown(speedLimit100, true), 10, 13, 3, 0, 0, menuItemSelected == 4, 0, 0);
		PrintEZ(".", 10, -1, -1, 0, 0, menuItemSelected == 4, 0, 0);
		PrintEZ(SpeedBreakdown(speedLimit100, false), 10, -1, -1, 0, 0, menuItemSelected == 4, 0, 0);
		PrintSpeedUnit(menuItemSelected == 4);
		mydisp.print(" ");

		switch (read_encoder()) {
		case 4:  // ENCODER BUTTON SHORT PRESS
			if (menuItemSelected == 0) {
				menuItemSelected = menuItem;
			}
			else {
				menuItemSelected = 0;
			}
			break;
		case 8:  // ENCODER BUTTON LONG PRESS
			Menu();
			break;
		case 16:  // ENCODER BUTTON NULL
			currentMillis = millis();
			if ((currentMillis - startMillis) >= menuTimeout * 1000) {
				mydisp.clearScreen();
				returnToMainDisp();
			}
			break;
		default:  // ENCODER TURNED
			startMillis = millis();
			switch (menuItemSelected) {
			case 1: // Limit RPM
				limitRPM -= encoderChange;
				if (limitRPM >= 200) limitRPM = 0; // limitRPM would be >=200 instead of negative as bytes can't be negative so they wrap backwards to 255 or less.
				if (limitRPM >= 3) limitRPM = 2;
				break;
			case 2: // Limit Speed
				limitSpeed -= encoderChange;
				if (limitSpeed >= 200) limitSpeed = 0; // limitSpeed would be >=200 instead of negative as bytes can't be negative so they wrap backwards to 255 or less.
				if (limitSpeed >= 3) limitSpeed = 2;
				break;
			case 3: // RPM Limit
				RPMLimit -= encoderChange;
				if (RPMLimit > 99) RPMLimit = 99;
				if (RPMLimit < 1) RPMLimit = 1;
				break;
			case 4: // Speed Limit
				speedLimit100 -= encoderChange * 10;
				speedLimit100 = boundsCheck(speedLimit100, 1000, 9900);
				break;
			default:
				changeMenuItem(4);
				break;
			}
			break;
		}
	} while (stillSelecting);
}

void riverMenu() {
	menuInitialize();
	do {
		PrintEZ(" Enabled: ", 10, 0, 0, 0, 0, menuItem == 1, 0, 0);
		PrintSetting(riverModeEnabled, menuItemSelected == 1);
		
		PrintEZ(" Heading: ", 10, 0, 1, 0, 0, menuItem == 2, 0, 0);
		PrintEZ(riverModeHeading, 10, 10, 1, 0, 0, menuItemSelected == 2, 20, 0);
		PrintEZ(" ", 10, -1, -1, 0, 0, menuItemSelected == 2, 0, 0);
		PrintEZ(TinyGPSPlus::cardinal(riverModeHeading) , 10, -1, -1, 0, 0, menuItemSelected == 2, 20, 3);
		
		PrintEZ(" Heading Offset: ", 10, 0, 2, 0, 0, menuItem == 3, 0, 0);
		PrintEZ(riverModeHeadingOffset, 10, -1, -1, 0, 0, menuItemSelected == 3, 20, 0);

		PrintEZ(" Spd Offset: ", 10, 0, 3, 0, 0, menuItem == 4, 0, 0);
		PrintEZ(SpeedBreakdown(riverModeSpeedOffset, true), 10, -1, -1, 0, 0, menuItemSelected == 4, 0, 0);
		PrintEZ(".", 10, -1, -1, 0, 0, menuItemSelected == 4, 0, 0);
		PrintEZ(SpeedBreakdown(riverModeSpeedOffset, false), 10, -1, -1, 0, 0, menuItemSelected == 4, 0, 0);
		PrintSpeedUnit(menuItemSelected == 4);
		mydisp.print(" ");

		PrintEZ(" Delay: ", 10, 0, 4, 0, 0, menuItem == 5, 0, 0);
		PrintEZ(riverModeDelay, 10, -1, -1, 0, 0, menuItemSelected == 5, 20, 0);
		if (riverModeDelay > 1) {
			PrintEZ(" seconds", 10, -1, -1, 0, 0, menuItemSelected == 5, 0, 1);
		}
		else {
			PrintEZ(" second", 10, -1, -1, 0, 0, menuItemSelected == 5, 0, 1);
		}
		
		switch (read_encoder()) {
		case 4:  // ENCODER BUTTON SHORT PRESS
			if (menuItemSelected == 0) {
				menuItemSelected = menuItem;
			}
			else {
				menuItemSelected = 0;
			}
			break;
		case 8:  // ENCODER BUTTON LONG PRESS
			configurationMenu();
			break;
		case 16:  // ENCODER BUTTON NULL
			currentMillis = millis();
			if ((currentMillis - startMillis) >= menuTimeout * 1000) {
				mydisp.clearScreen();
				configurationMenu();
			}
			break;
		default:  // ENCODER TURNED
			startMillis = millis();

			switch (menuItemSelected) {
			case 1: // Enabled
				if (encoderChange || 0) riverModeEnabled = !riverModeEnabled;
				break;
			case 2: // Heading
				riverModeHeading -= encoderChange * 1;
				riverModeHeading = boundsCheck(riverModeHeading, 0, 359);
				break;
			case 3: // Heading Offset
				riverModeHeadingOffset -= encoderChange * 1;
				if (riverModeHeadingOffset < 0) riverModeHeadingOffset = 0;
				if (riverModeHeadingOffset >= 90) riverModeHeadingOffset = 90;
				break;
			case 4: // Speed Offset
				riverModeSpeedOffset -= encoderChange * 10;
				riverModeSpeedOffset = boundsCheck(riverModeSpeedOffset, 0, 990);
				break;
			case 5: // Delay
				riverModeDelay -= encoderChange * 1;
				riverModeDelay = boundsCheck(riverModeDelay, 0, 99);
				break;
			default:
				changeMenuItem(5);
				break;
			}
			break;
		}
	} while (stillSelecting);
}

void miscMenu()
{
	menuInitialize(); 
	do {
		PrintEZ(" GPS Info ", 20, 0, 0, 0, 0, menuItem == 1, 20, 0);
		PrintEZ(" Reset ", 20, 0, 1, 0, 0, menuItem == 2, 20, 0);
		PrintEZ(" About ", 20, 0, 2, 0, 0, menuItem == 3, 20, 0);
		switch (read_encoder()) {
		case 1:  // ENCODER UP
			startMillis = millis();
			menuItem--;
			if (menuItem < 1) menuItem = 3;
			break;
		case 2:    //ENCODER DOWN
			startMillis = millis();
			menuItem++;
			if (menuItem > 3) menuItem = 1;
			break;
		case 4:  // ENCODER BUTTON SHORT PRESS
			if (menuItem == 1) GPSMenu();
			if (menuItem == 2) resetMenu();
			if (menuItem == 3) aboutMenu();
			break;
		case 8:  // ENCODER BUTTON LONG PRES
			stillSelecting = false;
			Menu();
			break;
		case 16:  // ENCODER BUTTON NULL
			currentMillis = millis();
			if ((currentMillis - startMillis) >= menuTimeout * 1000) {
				mydisp.clearScreen();
				Menu();
			}
			break;
		}
	} while (stillSelecting);
}

void GPSMenu()
{
	menuInitialize();
	do {
		
		PrintEZ(" Lat:", 10, 0, 0, 0, 0, false, 20, 17);
		mydisp.setPrintPos(6, 0);
		printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);

		PrintEZ(" Long:", 10, 0, 1, 0, 0, false, 20, 16);
		mydisp.setPrintPos(7, 1);
		printFloat(gps.location.lng(), gps.location.isValid(), 11, 6);
		mydisp.print(" ");

		PrintEZ(" Heading:", 10, 0, 2, 0, 0, false, 20, 12);
		mydisp.setPrintPos(10, 2);
		printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);
		PrintEZ(TinyGPSPlus::cardinal(gps.course.deg()) , 10, -1, -1, 0, 0, false, 20, 0);

		PrintEZ(" MPH:", 10, 0, 3, 0, 0, false, 0, 6);
		PrintEZ(gps.speed.mph(), 10, 6, 3, 0, 0, false, 20, 0);
		
		PrintEZ(" KPH:", 10, 0, 4, 0, 0, false, 0, 6);
		PrintEZ(gps.speed.kmph(), 10, 6, 4, 0, 0, false, 20, 0);

		PrintEZ(" Knots:", 10, 0, 5, 0, 0, false, 0, 6);
		PrintEZ(gps.speed.knots(), 10, 8, 5, 0, 0, false, 20, 0);

		PrintEZ("     HOLD TO EXIT    ", 10, 0, 6, 0, 0, true, 0, 0);
		
		switch (read_encoder()) {
		case 4:  // ENCODER BUTTON SHORT PRESS
			miscMenu();
			break;
		case 8:  // ENCODER BUTTON LONG PRESS
			miscMenu();
			break;
		case 16:  // ENCODER BUTTON NULL
			unsigned long start = millis();
			do
			{
				while (Serial1.available())
					gps.encode(Serial1.read());
			} while (millis() - start < 500);
			break;
		}

	} while (stillSelecting);
}

void resetMenu()
{
	menuInitialize();
	do {
		PrintEZ(" Reset PID Values ", 10, 0, 0, 0, 0, menuItem == 1, 20, 0);
		PrintEZ(" Factory Reset ", 10, 0, 1, 0, 0, menuItem == 2, 20, 0);
		switch (read_encoder()) {
		case 1:  // ENCODER UP
			startMillis = millis();
			menuItem--;
			if (menuItem < 1) menuItem = 2;
			break;
		case 2:    //ENCODER DOWN
			startMillis = millis();
			menuItem++;
			if (menuItem > 2) menuItem = 1;
			break;
		case 4:  // ENCODER BUTTON SHORT PRESS
			if (menuItem == 1) {
				initializePIDValues(true);
			}
			else {
				initializeEEPROMValues(true);
			}
			stillSelecting = false;
			miscMenu();
			break;
		case 8:  // ENCODER BUTTON LONG PRES
			stillSelecting = false;
			miscMenu();
			break;
		case 16:  // ENCODER BUTTON NULL
			currentMillis = millis();
			if ((currentMillis - startMillis) >= menuTimeout * 1000) {
				mydisp.clearScreen();
				stillSelecting = false;
			}
			break;
		}
	} while (stillSelecting);
}

void aboutMenu()
{
	mydisp.displayStartScreen(1);
	smartDelay(3000);
	mydisp.clearScreen();
	PrintEZ("  Steady Pass  ", 20, 0, 0, 0, 0, true, 1000, 0);
	PrintEZ("Designed by: Dimitry", 10, 0, 3, 0, 0, false, 1000, 0);
	PrintEZ("Code by:", 10, 0, 5, 0, 0, false, 1000, 0);
	PrintEZ("Dimitry, Sam & Nick", 10, 0, 6, 0, 0, false, 1000, 0);
	delay(4000);
	mydisp.clearScreen();
}

void initializePIDValues(bool confirm)
{
	if (confirm) {
		if (!confirmPrompt(true)) return;
	}
	
	EEPROM.update(1, 50); //PonE P 
	EEPROM.update(2, 25); //PonE I 
	EEPROM.update(3, 5);  //PonE D 
	EEPROM.update(4, 30); //PonM P 
	EEPROM.update(5, 25); //PonM I 
	EEPROM.update(6, 5);  //PonM D 
	updateWord(7, 500);   //Accel to Target Transition Speed

	if (confirm) readPIDValues();
}

void readPIDValues()
{
	PonEKp = EEPROM.read(1) / 100.00;
	PonEKi = EEPROM.read(2) / 100.00;
	PonEKd = EEPROM.read(3) / 100.00;
	PonMKp = EEPROM.read(4) / 100.00;
	PonMKi = EEPROM.read(5) / 100.00;
	PonMKd = EEPROM.read(6) / 100.00;
	accel100 = readWord(7);
}

void initializeEEPROMValues(bool confirm)
{
	if (confirm) {
		if (!confirmPrompt(false)) return;
	}
	
	EEPROM.update(9, 6);	//cylCoeff
	EEPROM.update(10, 1);   //Speed Units
	EEPROM.update(11, 5);  //Time Offset
	EEPROM.update(12, 195); //MIN Throttle
	EEPROM.update(13, 95);  //MAX Throttle
	EEPROM.update(14, 0);   //Temp Units
	updateWord(15, 2560);	//Startup Target
	EEPROM.update(17, 30);  //Contrast
	updateWord(18, 3000);   //Startup RPM
	EEPROM.update(20, 1);	//Warn Drain Plug
	EEPROM.update(21, 10);	//Menu Timeout
	updateWord(22, 1240);	//Volts Warning
	EEPROM.update(24, 0);	//Clock Mode
	EEPROM.update(25, 1);	//Display Throttle
	EEPROM.update(26, 1);	//Display Time
	EEPROM.update(27, 1);	//Display Voltage
	EEPROM.update(28, 1);	//Display Course
	EEPROM.update(29, 1);	//Display Water Temp
	EEPROM.update(30, 1);	//Display Air Temp
	EEPROM.update(31, 1);	//Display RPM
	EEPROM.update(32, 0);	//Limit RPM
	EEPROM.update(33, 0);	//Limit Speed
	EEPROM.update(34, 55);	//Max RPM
	updateWord(35, 2100);	//Max Speed
	EEPROM.update(37, 0);	//River Mode Enabled
	updateWord(38, 180);	//River Mode Heading
	updateWord(40, 60);		//Heading Offset
	updateWord(42, 400);	//Speed Offset
	EEPROM.update(44, 10);	//Delay
	EEPROM.update(200, 22);	//Preferences Reset Bit
	
	if (confirm) readEEPROMValues();
}

void readEEPROMValues()
{
	cylCoeff = EEPROM.read(9);
	mph = EEPROM.read(10);
	hourOffset = EEPROM.read(11);
	maxServo = 10 * EEPROM.read(12);
	minServo = 10 * EEPROM.read(13);
	celsius = EEPROM.read(14);
	target100 = readWord(15);
	Contrast = EEPROM.read(17);
	targetRPM = readWord(18);
	warnDrainPlug = EEPROM.read(20);
	menuTimeout = EEPROM.read(21);
	voltageWarning100 = readWord(22);
	twentyFourHourClockMode = EEPROM.read(24);
	displayThrottle = EEPROM.read(25);
	displayTime = EEPROM.read(26);
	displayVoltage = EEPROM.read(27);
	displayCourse = EEPROM.read(28);
	displayWaterTemp = EEPROM.read(29);
	displayAirTemp = EEPROM.read(30);
	displayRPM = EEPROM.read(31);
	limitRPM = EEPROM.read(32);
	limitSpeed = EEPROM.read(33);
	RPMLimit = EEPROM.read(34);
	speedLimit100 = readWord(35);
	riverModeEnabled = EEPROM.read(37);
	riverModeHeading = readWord(38);
	riverModeHeadingOffset = readWord(40);
	riverModeSpeedOffset = readWord(42);
	riverModeDelay = EEPROM.read(44);
}

bool confirmPrompt(bool PIDMessage) {
	mydisp.clearScreen();
	
	PrintEZ("   PLEASE CONFIRM   ", 10, 0, 0, 0, 0, true, 20, 0);
	if (PIDMessage)	{
		PrintEZ("  Reset PID values?", 10, 0, 2, 0, 0, false, 20, 0);
	}
	else {
		PrintEZ("   Factory Reset?  ", 10, 0, 2, 0, 0, false, 20, 0);
	}

	menuInitialize(); 
	do {

		PrintEZ(" No ", 20, 2, 4, 0, 0, menuItem == 1, 20, 0);
		PrintEZ(" Yes ", 20, 9, 4, 0, 0, menuItem == 2, 20, 0);
		switch (read_encoder()) {
		case 1:  // ENCODER UP
			menuItem--;
			if (menuItem < 1) menuItem = 2;
			break;
		case 2:    //ENCODER DOWN
			menuItem++;
			if (menuItem > 2) menuItem = 1;
			break;
		case 4:  // ENCODER BUTTON SHORT PRESS
			stillSelecting = false;
			if (menuItem == 1) {
				return false;
			}
			else {
				return true;
			}
			break;
		}
	} while (stillSelecting);
}

void servoMenu() {
	menuInitialize(); 
	do {
		PrintEZ("Min Servo: ", 10, 0, 0, 0, 0, menuItem == 1, 0, 0);
		PrintEZ(minServo, 10, 12, 0, 0, 0, menuItemSelected == 1, 20, 1);
		
		PrintEZ("Max Servo: ", 10, 0, 1, 0, 0, menuItem == 2, 0, 0);
		PrintEZ(maxServo, 10, 12, 1, 0, 0, menuItemSelected == 2, 20, 1);
		
		PrintEZ("Test: ", 10, 0, 2, 0, 0, menuItem == 3, 20, 0);

		if (menuItemSelected == 3) {
			PrintEZ("Testing", 10, 7, 2, 0, 0, true, 0, 0);
		}
		else {
			PrintEZ("Off", 10, 7, 2, 0, 0, false, 0, 4);
		}
		mydisp.setPrintPos(0, 5);
		mydisp.print("Servo Position: "); mydisp.print(pos); mydisp.print("  ");

		myservo.writeMicroseconds(pos);
		delay(100);
	
		switch (read_encoder()) {
		case 4:  // ENCODER BUTTON SHORT PRESS
			if (menuItemSelected == 0) {
				menuItemSelected = menuItem;
			}
			else {
				menuItemSelected = 0;
			}
			break;
		case 8:  // ENCODER BUTTON LONG PRESS
			myPID.SetOutputLimits(minServo, maxServo);
			configurationMenu();
			break;
		case 16:  // ENCODER BUTTON NULL
			if (menuItemSelected == 3) {
				pos += 10 * S;
				if (pos <= minServo) S = 1;
				if (pos >= maxServo) S = -1;
			}
			break;
		default:
			switch (menuItemSelected) {
			case 1: // MIN SERVO
				minServo -= encoderChange * 10;
				if (minServo <= 950) minServo = 950;
				if (minServo >= maxServo) minServo = maxServo - 10;
				pos = minServo;
				break;
			case 2: // MAX SERVO
				maxServo -= encoderChange * 10;
				if (maxServo <= minServo) maxServo = minServo + 10;
				if (maxServo >= 1950) maxServo = 1950;
				pos = maxServo;
				break;
			default:
				changeMenuItem(3);
				break;
			}
			break;
		}
	} while (stillSelecting);
}

void returnToMainDisp()
{
	stillSelecting = false;
	buttonTimes = 0;
	
	EEPROM.update(1, PonEKp * 100);
	EEPROM.update(2, PonEKi * 100);
	EEPROM.update(3, PonEKd * 100);
	EEPROM.update(4, PonMKp * 100);
	EEPROM.update(5, PonMKi * 100);
	EEPROM.update(6, PonMKd * 100);
	updateWord(7, accel100);
	EEPROM.update(9, cylCoeff);
	if (mph) EEPROM.update(10, 1); else EEPROM.update(10, 0);
	EEPROM.update(11, hourOffset);
	EEPROM.update(12, maxServo / 10);
	EEPROM.update(13, minServo / 10);
	if (celsius) EEPROM.update(14, 1); else EEPROM.update(14, 0);
	updateWord(15, target100);
	EEPROM.update(17, Contrast);
	updateWord(18, targetRPM);
	EEPROM.update(20, warnDrainPlug);
	EEPROM.update(21, menuTimeout);
	updateWord(22, voltageWarning100);
	EEPROM.update(24, twentyFourHourClockMode);
	EEPROM.update(25, displayThrottle);
	EEPROM.update(26, displayTime);
	EEPROM.update(27, displayVoltage);
	EEPROM.update(28, displayCourse);
	EEPROM.update(29, displayWaterTemp);
	EEPROM.update(30, displayAirTemp);
	EEPROM.update(31, displayRPM);
	EEPROM.update(32, limitRPM);
	EEPROM.update(33, limitSpeed);
	EEPROM.update(34, RPMLimit);
	updateWord(35, speedLimit100);
	EEPROM.update(37, riverModeEnabled);
	updateWord(38, riverModeHeading);
	updateWord(40, riverModeHeadingOffset);
	updateWord(42, riverModeSpeedOffset);
	EEPROM.update(44, riverModeDelay);
}

void updateWord(unsigned address, unsigned value)
{
	EEPROM.update(address, highByte(value));
	EEPROM.update(address + 1, lowByte(value));
}

//*******************************************************************
/* FUNCTION: readWord
   DESCRIPTION: used for reading large value from EEPROM
*/
unsigned readWord(unsigned address)
{
	return word(EEPROM.read(address), EEPROM.read(address + 1));
}

//*******************************************************************
/* FUNCTION: updateTemperatureVoltRead
   DESCRIPTION: updates the temperature and voltage readings for use by the main display
*/
void updateTemperatureVoltRead()
{
	//READS VOLTAGE

	for (i = 1; i < 3; i++)  voltage = analogRead(0);
	voltage = (float)voltage / 35.2;

	//READS AIR TEMP
	airSensor.requestTemperatures();
	airTemp = airSensor.getTempCByIndex(0);
	if (airTemp == 129) airTemp = 0;
	else if (!celsius)  airTemp = airTemp * 9 / 5 + 32;

	//READS WATER TEMP
	wtrSensor.requestTemperatures();
	wtrTemp = wtrSensor.getTempCByIndex(0);
	if (wtrTemp == 129) wtrTemp = 0;
	else if (!celsius) wtrTemp = wtrTemp * 9 / 5 + 32;
}

//*******************************************************************
/* FUNCTION: SpeedBreakdown
   DESCRIPTION: Used to separate a speed into integer values for either the whole part of the speed (Whole = true) or decimal part (Whole = false)
*/
int SpeedBreakdown(int Breakdown100, bool Whole)
{
	if (!mph) {
		SpeedInt = Breakdown100 * 1.61;
		SpeedInt = SpeedInt / 10;
	}
	else {
		SpeedInt = Breakdown100 / 10;
	}
	if (Whole) {
		speedBreakdown = SpeedInt / 10;
	}
	else {
		speedBreakdown = SpeedInt % 10;
	}
	return speedBreakdown;
}

//*******************************************************************
/* FUNCTION: DecimalBreakdown
   DESCRIPTION: Used to separate a decimal into integer values for either the whole part of the decimal (Whole = true) or decimal part (Whole = false)
*/
int DecimalBreakdown(int Breakdown100, bool Whole)
{
	Breakdown100 = Breakdown100 / 10;
	
	if (Whole) {
		decimalBreakdown = Breakdown100 / 10;
	}
	else {
		decimalBreakdown = Breakdown100 % 10;
	}
	return decimalBreakdown;
}

void PrintSpeedUnit(bool inverted)
{
	mydisp.print(" ");
	if (mph) {
		PrintEZ("MPH", 10, -1, -1, 0, 0, inverted, 20, 0);
	}
	else {
		PrintEZ("KPH", 10, -1, -1, 0, 0, inverted, 20, 0);
	}
}

void PrintShowHide(bool setting, bool inverted)
{
	mydisp.print(" ");
	if (setting) {
		PrintEZ("Show", 10, -1, -1, 0, 0, inverted, 20, 0);
	}
	else {
		PrintEZ("Hide", 10, -1, -1, 0, 0, inverted, 20, 0);
	}
}

//*******************************************************************
/* FUNCTION: headingError
   DESCRIPTION: Used to determine how close the current courseDegree is to desired heading.
*/
int headingError(int courseDegree, int heading)
{
	int error = heading - courseDegree;
	if (error > 180) error -= 360;
	if (error < -180) error += 360;
	return abs(error);
}

//*******************************************************************
/* FUNCTION: PrintSetting
   DESCRIPTION: Used to print the appropriate text for a bool or byte setting
*/
void PrintSetting(byte setting, bool inverted)
{
	mydisp.print(" ");
	switch (setting)
	{
	case 0:
		PrintEZ("Off", 10, -1, -1, 0, 0, inverted, 20, 0);
		PrintEZ("   ", 10, -1, -1, 0, 0, false, 20, 0);
		break;
	case 1:
		PrintEZ("On", 10, -1, -1, 0, 0, inverted, 20, 0);
		PrintEZ("   ", 10, -1, -1, 0, 0, false, 20, 0);
		break;
	case 2:
		PrintEZ("Always", 10, -1, -1, 0, 0, inverted, 20, 0);
		break;
	}

}

//*******************************************************************
/* FUNCTION: BeforePrintEZ
   DESCRIPTION: Used to set multiple print parameters before printing using the PrintEZ function
*/
void BeforePrintEZ(int fontSize, int posX, int posY, int offsetX, int offsetY, bool inverted)
{
	mydisp.setFont(fontSize);
	if (posX >= 0 || posY >= 0) mydisp.setPrintPos(posX, posY);
	if (offsetX != 0 || offsetY != 0) mydisp.setTextPosOffset(offsetX, offsetY);

	//Print font in white on black if printing inverted
	if (inverted) {
		mydisp.setColor(0);
		mydisp.setBgColor(1);
	}
}

//*******************************************************************
/* FUNCTION: AfterPrintEZ
   DESCRIPTION: Used to set multiple print parameters after printing using the PrintEZ function
*/
void AfterPrintEZ(bool inverted, int delayAfter, int spacesAfter)
{
	//Return font to normal black on white state
	if (inverted) {
		mydisp.setColor(1);
		mydisp.setBgColor(0);
	}
	for (int i = 0; i < spacesAfter; ++i)
		mydisp.print(' ');
	if (delayAfter > 0) delay(delayAfter);
}

//*******************************************************************
/* FUNCTION: PrintEZ
   DESCRIPTION: Used to set multiple print parameters in one line of code
*/
void PrintEZ(const char* text, int fontSize, int posX, int posY, int offsetX, int offsetY, bool inverted, int delayAfter, int spacesAfter)
{
	BeforePrintEZ(fontSize, posX, posY, offsetX, offsetY, inverted);
	mydisp.print(text);
	AfterPrintEZ(inverted, delayAfter, spacesAfter);
}

//*******************************************************************
/* FUNCTION: PrintEZ
   DESCRIPTION: Used to set multiple print parameters in one line of code
*/
void PrintEZ(double text, int fontSize, int posX, int posY, int offsetX, int offsetY, bool inverted, int delayAfter, int spacesAfter)
{
	BeforePrintEZ(fontSize, posX, posY, offsetX, offsetY, inverted);
	mydisp.print(text);
	AfterPrintEZ(inverted, delayAfter, spacesAfter);
}

//*******************************************************************
/* FUNCTION: PrintEZ
   DESCRIPTION: Used to set multiple print parameters in one line of code
*/
void PrintEZ(int text, int fontSize, int posX, int posY, int offsetX, int offsetY, bool inverted, int delayAfter, int spacesAfter)
{
	BeforePrintEZ(fontSize, posX, posY, offsetX, offsetY, inverted);
	mydisp.print(text);
	AfterPrintEZ(inverted, delayAfter, spacesAfter);
}

//*******************************************************************
/* FUNCTION: PrintEZ
   DESCRIPTION: Used to set multiple print parameters in one line of code
*/
void PrintEZ(byte text, int fontSize, int posX, int posY, int offsetX, int offsetY, bool inverted, int delayAfter, int spacesAfter)
{
	BeforePrintEZ(fontSize, posX, posY, offsetX, offsetY, inverted);
	mydisp.print(text);
	AfterPrintEZ(inverted, delayAfter, spacesAfter);
}

static void printFloat(float val, bool valid, int len, int prec)
{
	if (!valid)
	{
		while (len-- > 1)
			mydisp.print('*');
		mydisp.print(' ');
	}
	else
	{
		mydisp.print(val, prec);
		int vi = abs((int)val);
		int flen = prec + (val < 0.0 ? 2 : 1); // . and -
		flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
		for (int i = flen; i < len; ++i)
			mydisp.print(' ');
	}
}

void changeMenuItem(byte maxMenuItems) {
	if (encoderChange < 0) {
		menuItem++;
		if (menuItem > maxMenuItems) menuItem = 1;
	}
	else if (encoderChange > 0) {
		menuItem--;
		if (menuItem < 1) menuItem = maxMenuItems;
	}
}

double adjustPIDValue(double PIDValue) {
	PIDValue = PIDValue * 100;
	PIDValue -= encoderChange;
	if (PIDValue >= 100) PIDValue = 100;
	if (PIDValue <= 0) PIDValue = 0;
	PIDValue /= 100;
	return PIDValue;
}

void menuInitialize() {
	menuItemSelected = 0;
	menuItem = 1;
	startMillis = millis();
	stillSelecting = true;
}

int boundsCheck(int value, int min, int max) {
	if (value < min) {
		return min;
	}
	else if (value> max){
		return max;
	}
	else {
		return value;
	}
}