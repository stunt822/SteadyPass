

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
#include <TinyGPS++.h>
#include <Servo.h>
#include <SPI.h>
#include <OneWire.h>
#include <EEPROM.h>
#define _Digole_Serial_SPI_
#include <DigoleSerial.h>
#include <PID_v1.h>
#include <DallasTemperature.h>
//---------------------

double CurrentVersion = 21;

//*************INITIALIZING DEFINITIONS*************
byte menuItemSelected = 0;
boolean firstLoopOnStart = true;
boolean firstMainDispLoop = true;
boolean stillSelecting = true;
boolean calcPonMMode = false;
boolean gpsSignalFlag = false;
double Setpoint, Input, Output, gap;
double PonMKp = 0.25, PonMKi = 0.50, PonMKd = 0.05, PonEKp = 0.30, PonEKi = 0.25, PonEKd = 0.05;
double KpInput = PonMKp;
double KiInput = PonMKi;
double KdInput = PonMKd;
PID myPID(&Input, &Output, &Setpoint, KpInput, KiInput, KdInput, P_ON_M, REVERSE);

TinyGPSPlus gps;                                //required for TinyGPSplus Library
DigoleSerialDisp mydisp(9, 8, 10);              //Pin Config SPI | 9: data | 8:clock | 10: SS | you can assign 255 to SS, and hard ground SS pin on module
boolean led = false;                            //LED on | off
float voltage = 0, volt1, volt2;                //voltage value
byte cylCoeff = 6;                              //Number of Cylinders
boolean gpsMode = true;                         //GPS mode (true) or RPM mode (false)
byte mode = 0;                                  //Calculations mode 0=off 1=RPM 2=MPH/KPH


//*****PIN CONFIGURATION******
//0 and 1 = GPS
const byte PinCLK = 3;             //encoder second pin
const byte PinDT = 2;              // Used for reading DT signal of encoder
const byte PinSW = 4;              //encoder button
const byte servoPin = 5;           //servo PWM connected here

#if defined(__AVR_ATmega2560__)
	//Code in here will only be compiled if an Arduino Mega is used.
	const byte rpmPin = 44;        //Engine RPM input connected here
#else
	//Code in here will only be compiled if an Arduino Leonardo is used.
	const byte rpmPin = 7;         //Engine RPM input connected here
#endif

//8 and 9 - LCD, defined above
//open - 10 - LCD SS?
const byte tempWtrPin = 11;        //temperature water
const byte tempAirPin = 12;        //temperature air
const byte voltPin = A0;           //battery voltage divider
//OPEN A1, A2, A3, A4, A5

//----------------------

//***TEMP SENSOR INITIALIZATION***
OneWire  wtr(tempWtrPin);
OneWire  air(tempAirPin);
DallasTemperature wtrSensor(&wtr);
DallasTemperature airSensor(&air);
byte data[12];
byte addr[8];
byte present = 0;
byte i;
byte Temp, airTemp, wtrTemp;
boolean celsius = true;
//------END TEMP------

//*************SPEED INITIALIZATION*************
int speedValue = 0, speedGps = 0;
int TargetSpeedInt, target100 = 500;
int speed100, pos100;
int gpsDegree;//,gpsHDOP,gpsSats,gpsLat,gpsLng,gpsAlt;
int gpsSignalCounter = 0;
int EKp100, EKi100, EKd100, Kp100, Ki100, Kd100;
char gpsCourse[9];
boolean mph = true;
unsigned long delayCheck = 0;
unsigned long throttleCheck = 0;
unsigned long temperatureCheck = 0;
unsigned long gpsCourseCheck = 0;
byte targetSpeedWhole = TargetSpeedInt / 10;
byte targetSpeedDecimal = TargetSpeedInt % 10;
byte Contrast;
byte menuItem = 1;
//*************RPM INITIALIZATION*************
volatile unsigned long duration = 0; // accumulates pulse width
volatile unsigned long pulsecount = 0; //incremented by interrupt
volatile unsigned long previousMicros = 0;
int targetRPM = 3000;
byte CalcMode = 0;
//*************SERVO INITIALIZATION*************
Servo myservo;
int minServo = 950;
int maxServo = 1950;
int pos = minServo;
int S = 10;
float throttlePer = 100;

//*************ENCODER INITIALIZATION*************
volatile boolean TurnDetected = false;
volatile boolean up = false;
byte buttonTimes = 0;
byte inMenuPress = 0;
byte buttonTarget = 7;
boolean mainDisplay = true;
volatile long encoder = 0;
int encoderChange = 0;
byte hours, minutes, seconds, hourOffset = 12;
boolean speedMode = false;
unsigned long startMillis = 0;
unsigned long currentMillis;
const unsigned long menuTimeout = 10000;

//*************FUNCTION INITIALIZATION*************
int   readRpm();                              //read RPM and reset counters
void  rpmIntHandler();                        //called by interrupt, increments counters
static void smartDelay(unsigned long ms);     //Reads Data from GPS device
void  isr();                                  //encoder - Interrupt service routine is executed when a HIGH to LOW transition is detected on CLK
void  PIDCalculations();                      //Calculate Speed Adjustments based on MPH or RPM depending on mode
void  MainDisplay();                          //Runs the Main LCD screen
void  Menu();                                 //runs the USER menu options
int read_encoder();                           //Reads knob up/down/press/longpress for menu cases
void updateTemperatureVoltRead();
void PrintEZ(const char* text, int fontSize, int posX, int posY, int offsetX, int offsetY, bool inverted, int delayAfter);
void PrintEZ(double text, int fontSize, int posX, int posY, int offsetX, int offsetY, bool inverted, int delayAfter);
void PrintEZ(int text, int fontSize, int posX, int posY, int offsetX, int offsetY, bool inverted, int delayAfter);
void PrintEZ(byte text, int fontSize, int posX, int posY, int offsetX, int offsetY, bool inverted, int delayAfter);

//************************************************************************************************|
//*********** PERFORM STARTUP TASKS | DISPLAY STARTUP | GPS CONFIGURATION | READ MEMORY **********|
//************************************************************************************************|
void setup()
{
	//for debug---------
	Serial.begin(9600);
	delay(100);
	//------------------
	Serial1.begin(9600);   //GPS device
	delay(3000);
	mydisp.begin();
	mydisp.clearScreen();  delay(100);
	PrintEZ("STEADYPASS", 30, 3, 1, 0, 0, false, 1000);
	PrintEZ(" Version: ", 30, 0, 2, 0, 0, false, 0);
	PrintEZ(CurrentVersion, 30, 10, 2, 0, 0, false, 1000);
	PrintEZ(" DRAIN PLUG IN? ", 30, 0, 3, 0, 0, false, 3000);
	mydisp.clearScreen();
	delay(500);
	if (EEPROM.read(200) != 2)
	{
		PrintEZ(" INITIALIZE MEM  ", 30, 0, 3, 0, 0, false, 2000);
		EEPROM.update(11, 6);      //cylCoeff
		EEPROM.update(18, 1);      //Speed Units
		EEPROM.update(19, 12);     //Time Offset
		EEPROM.update(20, 195);    //MIN Throttle
		EEPROM.update(21, 95);     //MAX Throttle
		EEPROM.update(29, 1);      //Temp Units
		EEPROM.update(30, 10);     //Startup Target
		EEPROM.update(31, 1);      //Startup Target
		//EEPROM.update(32, 0);
		EEPROM.update(33, 30);     //Contrast
		EEPROM.update(34, 11);	  //Startup RPM
		EEPROM.update(35, 184);	  //Startup RPM
		EEPROM.update(200, 2);     //Preferences Reset Bit
		mydisp.clearScreen();
		delay(500);
	}
	if (EEPROM.read(201) != 3)
	{
		PrintEZ("UPDATING TUNING", 30, 0, 3, 0, 0, false, 5000);
		EEPROM.update(12, 0);  //KP whole
		EEPROM.update(13, 50); //KP dec
		EEPROM.update(14, 0);  //KI whole
		EEPROM.update(15, 25); //KI dec
		EEPROM.update(16, 0);  //KD whole
		EEPROM.update(17, 5);  //KD dec
		EEPROM.update(50, 0);  //KP whole
		EEPROM.update(51, 30); //KP dec
		EEPROM.update(52, 0);  //KI whole
		EEPROM.update(53, 25); //KI dec
		EEPROM.update(54, 0);  //KD whole
		EEPROM.update(55, 5);  //KD dec
		EEPROM.update(201, 3); //tuning Update Bit
		mydisp.clearScreen();
		delay(500);
	}
	PrintEZ("ACQUIRING SIGNAL", 30, 0, 1, 0, 0, false, 0);
	PrintEZ("  PLEASE WAIT ", 30, 0, 3, 0, 0, false, 1000);
	pinMode(PinCLK, INPUT);                     //rotary encoder
	pinMode(PinDT, INPUT);                      //rotary encoder
	pinMode(PinSW, INPUT_PULLUP);               //rotary encoder button
	attachInterrupt(0, isr, RISING);            // interrupt 0 pin 3 for encoder
	//pinMode(7,INPUT);                         //RPM pin, high when no pulse.
	attachInterrupt(4, rpmIntHandler, CHANGE);  //  RPM - pin 7 is grounded on pulse
	myservo.attach(servoPin);                   // attaches the servo on pin  to the servo object
	//PID Variables//
	Input = speed100;
	Setpoint = target100;
	//Turn on PID//
	myPID.SetMode(AUTOMATIC);


	//******************LOADING SAVED SETTINGS*******************

	PonEKp = readWord(12) / 100.00;
	// 13 taken
	PonEKi = readWord(14) / 100.00;
	// 15 taken
	PonEKd = readWord(16) / 100.00;
	// 17 taken
	PonMKp = readWord(50) / 100.00;
	// 51 taken
	PonMKi = readWord(52) / 100.00;
	// 53 taken
	PonMKd = readWord(54) / 100.00;
	// 55 taken
	cylCoeff = EEPROM.read(11);
	mph = EEPROM.read(18);
	hourOffset = EEPROM.read(19);
	maxServo = 10 * EEPROM.read(20);
	minServo = 10 * EEPROM.read(21);
	celsius = EEPROM.read(29);
	target100 = readWord(30);
	Contrast = EEPROM.read(33);
	targetRPM = readWord(34);

	EKp100 = PonEKp * 100;
	EKi100 = PonEKi * 100;
	EKd100 = PonEKd * 100;
	Kp100 = PonMKp * 100;
	Ki100 = PonMKi * 100;
	Kd100 = PonMKd * 100;
	//----------------LOADED VALUES COMPLETE-------------------

	myPID.SetOutputLimits(minServo, maxServo);
	myservo.writeMicroseconds(minServo);
	//******************LOADING GPS CONFIGURATION*******************
	//Serial1.println("$PMTK314,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29");   //Sets GPS to GTV only
	Serial1.println("$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29");     //Sets GPS to RMC only
	delay(100);
	Serial1.println("$PMTK220,200*2C");  //Set GPS to 5hz
	delay(100);
}

//**RUN MAIN**|
void loop()
{
	//  if (led) {
	//    digitalWrite(13, LOW);
	//    led = false;
	//  }
	//  else {
	//    digitalWrite(13, HIGH);
	//    led = true;
	//  }
	smartDelay(50);
	//delay(200);
	if (firstLoopOnStart) {
		mydisp.clearScreen();
		delay(100);
		mydisp.setContrast(Contrast);
	}
	if (gps.speed.isValid()) {
		speed100 = 100 * gps.speed.mph();
		gpsDegree = gps.course.deg();
	}

	//UPDATE TEMPS/VOLTAGE CHECK
	if (temperatureCheck < millis()) {
		temperatureCheck = millis() + 5000;
		updateTemperatureVoltRead();
	}

	//GPS COURSE CHECK
	if (gpsCourseCheck < millis()) {
		gpsCourseCheck = millis() + 1000;
		if (gpsDegree > 337 && gpsDegree <= 22) {
			gpsCourse[1] = 'N';
			gpsCourse[2] = ' ';
		}
		if (gpsDegree > 22 && gpsDegree <= 67) {
			gpsCourse[1] = 'N';
			gpsCourse[2] = 'E';
		}
		if (gpsDegree > 67 && gpsDegree <= 112) {
			gpsCourse[1] = 'E';
			gpsCourse[2] = ' ';
		}
		if (gpsDegree > 112 && gpsDegree <= 157) {
			gpsCourse[1] = 'S';
			gpsCourse[2] = 'E';
		}
		if (gpsDegree > 157 && gpsDegree <= 202) {
			gpsCourse[1] = 'S';
			gpsCourse[2] = ' ';
		}
		if (gpsDegree > 202 && gpsDegree <= 247) {
			gpsCourse[1] = 'S';
			gpsCourse[2] = 'W';
		}
		if (gpsDegree > 247 && gpsDegree <= 292) {
			gpsCourse[1] = 'W';
			gpsCourse[2] = ' ';
		}
		if (gpsDegree > 292 && gpsDegree <= 337) {
			gpsCourse[1] = 'N';
			gpsCourse[2] = 'W';
		}
	}

	//RUN CURRENT CALCULATION MODE//
	if (mode == 0) {
		pos = minServo;
	}
	else {
		if (mode == 2) {
			speedMode = false;
		}
		else {
			speedMode = true;
		}
		PIDCalculations();
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
	if (firstLoopOnStart) {
		firstLoopOnStart = false;
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
	do
	{
		while (Serial1.available())
		{
			gps.encode(Serial1.read());
		}
	} while ((!gps.speed.isUpdated()));
}
//End Function

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
//End Function

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
//End Function

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
		up = true;
		encoder--;
	}
	else
	{
		up = false;
		encoder++;
	}
	TurnDetected = true;
}
//End Function

//*******************************************************************
/* FUNCTION: PIDCalculations
   INPUT: Speed100 or current RPM | target100 or targetRPM
   RETURN: pos
   DESCRIPTION: Runs an equation to calculate Change in speed based off MPH | needs better description later
*/
void PIDCalculations()
{
	gap = (Setpoint - Input);
	//  Serial.print(gap);
	  //PID INPUTS//

	if (speedMode) {
		if (gap >= 500) {
			if (!calcPonMMode) {
				calcPonMMode = true;
				KpInput = PonMKp;
				KiInput = PonMKi;
				KdInput = PonMKd;
				CalcMode = P_ON_M;
			}
		}
		else if (gap <= 0) {
			if (calcPonMMode) {
				calcPonMMode = false;
				KpInput = PonEKp;
				KiInput = PonEKi;
				KdInput = PonEKd;
				CalcMode = P_ON_E;
			}
		}
		myPID.SetTunings(KpInput, KiInput, KdInput, CalcMode);
		Input = speed100;
		Setpoint = target100;
	}
	else {
		Input = readRpm();
		Setpoint = targetRPM;
		PonEKd = 0.00;
		myPID.SetTunings(PonEKp, PonEKi, PonEKd, P_ON_E);
	}
	myPID.Compute();
	pos = Output;
	//  Serial.print(Output); Serial.print("|||");
	//  Serial.print(myPID.GetMode()); Serial.print("|||---");
}
//End Function

//*******************************************************************
/* FUNCTION:  MainDisplay()
   DESCRIPTION: This displays everything that you see
*/
void MainDisplay()
{
	//boat in motion detection
	// if (speed100 >= 500){boatInMotion = true;}
	// else {boatInMotion = false;}

	//GPS Time Display//
	hours = gps.time.hour() + hourOffset - 12;
	if (hours < 0) hours += 24;
	if (hours > 24) hours -= 24;
	minutes = gps.time.minute();
	seconds = gps.time.second();

	//Throttle percent update
	if (delayCheck < millis()) {
		delayCheck = millis() + 500;
		throttlePer = maxServo - pos;
		throttlePer = throttlePer / (maxServo - minServo);
		throttlePer *= 100;
	}

	//update MPH or KPH
	if (!mph) {
		TargetSpeedInt = target100 * 1.61;
		TargetSpeedInt = TargetSpeedInt / 10;
	}
	else {
		TargetSpeedInt = target100 / 10;
	}
	targetSpeedWhole = TargetSpeedInt / 10;
	targetSpeedDecimal = TargetSpeedInt % 10;
	int c = targetSpeedDecimal + targetSpeedWhole;

	//Time//
	mydisp.setFont(10);
	mydisp.setPrintPos(0, 1);
	mydisp.print(hours); mydisp.print(":"); if (minutes < 10) mydisp.print("0"); mydisp.print(minutes); mydisp.print(":"); if (seconds < 10) mydisp.print("0"); mydisp.print(seconds); mydisp.print("  ");

	//print word "GPS"//
	PrintEZ("GPS", 10, 18, 3, 2, -2, false, 0);

	//Print word "MPH" or "KPH"//
	mydisp.setPrintPos(18, 4);
	mydisp.setTextPosOffset(2, -3);
	if (mph) mydisp.print("MPH"); else mydisp.print("KPH");

	//Print word "POWER"//
	mydisp.setPrintPos(0, 0);
	mydisp.print("P:");
	mydisp.print((int)(throttlePer)); mydisp.print("");
	if (calcPonMMode) {
		mydisp.print("M  ");
	}
	else {
		mydisp.print("E  ");
	}

	//Print target MPH or KPH depending on mode selection
	mydisp.setFont(18);
	mydisp.setPrintPos(6, 0);

	switch (mode) {
	case 0:
		mydisp.print("   OFF  ");
		break;
	case 1:
		if (c < 0) {
			mydisp.print(" -");
		}
		else if (targetSpeedWhole < 10) {
			mydisp.print("  ");
		}
		else {
			mydisp.print(" ");
		}
		if (targetSpeedWhole < 0) {
			targetSpeedWhole = -targetSpeedWhole;
		}
		mydisp.print(targetSpeedWhole); mydisp.print(".");
		if (targetSpeedDecimal < 0) {
			targetSpeedDecimal = -targetSpeedDecimal;
		}
		mydisp.print(targetSpeedDecimal);
		if (mph) mydisp.print("MPH "); else mydisp.print("KPH ");
		break;
	case 2:
		mydisp.print(" ");
		mydisp.print(targetRPM);
		mydisp.print("RPM");
		break;
	}


	//Prints Voltage
	PrintEZ("V ", 10, 0, 2, 0, 0, false, 0);
	mydisp.print(voltage); mydisp.print("  ");

	//print GPS directional
	PrintEZ("DIR ", 10, 0, 3, 0, 0, false, 0);
	mydisp.print(gpsCourse[1]); mydisp.print(gpsCourse[2]);


	// Prints Water Temp and Air Temp
	mydisp.setPrintPos(0, 4);
	mydisp.print("WTR "); mydisp.print(wtrTemp); if (celsius)  mydisp.print("c  "); else mydisp.print("F  ");
	mydisp.setPrintPos(0, 5);
	mydisp.print("AIR "); mydisp.print(airTemp); if (celsius)  mydisp.print("c  "); else mydisp.print("F  ");

	//Prints current RPM reading
	mydisp.setPrintPos(0, 6);
	mydisp.print("RPM "); mydisp.print(readRpm()); mydisp.print("    ");

	//Prints the Main Speed value On Display
	PrintEZ("", 203, 0, 0, 46, 22, false, 0);
	if (speedValue < 100) mydisp.print('0');
	mydisp.print(speedValue / 10);

	//Prints the decimal value of the Main Speed
	PrintEZ(speedValue % 10, 201, 0, 0, 111, 43, false, 0);
	firstMainDispLoop = false;
}
//End Function
//-------------------------------------------------------------------






int read_encoder() {
#define btnUp         1
#define btnDown       2
#define btnPress      4
#define btnLongPress  8
#define btnNull       16

	if (TurnDetected) {
		if (encoder > 0) {
			encoderChange = encoder;
			encoder = 0;
			TurnDetected = false;
			return btnUp;
		}
		else if (encoder < 0) {
			encoderChange = encoder;
			encoder = 0;
			TurnDetected = false;
			return btnDown;
		}
		else {
			encoderChange = 0;
			TurnDetected = false;
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
			//mydisp.setFont(20);
			//mydisp.setPrintPos(0, 0);
			//mydisp.print("ENTER");
			//delay(500);
			//mydisp.clearScreen();
			return btnPress;
		}
		else {
			return btnNull;
		}

		if (inMenuPress > buttonTarget) {
			inMenuPress = 0;
			mydisp.clearScreen();
			PrintEZ("BACK", 20, 0, 0, 0, 0, false, 0);
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
	menuItemSelected = 0;
	menuItem = 1;
	startMillis = millis();
	do {
		stillSelecting = true;

		PrintEZ(" Tuning ", 20, 0, 0, 0, 0, menuItem == 1, 20);
		PrintEZ(" Servo ", 20, 0, 1, 0, 0, menuItem == 2, 20);
		PrintEZ(" Preferences ", 20, 0, 2, 0, 0, menuItem == 3, 20);
		switch (read_encoder())	{
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
			if (menuItem == 1) tuningMenu();
			if (menuItem == 2) servoMenu();
			if (menuItem == 3) preferencesMenu();
			break;
		case 8:  // ENCODER BUTTON LONG PRES
			stillSelecting = false;
			returnToMainDisp();
			break;
		case 16:  // ENCODER BUTTON NULL
			currentMillis = millis();
			if ((currentMillis - startMillis) >= menuTimeout) {
				mydisp.clearScreen();
				returnToMainDisp();
			}
			break;
		}
	} while (stillSelecting == true);
}


void tuningMenu() {
	menuItem = 1;
	menuItemSelected = 0;
	startMillis = millis();

	do {

		PrintEZ("Target P: ", 10, 0, 0, 0, 0, menuItem == 1, 0);
		PrintEZ(PonEKp, 10, 11, 0, 0, 0, menuItemSelected == 1, 25);

		PrintEZ("Target I: ", 10, 0, 1, 0, 0, menuItem == 2, 0);
		PrintEZ(PonEKi, 10, 11, 1, 0, 0, menuItemSelected == 2, 25);

		PrintEZ("Target D: ", 10, 0, 2, 0, 0, menuItem == 3, 0);
		PrintEZ(PonEKd, 10, 11, 2, 0, 0, menuItemSelected == 3, 25);

		PrintEZ("Accel P: ", 10, 0, 4, 0, 0, menuItem == 4, 0);
		PrintEZ(PonMKp, 10, 10, 4, 0, 0, menuItemSelected == 4, 25);

		PrintEZ("Accel I: ", 10, 0, 5, 0, 0, menuItem == 5, 0);
		PrintEZ(PonMKi, 10, 10, 5, 0, 0, menuItemSelected == 5, 25);

		PrintEZ("Accel D: ", 10, 0, 6, 0, 0, menuItem == 6, 0);
		PrintEZ(PonMKd, 10, 10, 6, 0, 0, menuItemSelected == 6, 25);

		stillSelecting = true;
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
			if ((currentMillis - startMillis) >= menuTimeout) {
				mydisp.clearScreen();
				stillSelecting = false;
				returnToMainDisp();
			}
			break;
		default:  // ENCODER TURNED
			startMillis = millis();

			switch (menuItemSelected) {
			case 1: // Target P
				PonEKp = PonEKp * 100;
				PonEKp -= encoderChange;
				if (PonEKp >= 100) PonEKp = 100;
				if (PonEKp <= 0) PonEKp = 0;
				EKp100 = PonEKp;
				PonEKp /= 100;
				break;
			case 2: // Target I
				PonEKi = PonEKi * 100;
				PonEKi -= encoderChange;
				if (PonEKi >= 100) PonEKi = 100;
				if (PonEKi <= 0) PonEKi = 0;
				EKi100 = PonEKi;
				PonEKi /= 100;
				break;
			case 3: // Target D
				PonEKd = PonEKd * 100;
				PonEKd -= encoderChange;
				if (PonEKd >= 100) PonEKd = 100;
				if (PonEKd <= 0) PonEKd = 0;
				EKd100 = PonEKd;
				PonEKd /= 100;
				break;
			case 4: // Accel P
				PonMKp = PonMKp * 100;
				PonMKp -= encoderChange;
				if (PonMKp >= 100) PonMKp = 100;
				if (PonMKp <= 0) PonMKp = 0;
				Kp100 = PonMKp;
				PonMKp /= 100;
				break;
			case 5: // Accel I
				PonMKi = PonMKi * 100;
				PonMKi -= encoderChange;
				if (PonMKi >= 100) PonMKi = 100;
				if (PonMKi <= 0) PonMKi = 0;
				Ki100 = PonMKi;
				PonMKi /= 100;
				break;
			case 6: // Accel D
				PonMKd = PonMKd * 100;
				PonMKd -= encoderChange;
				if (PonMKd >= 100) PonMKd = 100;
				if (PonMKd <= 0) PonMKd = 0;
				Kd100 = PonMKd;
				PonMKd /= 100;
				break;
			default: //No menu item selected
				if (encoderChange < 0) {
					menuItem++;
					if (menuItem > 6) menuItem = 1;
				}
				else if (encoderChange >0) {
					menuItem--;
					if (menuItem < 1) menuItem = 6;
				}
				break;
			}
			break;
		}
	} while (stillSelecting == true);
}


void servoMenu() {
	menuItemSelected = 0;
	menuItem = 1;
	do {
		PrintEZ("Min Servo: ", 10, 0, 0, 0, 0, menuItem == 1, 0);
		PrintEZ(minServo, 10, 12, 0, 0, 0, menuItemSelected == 1, 20);
		mydisp.print(" ");

		PrintEZ("Max Servo: ", 10, 0, 1, 0, 0, menuItem == 2, 0);
		PrintEZ(maxServo, 10, 12, 1, 0, 0, menuItemSelected == 2, 20);
		mydisp.print(" ");

		PrintEZ("Test: ", 10, 0, 2, 0, 0, menuItem == 3, 20);

		if (menuItemSelected == 3) {
			PrintEZ("Testing", 10, 7, 2, 0, 0, true, 0);
		}
		else {
			PrintEZ("Off    ", 10, 7, 2, 0, 0, false, 0);
		}
		mydisp.setPrintPos(0, 5);
		mydisp.print("Servo Position: "); mydisp.print(pos); mydisp.print("  ");

		myservo.writeMicroseconds(pos);
		delay(100);
		stillSelecting = true;

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
				if (encoderChange < 0) {
					menuItem++;
					if (menuItem > 3) menuItem = 1;
				}
				else if (encoderChange > 0) {
					menuItem--;
					if (menuItem < 1) menuItem = 3;
				}
				break;
			}
			break;
		}
	} while (stillSelecting == true);
}


void preferencesMenu() {
	menuItemSelected = 0;
	menuItem = 1;
	startMillis = millis();

	do {
		PrintEZ("Start Spd: ", 10, 0, 0, 0, 0, menuItem == 1, 0);
		PrintEZ(TargetSpeedWhole(), 10, 12, 0, 0, 0, menuItemSelected == 1, 0);
		PrintEZ(".", 10, -1, -1, 0, 0, menuItemSelected == 1, 0);
		PrintEZ(TargetSpeedDecimal(), 10, -1, -1, 0, 0, menuItemSelected == 1, 0);
		if (mph) {
			PrintEZ(" MPH ", 10, -1, -1, 0, 0, menuItemSelected == 1, 20);
		}
		else {
			PrintEZ(" KPH ", 10, -1, -1, 0, 0, menuItemSelected == 1, 20);
		}
		mydisp.print(" ");

		PrintEZ("Start RPM: ", 10, 0, 1, 0, 0, menuItem == 2, 0);
		PrintEZ(targetRPM, 10, 12, 1, 0, 0, menuItemSelected == 2, 20);

		PrintEZ("Engine Cyl: ", 10, 0, 2, 0, 0, menuItem == 3, 0);
		PrintEZ(cylCoeff, 10, 13, 2, 0, 0, menuItemSelected == 3, 20);

		PrintEZ("Clock Offset: ", 10, 0, 3, 0, 0, menuItem == 4, 0);
		PrintEZ(hourOffset - 12, 10, 15, 3, 0, 0, menuItemSelected == 4, 20);
		mydisp.print(" ");

		PrintEZ("Speed Units: ", 10, 0, 4, 0, 0, menuItem == 5, 0);
		if (mph) {
			PrintEZ(" MPH ", 10, -1, -1, 0, 0, menuItemSelected == 5, 20);
		}
		else {
			PrintEZ(" KPH ", 10, -1, -1, 0, 0, menuItemSelected == 5, 20);
		}

		PrintEZ("Temp Units: ", 10, 0, 5, 0, 0, menuItem == 6, 0);
		if (celsius) {
			PrintEZ("C", 10, 13, 5, 0, 0, menuItemSelected == 6, 20);
		}
		else {
			PrintEZ("F", 10, 13, 5, 0, 0, menuItemSelected == 6, 20);
		}

		PrintEZ("Contrast: ", 10, 0, 6, 0, 0, menuItem == 7, 0);
		PrintEZ(Contrast, 10, 11, 6, 0, 0, menuItemSelected == 7, 20);
		stillSelecting = true;
		switch (read_encoder()){
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
			if ((currentMillis - startMillis) >= menuTimeout) {
				mydisp.clearScreen();
				returnToMainDisp();
			}
			break;
		default:  // ENCODER TURNED
			startMillis = millis();

			switch (menuItemSelected) {
			case 1: // Start Speed
				target100 -= encoderChange *10;
				if (target100 < 500) target100 = 500;
				if (target100 >= 5000) target100 = 5000;
				break;
			case 2: // Start RPM
				targetRPM -= encoderChange *10;
				if (targetRPM < 500) targetRPM = 500;
				if (targetRPM >= 9900) targetRPM = 9900;
				break;
			case 3: // Engine Cylinders
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
			case 4: // Clock Offset
				hourOffset -= encoderChange;
				if (hourOffset < 0) hourOffset = 24;
				if (hourOffset > 24) hourOffset = 0;
				break;
			case 5: // Speed Units
				if (encoderChange || 0) mph = !mph;
				break;
			case 6: // Temp Units
				if (encoderChange || 0) celsius = !celsius;
				break;
			case 7: // Contrast
				Contrast -= encoderChange;
				if (Contrast >= 40) Contrast = 40;
				if (Contrast <= 20) Contrast = 20;
				mydisp.setContrast(Contrast);
				break;
			default:
				if (encoderChange < 0) {
					menuItem++;
					if (menuItem > 7) menuItem = 1;
				}
				else if (encoderChange > 0) {
					menuItem--;
					if (menuItem < 1) menuItem = 7;
				}
				break;
			}
			break;
		}
	} while (stillSelecting == true);
}


void returnToMainDisp()
{
	stillSelecting = false;
	buttonTimes = 0;
	firstMainDispLoop = true;
	loop();
	writeWord(12, EKp100);
	writeWord(14, EKi100);
	writeWord(16, EKd100);
	writeWord(50, Kp100);
	writeWord(52, Ki100);
	writeWord(54, Kd100);

	EEPROM.update(11, cylCoeff);
	if (mph) EEPROM.update(18, 1); else EEPROM.update(18, 0);
	EEPROM.update(19, hourOffset);
	EEPROM.update(20, maxServo / 10);
	EEPROM.update(21, minServo / 10);
	if (celsius) EEPROM.update(29, 1); else EEPROM.update(29, 0);
	writeWord(30, target100);
	EEPROM.update(33, Contrast);
	writeWord(34, targetRPM);
}


void writeWord(unsigned address, unsigned value)
{
	EEPROM.update(address, highByte(value));
	EEPROM.update(address + 1, lowByte(value));
}
//End Function
//-------------------------------------------------------------------


//*******************************************************************
/* FUNCTION: readWord
   DESCRIPTION: used for reading large value from EEPROM
*/
unsigned readWord(unsigned address)
{
	return word(EEPROM.read(address), EEPROM.read(address + 1));
} 
//End Function
//-------------------------------------------------------------------


//*******************************************************************
/* FUNCTION: updateTemperatureVoltRead
   DESCRIPTION: updates the temperature and voltage readings for use by the main display
*/
void updateTemperatureVoltRead()
{
	//READS VOLTAGE
	volt1 = voltage;
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

int TargetSpeedWhole()
{
	if (!mph) {
		TargetSpeedInt = target100 * 1.61;
		TargetSpeedInt = TargetSpeedInt / 10;
	}
	else {
		TargetSpeedInt = target100 / 10;
	}
	targetSpeedWhole = TargetSpeedInt / 10;
	return targetSpeedWhole;
}

int TargetSpeedDecimal()
{
	if (!mph) {
		TargetSpeedInt = target100 * 1.61;
		TargetSpeedInt = TargetSpeedInt / 10;
	}
	else {
		TargetSpeedInt = target100 / 10;
	}
	targetSpeedDecimal = TargetSpeedInt % 10;
	return targetSpeedDecimal;
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
//End Function
//-------------------------------------------------------------------

//*******************************************************************
/* FUNCTION: AfterPrintEZ
   DESCRIPTION: Used to set multiple print parameters after printing using the PrintEZ function
*/
void AfterPrintEZ(bool inverted, int delayAfter)
{
	//Return font to normal black on white state
	if (inverted) {
		mydisp.setColor(1);
		mydisp.setBgColor(0);
	}
	if (delayAfter > 0) delay(delayAfter);
}
//End Function
//-------------------------------------------------------------------


//*******************************************************************
/* FUNCTION: PrintEZ
   DESCRIPTION: Used to set multiple print parameters in one line of code
*/
void PrintEZ(const char* text, int fontSize, int posX, int posY, int offsetX, int offsetY, bool inverted, int delayAfter)
{
	BeforePrintEZ(fontSize, posX, posY, offsetX, offsetY, inverted);
	mydisp.print(text);
	AfterPrintEZ(inverted, delayAfter);
}
//End Function
//-------------------------------------------------------------------

//*******************************************************************
/* FUNCTION: PrintEZ
   DESCRIPTION: Used to set multiple print parameters in one line of code
*/
void PrintEZ(double text, int fontSize, int posX, int posY, int offsetX, int offsetY, bool inverted, int delayAfter)
{
	BeforePrintEZ(fontSize, posX, posY, offsetX, offsetY, inverted);
	mydisp.print(text);
	AfterPrintEZ(inverted, delayAfter);
}
//End Function
//-------------------------------------------------------------------

//*******************************************************************
/* FUNCTION: PrintEZ
   DESCRIPTION: Used to set multiple print parameters in one line of code
*/
void PrintEZ(int text, int fontSize, int posX, int posY, int offsetX, int offsetY, bool inverted, int delayAfter)
{
	BeforePrintEZ(fontSize, posX, posY, offsetX, offsetY, inverted);
	mydisp.print(text);
	AfterPrintEZ(inverted, delayAfter);
}
//End Function
//-------------------------------------------------------------------

//*******************************************************************
/* FUNCTION: PrintEZ
   DESCRIPTION: Used to set multiple print parameters in one line of code
*/
void PrintEZ(byte text, int fontSize, int posX, int posY, int offsetX, int offsetY, bool inverted, int delayAfter)
{
	BeforePrintEZ(fontSize, posX, posY, offsetX, offsetY, inverted);
	mydisp.print(text);
	AfterPrintEZ(inverted, delayAfter);
}
//End Function
//-------------------------------------------------------------------