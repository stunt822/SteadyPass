

/*
  _________ __                     .___       __________                        ____   ________
  /   _____//  |_  ____ _____     __| _/__.__. \______   \_____    ______ ______ \   \ /   /_   |
  \_____  \\   __\/ __ \\__  \   / __ <   |  |  |     ___/\__  \  /  ___//  ___/  \       / |   |
  /        \|  | \  ___/ / __ \_/ /_/ |\___  |  |    |     / __ \_\___ \ \___ \    \     /  |   |
  /_______  /|__|  \___  >____  /\____ |/ ____|  |____|    /____  /____  >____  >    \___/   |___|
  ================================================================================================
************************************************************************************************
  ---|FILE: SteadyPassV1.ino
  ---|Initial document Created By: Dimitry
  ---|Description: This is the code used to make everything work
  ---|  Change History
  ---| <17.xx - Dimitry   - All versions Prior to 17.00 have been soley created by Dimitry
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
  ---|  18.14 - ''''''''' - Added menu text to clarify long and short press (cancel and save),fixed startup speed rounding off issue,fixed KP and KI gain mixup, set default servo values tp 950 and 1950, Set contrast limits, Added more GPS data variables
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
boolean selectPonEKp = false;
boolean selectPonEKi = false;
boolean selectPonEKd = false;
boolean selectPonMKp = false;
boolean selectPonMKi = false;
boolean selectPonMKd = false;
boolean selectMaxServo = false;
boolean selectMinServo = false;
boolean selectStartSpeed = false;
boolean selectContrastMenu = false;
boolean selectCylinderMenu = false;
boolean selectClockMenu = false;
boolean selectTempMenu = false;
boolean selectMeasurementMenu = false;
boolean selectTestServo = false;
boolean firstLoopOnStart = true;
boolean firstMainDispLoop = true;
boolean stillSelecting = true;
boolean calcPonMMode = false;
boolean gpsSignalFlag = false;
double Setpoint, Input, Output, gap;
double PonMKp = 0.25, PonMKi = 0.50 , PonMKd = 0.05, PonEKp = 0.30, PonEKi = 0.25, PonEKd = 0.05;
double KpInput = PonMKp;
double KiInput = PonMKi;
double KdInput = PonMKd;
PID myPID(&Input,&Output,&Setpoint,KpInput,KiInput,KdInput,P_ON_M,REVERSE);
#define MOVECURSOR 1  // constants for indicating whether cursor should be redrawn
#define MOVELIST 2  // constants for indicating whether cursor should be redrawn
byte totalRows = 6;  // total rows of LCD
byte totalCols = 1;  // total columns of LCD


TinyGPSPlus gps;                                //required for TinyGPSplus Library
DigoleSerialDisp mydisp(9, 8, 10);              //Pin Config SPI | 9: data | 8:clock | 10: SS | you can assign 255 to SS, and hard ground SS pin on module
boolean led = false;                            //LED on | off
float voltage = 0, volt1, volt2;                //voltage value
byte cylCoeff = 6;                               //Number of Cylinders
boolean gpsMode = true;                         //GPS mode (true) or RPM mode (false)
byte mode = 0;                                   //Calculations mode 0=off 1=RPM 2=MPH/KPH


//*****PIN CONFIGURATION******
//0 and 1 = GPS
const byte PinCLK = 3;             //encoder second pin
const byte PinDT = 2;              // Used for reading DT signal of encoder
const byte PinSW = 4;              //encoder button
const byte servoPin = 5;           //servo PWM connected here
const byte rpmPin = 44;            //Engine RPM input connected here
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
int TargetSpeedInt, Target100 = 500;
int speed100, pos100;
int gpsDegree;//,gpsHDOP,gpsSats,gpsLat,gpsLng,gpsAlt;
int gpsSignalCounter = 0;
int EKp100,EKi100,EKd100,Kp100,Ki100,Kd100;
char gpsCourse[9];
boolean mph = true;
unsigned long delayCheck = 0;
unsigned long throttleCheck = 0;
unsigned long tempuratureCheck = 0;
unsigned long gpsCourseCheck = 0;
int targetSpeedWhole = TargetSpeedInt / 10;
int targetSpeedDecimal = TargetSpeedInt % 10;
int Contrast;
int menuItem = 1;
//*************RPM INITIALIZATION*************
volatile unsigned long duration = 0; // accumulates pulse width
volatile unsigned long pulsecount = 0; //incremented by interrupt
volatile unsigned long previousMicros = 0;
//int targetRPM = 3000;
int CalcMode = 0;
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
int hours, minutes, seconds, hourOffset = 12;
boolean speedMode = false;

//*************FUNCTION INITIALIZATION*************
int   readRpm();                              //read RPM and reset counters
void  rpmIntHandler();                        //called by interupt, increments counters
static void smartDelay(unsigned long ms);     //Reads Data from GPS device
void  isr();                                  //encoder - Interrupt service routine is executed when a HIGH to LOW transition is detected on CLK
void  PIDCalculations();                      //Calulate Speed Adjustments based on MPH or RPM depending on mode
void  MainDisplay();                          //Runs the Main LCD screen
void  Menu();                                 //runs the USER menu options
int read_encoder();                          //Reads knob up/down/press/longpress for menu cases
void updateTempuratureVoltRead();

//************************************************************************************************|
//*********** PERFORM STARTUP TASKS | DISPLAY STARTUP | GPS CONFIGURATION | READ MEMORY **********|
//************************************************************************************************|
void setup()
{
  //for debug---------
  Serial.begin(9600);
  delay (100);
  //------------------
  Serial1.begin(9600);   //GPS device
  delay (3000);
  mydisp.begin();
  mydisp.clearScreen();  delay(100);
  mydisp.setFont(30);
  mydisp.setPrintPos(0, 1);
  mydisp.print("   ");  delay(100);  mydisp.print("STEADYPASS");
  mydisp.setPrintPos(0, 2);
  delay(1000);  
  mydisp.print(" Version: ");
  mydisp.print(CurrentVersion);
  delay(1000);
  mydisp.setPrintPos(0, 3);
  mydisp.print(" DRAIN PLUG IN? ");
  delay(3000);
  mydisp.clearScreen();
  delay(500);
  if (EEPROM.read(200) != 2)
  {
    mydisp.setFont(30);
    mydisp.setPrintPos(0, 3);
    mydisp.print(" INITIALIZE MEM  ");
    delay(2000);
    EEPROM.write(11, 6);      //cylCoeff
    EEPROM.write(18, 1);      //Speed Units
    EEPROM.write(19, 12);     //Time Offset
    EEPROM.write(20, 195);    //MIN Throttle
    EEPROM.write(21, 95);     //MAX Throttle
    EEPROM.write(29, 1);      //Temp Units
    EEPROM.write(30, 10);     //Startup Target
    EEPROM.write(31, 1);      //Startup Target
    //EEPROM.write(32, 0);
    EEPROM.write(33, 30);     //Contrast
    EEPROM.write(200, 2);     //Preferences Reset Bit
    mydisp.clearScreen();
    delay(500);
  }
  if (EEPROM.read(201) != 3)
  {
    mydisp.setFont(30);
    mydisp.setPrintPos(0, 3);
    mydisp.print("UPDATING TUNING");
    delay(5000);
    EEPROM.write(12, 0); //KP whole
    EEPROM.write(13, 50); //KP dec
    EEPROM.write(14, 0); //KI whole
    EEPROM.write(15, 25); //KI dec
    EEPROM.write(16, 0); //KD whole
    EEPROM.write(17, 5); //KD dec
    EEPROM.write(50, 0); //KP whole
    EEPROM.write(51, 30); //KP dec
    EEPROM.write(52, 0); //KI whole
    EEPROM.write(53, 25); //KI dec
    EEPROM.write(54, 0); //KD whole
    EEPROM.write(55, 5); //KD dec
    EEPROM.write(201, 3);//tuning Update Bit
    mydisp.clearScreen();
    delay(500);
  }
  mydisp.setFont(30);
  mydisp.setPrintPos(0, 1);
  mydisp.print("ACQUIRING SIGNAL");
  mydisp.setPrintPos(0, 3);
  mydisp.print("  PLEASE WAIT ");
  delay(1000);
  pinMode(PinCLK, INPUT);                     //rotary encoder
  pinMode(PinDT, INPUT);                      //rotary encoder
  pinMode(PinSW, INPUT_PULLUP);               //rotary encoder button
  attachInterrupt (0, isr, RISING);           // interrupt 0 pin 3 for encoder
  //pinMode(7,INPUT);                         //RPM pin, high when no pulse.
  attachInterrupt(4, rpmIntHandler, CHANGE);  //  RPM - pin 7 is grounded on pulse
  myservo.attach(servoPin);                   // attaches the servo on pin  to the servo object
  //PID Variables//
  Input = speed100;
  Setpoint = Target100;
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
  Target100 = readWord(30);
  Contrast = EEPROM.read(33);

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
  delay (100);
  Serial1.println("$PMTK220,200*2C");  //Set GPS to 5hz
  delay (100);
}

//**RUN MAIN**|
void loop ()
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
  if (tempuratureCheck < millis()) {
    tempuratureCheck = millis() + 5000;
    updateTempuratureVoltRead();
  }

  //GPS COURSE CHECK
  if (gpsCourseCheck < millis()) {
    gpsCourseCheck = millis() + 1000;
    if (gpsDegree  > 337 && gpsDegree <= 22) {
      gpsCourse[1] = 'N';
      gpsCourse[2] = ' ';
    }
    if (gpsDegree  > 22  && gpsDegree <= 67) {
      gpsCourse[1] = 'N';
      gpsCourse[2] = 'E';
    }
    if (gpsDegree  > 67  && gpsDegree <= 112) {
      gpsCourse[1] = 'E';
      gpsCourse[2] = ' ';
    }
    if (gpsDegree  > 112 && gpsDegree <= 157) {
      gpsCourse[1] = 'S';
      gpsCourse[2] = 'E';
    }
    if (gpsDegree  > 157 && gpsDegree <= 202) {
      gpsCourse[1] = 'S';
      gpsCourse[2] = ' ';
    }
    if (gpsDegree  > 202 && gpsDegree <= 247) {
      gpsCourse[1] = 'S';
      gpsCourse[2] = 'W';
    }
    if (gpsDegree  > 247 && gpsDegree <= 292) {
      gpsCourse[1] = 'W';
      gpsCourse[2] = ' ';
    }
    if (gpsDegree  > 292 && gpsDegree <= 337) {
      gpsCourse[1] = 'N';
      gpsCourse[2] = 'W';
    }
  }

  //RUN CURRENT CALCULATION MODE//
  if (mode == 0) {
    pos = minServo;
  }
  else {
    speedMode = true;
    PIDCalculations();
  }
  myservo.writeMicroseconds(pos);

  //DISPLAY MAIN OUTPUT section
  TargetSpeedInt = Target100 / 10;
  speedValue = speed100 / 10;
  if (!mph) speedValue *= 1.61;

  if (mainDisplay) MainDisplay();  //DISPLAY MAIN OUTPUT

  if (TurnDetected) {
    if (mode == 1) Target100 += -10 * encoder;
    if (Target100 < 0) Target100 = 0;
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
      if (mode > 1) mode = 0;
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
   DESCTRIPTION: Reads Data from GPS device, Loops whiles data is transfering over Serial, ends loop once data is read.
*/
static void smartDelay(unsigned long ms)
{
  do
  {
    while (Serial1.available())
    {
      gps.encode(Serial1.read());
    }
  }
  while ( (!gps.speed.isUpdated())  );
}
//End Function

//*******************************************************************
/* FUNCTION: readRPM()
   INPUT: duration | pulsecount
   RETURN:freqq
   DESCTRIPTION: Takes INPUTs and runs an equation to turn it into an RPM reading
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
   DESCTRIPTION: it's a interrupt handler for RPMs, need better knowledge for better description
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
   RETURN: incriments encoder up or down
   DESCTRIPTION: encoder - Interrupt service routine is executed when a HIGH to LOW transition is detected on CLK
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
   DESCTRIPTION: Runs an equation to calculate Change in speed based off MPH | needs better description later
*/
void PIDCalculations()
{
  gap = (Setpoint - Input);
//  Serial.print(gap);
  //PID INPUTS//
  
//  if (speedMode) {
    if (gap >= 500){
      if (!calcPonMMode){
        calcPonMMode = true;
        KpInput = PonMKp;
        KiInput = PonMKi;
        KdInput = PonMKd;
        CalcMode = P_ON_M;
      }
    }
    else if(gap <=0){
      if (calcPonMMode){
        calcPonMMode = false;
        KpInput = PonEKp;
        KiInput = PonEKi;
        KdInput = PonEKd;
        CalcMode = P_ON_E;
      }
    }
    myPID.SetTunings(KpInput,KiInput,KdInput,CalcMode);
    Input = speed100;
    Setpoint = Target100;
    myPID.Compute();
    pos = Output;
//  Serial.print(Output); Serial.print("|||");
//  Serial.print(myPID.GetMode()); Serial.print("|||---");
}
//End Function

//*******************************************************************
/* FUNCTION:  MainDisplay()
   DESCTRIPTION: This displays everything that you see
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
    TargetSpeedInt = Target100 * 1.61;
    TargetSpeedInt = TargetSpeedInt / 10;
  }
  else {
    TargetSpeedInt = Target100 / 10;
  }
  targetSpeedWhole = TargetSpeedInt / 10;
  targetSpeedDecimal = TargetSpeedInt % 10;
  int c = targetSpeedDecimal + targetSpeedWhole;

  //Time//
  mydisp.setFont(10);
  mydisp.setPrintPos(0, 1);
  mydisp.print(hours); mydisp.print(":"); if (minutes < 10) mydisp.print("0"); mydisp.print(minutes); mydisp.print(":"); if (seconds < 10) mydisp.print("0"); mydisp.print(seconds); mydisp.print("  ");

  //print word "GPS"//
  mydisp.setFont(10);
  mydisp.setPrintPos(18, 3);
  mydisp.setTextPosOffset(2, -2);
  mydisp.print("GPS");

  //Print word "MPH" or "KPH"//
  mydisp.setPrintPos(18, 4);
  mydisp.setTextPosOffset(2, -3);
  if (mph) mydisp.print("MPH"); else mydisp.print("KPH");

  //Print word "POWER"//
  mydisp.setPrintPos(0, 0);
  mydisp.print("P:");
  mydisp.print((int)(throttlePer)); mydisp.print("");
  if(calcPonMMode){
    mydisp.print("M  ");
  } 
  else {
    mydisp.print("E  ");
  }

  //Printtarget MPH or KPH depending on mode selection
  mydisp.setFont(18);
  mydisp.setPrintPos(6, 0);
  if (mode == 0) mydisp.print("   OFF  ");
  if (mode == 1) {
    if (c < 0) {
      mydisp.print (" -");
    }
    else if (targetSpeedWhole < 10) {
      mydisp.print ("  ");
    }
    else {
      mydisp.print (" ");
    }
    if (targetSpeedWhole < 0) {
      targetSpeedWhole = -targetSpeedWhole;
    }
    mydisp.print (targetSpeedWhole); mydisp.print (".");
    if (targetSpeedDecimal < 0) {
      targetSpeedDecimal = -targetSpeedDecimal;
    }
    mydisp.print (targetSpeedDecimal);
    if (mph) mydisp.print("MPH "); else mydisp.print("KPH ");
  }

  //Prints Voltage
  mydisp.setFont(10);
  mydisp.setPrintPos(0, 2);
  mydisp.print("V ");  mydisp.print(voltage); mydisp.print("  ");

  //print GPS directional
  mydisp.setFont(10);
  mydisp.setPrintPos(0, 3);
  mydisp.print("DIR "); mydisp.print(gpsCourse[1]); mydisp.print(gpsCourse[2]);


  // Prints Water Temp and Air Temp
  mydisp.setPrintPos(0, 4);
  mydisp.print("WTR "); mydisp.print(wtrTemp); if (celsius)  mydisp.print("c  "); else mydisp.print("F  ");
  mydisp.setPrintPos(0, 5);
  mydisp.print("AIR "); mydisp.print(airTemp); if (celsius)  mydisp.print("c  "); else mydisp.print("F  ");

  //Prints current RPM reading
  mydisp.setPrintPos(0, 6);
  mydisp.print("RPM "); mydisp.print(readRpm()); mydisp.print("    ");

  //Prints the Main Speed value On Display
  mydisp.setFont(203);
  mydisp.setPrintPos(0, 0);
  mydisp.setTextPosOffset(46, 22);
  if (speedValue < 100) mydisp.print('0');
  mydisp.print(speedValue / 10);
  mydisp.setFont(201);
  mydisp.setPrintPos(0, 0);
  mydisp.setTextPosOffset(111, 43);
  mydisp.print(speedValue % 10);
  firstMainDispLoop = false;
}
//End Function
//-------------------------------------------------------------------


//*******************************************************************
/* FUNCTION: Menu()
   DESCTRIPTION: Displays the main settings menu
*/



int read_encoder() {
#define btnUp         1
#define btnDown       2
#define btnPress      4
#define btnLongPress  8
#define btnNull       16

  if (TurnDetected) {
    if (encoder > 0) {
      encoder = 0;
      TurnDetected = false;
      return btnUp;
    }
    else if (encoder < 0) {
      encoder = 0;
      TurnDetected = false;
      return btnDown;
    }
    else {
      TurnDetected = false;
      return btnNull;
    }
  }
  else {
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
      mydisp.setFont(20);
      mydisp.setPrintPos(0, 0);
      mydisp.print("BACK");
      while (digitalRead(PinSW) == LOW) {};
      mydisp.clearScreen();
      return btnLongPress;
    }
  }
}

void Menu(){
  //Reset selectors to false
	selectPonEKp = false;
	selectPonEKi = false;
	selectPonEKd = false;
	selectPonMKp = false;
	selectPonMKi = false;
	selectPonMKd = false;
	selectMaxServo = false;
	selectMinServo = false;
	selectStartSpeed = false;
	selectContrastMenu = false;
	selectCylinderMenu = false;
	selectClockMenu = false;
	selectTempMenu = false;
	selectMeasurementMenu = false;
	selectTestServo = false;
	menuItem = 1;
	mydisp.setFont(10); 
  do {
    stillSelecting = true;
    mydisp.setPrintPos(0, 0); 
    if (menuItem == 1) mydisp.print(">>");
    mydisp.print("Tuning  ");
    delay(20);
     
    mydisp.setPrintPos(0, 1); 
    if (menuItem == 2) mydisp.print(">>");
    mydisp.print("Servo  ");
    delay(20);
  
    mydisp.setPrintPos(0, 2); 
    if (menuItem == 3) mydisp.print(">>");
    mydisp.print("Preferences  ");
    delay(20); 
    
        switch (read_encoder())
      {
        case 1:  // ENCODER UP
           menuItem--;
           if(menuItem < 1) menuItem = 1;
          break;
        case 2:    //ENCODER DOWN
           menuItem++;
           if(menuItem > 3) menuItem = 3;
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
          
          break;
      }
  }
  while (stillSelecting == true);
}

  
void tuningMenu(){
  menuItem = 1;
  
do{
  mydisp.setFont(10); 
  mydisp.setPrintPos(0, 0); 
  if (menuItem == 1) mydisp.print(">>");
  mydisp.print("Target P:"); if (selectPonEKp) mydisp.print("("); mydisp.print(PonEKp); if (selectPonEKp) mydisp.print(")"); mydisp.print("  ");
  delay(25); 
  
  mydisp.setPrintPos(0, 1);
  if (menuItem == 2) mydisp.print(">>");
  mydisp.print("Target I:"); if (selectPonEKi) mydisp.print("("); mydisp.print(PonEKi);  if (selectPonEKi) mydisp.print(")"); mydisp.print("  ");
  delay(25);

  mydisp.setPrintPos(0, 2);
   if (menuItem == 3) mydisp.print(">>");
   mydisp.print("Target D:"); if (selectPonEKd) mydisp.print("("); mydisp.print(PonEKd);  if (selectPonEKd) mydisp.print(")"); mydisp.print("  ");
    delay(25);
 
   mydisp.setPrintPos(0, 4);
   if (menuItem == 4) mydisp.print(">>");
   mydisp.print("Accel P:"); if (selectPonMKp) mydisp.print("("); mydisp.print(PonMKp); if (selectPonMKp) mydisp.print(")"); mydisp.print("  ");
   delay(25);  

  mydisp.setPrintPos(0, 5);
  if (menuItem == 5) mydisp.print(">>");
  mydisp.print("Accel I:"); if (selectPonMKi) mydisp.print("("); mydisp.print(PonMKi); if (selectPonMKi) mydisp.print(")"); mydisp.print("  ");
  delay(25); 

  mydisp.setPrintPos(0, 6); 
  if (menuItem == 6) mydisp.print(">>");
  mydisp.print("Accel D:"); if (selectPonMKd) mydisp.print("("); mydisp.print(PonMKd); if (selectPonMKd) mydisp.print(")");mydisp.print("  ");
  delay(25);


  switch (read_encoder())
      {
        stillSelecting = true;  
        case 1:  // ENCODER UP

          if (selectPonEKp){
            PonEKp = PonEKp * 100;
            if (PonEKp >= 100) {
              PonEKp = 100;
            }
            else {
              PonEKp += 1;
            }
            EKp100 = PonEKp;
            PonEKp /= 100;
          }
          
          else if (selectPonEKi){
            PonEKi = PonEKi * 100;
            if (PonEKi >= 100) {
              PonEKi = 100;
            }
            else {
              PonEKi += 1;
            }
            EKi100 = PonEKi;
            PonEKi /= 100;
          }
          
          else if (selectPonEKd){
            PonEKd = PonEKd * 100;
            if (PonEKd >= 100) {
              PonEKd = 100;
            }
            else {
              PonEKd += 1;
            }
            EKd100 = PonEKd;
            PonEKd /= 100;
          }
          
          else if (selectPonMKp){
            PonMKp = PonMKp * 100;
            if (PonMKp >= 100) {
              PonMKp = 100;
            }
            else {
              PonMKp += 1;
            }
            Kp100 = PonMKp;
            PonMKp /= 100;
          }
          
          else if (selectPonMKi){
            PonMKi = PonMKi * 100;
            if (PonMKi >= 100) {
              PonMKi = 100;
            }
            else {
              PonMKi += 1;
            }
            Ki100 = PonMKi;
            PonMKi /= 100;
          }
          
          else if (selectPonMKd){
            PonMKd = PonMKd * 100;
            if (PonMKd >= 100) {
              PonMKd = 100;
            }
            else {
              PonMKd += 1;
            }
            Kd100 = PonMKd;
            PonMKd /= 100;
          }
          else{
            menuItem--;
            if(menuItem < 1) menuItem = 1;
          }
          break;

          
        case 2:    //ENCODER DOWN
        if (selectPonEKp){
          PonEKp = PonEKp * 100;
          if (PonEKp <= 0) {
            PonEKp = 0;
          }
          else {
            PonEKp -= 1;
          }
          EKp100 = PonEKp;
          PonEKp /= 100;
        }
        
        else if (selectPonEKi){
          PonEKi = PonEKi * 100;
          if (PonEKi <= 0) {
            PonEKi = 0;
          }
          else {
            PonEKi -= 1;
          }
          EKi100 = PonEKi;
          PonEKi /= 100;
        }
        
        else if (selectPonEKd){
          PonEKd = PonEKd * 100;
          if (PonEKd <= 0) {
            PonEKd = 0;
          }
          else {
            PonEKd -= 1;
          }
          EKd100 = PonEKd;
          PonEKd /= 100;
        }
        
        else if (selectPonMKp){
          PonMKp = PonMKp * 100;
          if (PonMKp <= 0) {
            PonMKp = 0;
          }
          else {
            PonMKp -= 1;
          }
          Kp100 = PonMKp;
          PonMKp /= 100;
        }

        else if (selectPonMKi){
          PonMKi = PonMKi * 100;
          if (PonMKi <= 0) {
            PonMKi = 0;
          }
          else {
            PonMKi -= 1;
          }
          Ki100 = PonMKi;
          PonMKi /= 100;
        }

        else if (selectPonMKd){
          PonMKd = PonMKd * 100;
          if (PonMKd <= 0) {
            PonMKd = 0;
          }
          else {
            PonMKd -= 1;
          }
          Kd100 = PonMKd;
          PonMKd /= 100;
        }
        
        else{
           menuItem++;
           if(menuItem > 6) menuItem = 6;
        }
          break;
        case 4:  // ENCODER BUTTON SHORT PRESS
          if (menuItem == 1) selectPonEKp = !selectPonEKp;
          if (menuItem == 2) selectPonEKi = !selectPonEKi;
          if (menuItem == 3) selectPonEKd = !selectPonEKd;
          if (menuItem == 4) selectPonMKp = !selectPonMKp;
          if (menuItem == 5) selectPonMKi = !selectPonMKi;
          if (menuItem == 6) selectPonMKd = !selectPonMKd;
          break;
        case 8:  // ENCODER BUTTON LONG PRESS
          Menu();
          break;
        case 16:  // ENCODER BUTTON NULL
          
          break;
      }
  }
  while (stillSelecting == true);
}

void servoMenu(){
  menuItem = 1;
  do{
    mydisp.setFont(10);
    mydisp.setPrintPos(0, 0); 
    if (menuItem == 1) mydisp.print(">>");
    mydisp.print("Min Servo:"); if (selectMinServo) mydisp.print("("); mydisp.print(minServo); if (selectMinServo) mydisp.print(")"); mydisp.print("  "); 
    
    mydisp.setPrintPos(0, 1);
    if (menuItem == 2) mydisp.print(">>");
    mydisp.print("Max Servo:"); if (selectMaxServo) mydisp.print("("); mydisp.print(maxServo); if (selectMaxServo) mydisp.print(")"); mydisp.print("  ");
  
    mydisp.setPrintPos(0, 2);
     if (menuItem == 3) mydisp.print(">>");
     mydisp.print("Test:");
     if (selectTestServo) mydisp.print("(");
     if (selectTestServo)mydisp.print("Testing"); else mydisp.print("Off");
     if (selectTestServo) mydisp.print(")");
     mydisp.print("    ");
     mydisp.setPrintPos(0, 5);
     mydisp.print("Servo Position: "); mydisp.print(pos); mydisp.print("  ");
     
     myservo.writeMicroseconds(pos);
     delay(100);
     
  switch (read_encoder())
      {
        stillSelecting = true;  
        case 1:  // ENCODER UP
          if(selectMaxServo){
            if (maxServo >= 1950) {
              maxServo = 1950;
            }
            else {
              maxServo += 10;
            }
            pos = maxServo;
          }
          
          else if(selectMinServo){
            if (minServo >= maxServo) {
              minServo = maxServo - 10;
            }
            else {
              minServo += 10;
            }
            pos = minServo;
          }
          
          else{
            menuItem--;
            if(menuItem < 1) menuItem = 1;
          }
          break;
          
        case 2:    //ENCODER DOWN
          if(selectMaxServo){
            if (maxServo <= minServo) {
              maxServo = minServo + 10;
            }
            else {
              maxServo -= 10;
            }
            pos = maxServo;
          }
          
          else if(selectMinServo){
            if (minServo <= 950) {
              minServo = 950;
            }
            else {
              minServo -= 10;
            }
            pos = minServo;
          }
          
          else{
            menuItem++;
            if(menuItem > 3) menuItem = 3;
          }
          break;

        case 4:  // ENCODER BUTTON SHORT PRESS
          if (menuItem == 1) selectMinServo = !selectMinServo;
          if (menuItem == 2) selectMaxServo = !selectMaxServo;
          if (menuItem == 3) selectTestServo = !selectTestServo;

          break;
        case 8:  // ENCODER BUTTON LONG PRESS
          Menu();
          break;
        case 16:  // ENCODER BUTTON NULL
            if(selectTestServo){
              if (pos <= minServo) S = 1;
              if (pos >= maxServo) S = -1;
              pos += 10 * S;
            }
          break;
      }
  }
  while (stillSelecting == true);
}
 

void preferencesMenu(){
   menuItem = 1;
   
   gps.encode(Serial1.read());
       hours = gps.time.hour() + hourOffset - 12;
       int hour = hours;
       minutes = gps.time.minute();
       seconds = gps.time.second();
       
   do{
      mydisp.setFont(10);
      mydisp.setPrintPos(0, 0); 
      if (menuItem == 1) mydisp.print(">>");
      mydisp.print("StartSpd:"); if (selectStartSpeed) mydisp.print("("); printTargetSpeed(); if (selectStartSpeed) mydisp.print(")"); mydisp.print("  ");
        delay(20);
        
      mydisp.setPrintPos(0, 1);
      if (menuItem == 2) mydisp.print(">>");
      mydisp.print("Engine Cyl:"); if (selectCylinderMenu) mydisp.print("("); mydisp.print(cylCoeff); if (selectCylinderMenu) mydisp.print(")"); mydisp.print("  ");
      delay(20);
      
      mydisp.setPrintPos(0, 2);
       if (menuItem == 3) mydisp.print(">>");
       mydisp.print("Clock Offset:");if (selectClockMenu) mydisp.print("("); mydisp.print(hourOffset - 12); if (selectClockMenu) mydisp.print(")"); mydisp.print("   "); // mydisp.print(hour); mydisp.print("  ");
        delay(20);
        
     mydisp.setPrintPos(0, 3);
       if (menuItem == 4) mydisp.print(">>");
       mydisp.print("Unit Speed:"); 
       if (selectMeasurementMenu) mydisp.print("("); 
       if (mph) mydisp.print("MPH"); else mydisp.print("KPH");
       if (selectMeasurementMenu) mydisp.print(")");  
        mydisp.print("  ");
       delay(20);
         
     mydisp.setPrintPos(0, 4);
       if (menuItem == 5) mydisp.print(">>");
       mydisp.print("Unit Temp:");
       if (selectTempMenu) mydisp.print("("); 
       if (celsius) mydisp.print("C"); else mydisp.print("F"); 
       if (selectTempMenu) mydisp.print(")"); 
       mydisp.print("  ");
       delay(20);
        
    mydisp.setPrintPos(0, 5);
       if (menuItem == 6) mydisp.print(">>");
       mydisp.print("Contrast:"); if (selectContrastMenu) mydisp.print("(");  mydisp.print(Contrast); if (selectContrastMenu) mydisp.print(")"); mydisp.print("  ");
       delay(20);
       

    switch (read_encoder())
      {
        stillSelecting = true;  
        case 1:  // ENCODER UP
          if(selectStartSpeed){
            if (Target100 >= 5000) {
               Target100 = 5000;
             }
             else {
               Target100 += 10;
             }
          }
          
          else if(selectCylinderMenu){
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

          else if(selectClockMenu){
             if (hourOffset < 24) {
               hourOffset += 1;
             }
             else {
               hourOffset = 24;
             }
             hour = hour + hourOffset - 12;
             if (hour < 0) hour += 24;
             if (hour > 24) hour -= 24;
          }

          else if(selectMeasurementMenu){
            mph = !mph;
          }

          else if(selectTempMenu){
            celsius = !celsius;
          }

          else if(selectContrastMenu){
             if (Contrast >= 40) {
               Contrast = 40;
             }
             else {
               Contrast += 1;
             }
             mydisp.setContrast(Contrast);
          }
          else{
            menuItem--;
            if(menuItem < 1) menuItem = 1;
          }
          break;
          
        case 2:    //ENCODER DOWN
          if(selectStartSpeed){
            if (Target100 <= 500) {
              Target100 = 500;
            }
            else {
              Target100 -= 10;
            }
          }
          
          else if(selectCylinderMenu){
            if (cylCoeff > 4) {
              cylCoeff = cylCoeff - 2;
            }
            else {
              cylCoeff = 1;
            }
          }

          else if(selectClockMenu){
             if (hourOffset > 0) {
               hourOffset -= 1;
             }
             else {
               hourOffset = 0;
             }
             hours = gps.time.hour() + hourOffset - 12;
             if (hours < 0) hours += 24;
             if (hours > 24) hours -= 24;
          }

          else if(selectMeasurementMenu){
            mph = !mph;
          }

          else if(selectTempMenu){
            celsius = !celsius;
          }

          else if(selectContrastMenu){
             if (Contrast <= 20) {
               Contrast = 20;
             }
             else {
               Contrast -= 1;
             }
            mydisp.setContrast(Contrast);
          }
          else{
            menuItem++;
            if(menuItem > 6) menuItem = 6;
          }
          break;

        case 4:  // ENCODER BUTTON SHORT PRESS
          if (menuItem == 1) selectStartSpeed = !selectStartSpeed;
          if (menuItem == 2) selectCylinderMenu = !selectCylinderMenu;
          if (menuItem == 3) selectClockMenu = !selectClockMenu;
          if (menuItem == 4) selectMeasurementMenu = !selectMeasurementMenu;
          if (menuItem == 5) selectTempMenu = !selectTempMenu;
          if (menuItem == 6) selectContrastMenu = !selectContrastMenu;

          break;
        case 8:  // ENCODER BUTTON LONG PRESS
          Menu();
          break;
        case 16:  // ENCODER BUTTON NULL
          break;
      }
  } 
  while (stillSelecting == true);
}


void returnToMainDisp()
{
  stillSelecting = false;
  buttonTimes = 0;
  firstMainDispLoop = true;
  loop ();
    writeWord(12, EKp100);
    writeWord(14, EKi100);
    writeWord(16, EKd100);
    writeWord(50, Kp100);
    writeWord(52, Ki100);
    writeWord(54, Kd100);
    
    EEPROM.write(11, cylCoeff);
    if (mph) EEPROM.write(18, 1); else EEPROM.write(18, 0);
    EEPROM.write(19, hourOffset);
    EEPROM.write(20, maxServo / 10);
    EEPROM.write(21, minServo / 10);
    if (celsius) EEPROM.write(29, 1); else EEPROM.write(29, 0);
    writeWord(30, Target100);
    EEPROM.write(33, Contrast);
}


void writeWord(unsigned address, unsigned value)
{
  EEPROM.write(address, highByte(value));
  EEPROM.write(address + 1, lowByte(value));
}
//End Function
//-------------------------------------------------------------------


//*******************************************************************
/* FUNCTION: readWord
   DESCTRIPTION: used for reading large value from EEPROM
*/
unsigned readWord(unsigned address)
{
  return word(EEPROM.read(address), EEPROM.read(address + 1));
}
//End Function
//-------------------------------------------------------------------


//*******************************************************************
/* FUNCTION: updateTempuratureVoltRead
   DESCTRIPTION: updates the tempurature and voltage readings for use by the main display
*/
void updateTempuratureVoltRead()
{
  //READS VOLTAGE
  volt1 = voltage;
  for (i = 1; i < 3; i++)  voltage = analogRead(0);
  voltage = (float)voltage / 35.2 ;

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

void printTargetSpeed()
{
  if (!mph) {
    TargetSpeedInt = Target100 * 1.61;
    TargetSpeedInt = TargetSpeedInt / 10;
  }
  else {
    TargetSpeedInt = Target100 / 10;
  }
  targetSpeedWhole = TargetSpeedInt / 10;
  targetSpeedDecimal = TargetSpeedInt % 10;
  int c = targetSpeedDecimal + targetSpeedWhole;
  if (c < 0) mydisp.print ("-");
  else if (targetSpeedWhole < 10) mydisp.print (" ");
  else mydisp.print ("");
  if (targetSpeedWhole < 0) targetSpeedWhole = -targetSpeedWhole;
  mydisp.print (targetSpeedWhole);
  mydisp.print (".");
  if (targetSpeedDecimal < 0)targetSpeedDecimal = -targetSpeedDecimal;
  mydisp.print (targetSpeedDecimal);
  if (mph) mydisp.print("MPH "); else mydisp.print("KPH ");
}
