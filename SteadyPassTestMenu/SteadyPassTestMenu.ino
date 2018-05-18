/*
  _________ __                     .___       __________                        ____   ________
 /   _____//  |_  ____ _____     __| _/__.__. \______   \_____    ______ ______ \   \ /   /_   |
 \_____  \\   __\/ __ \\__  \   / __ <   |  |  |     ___/\__  \  /  ___//  ___/  \       / |   |
 /        \|  | \  ___/ / __ \_/ /_/ |\___  |  |    |     / __ \_\___ \ \___ \    \     /  |   |
/_______  /|__|  \___  >____  /\____ |/ ____|  |____|    /____  /____  >____  >    \___/   |___|
================================================================================================
************************************************************************************************
  ---|FILE: SteadyPassV1.ino
  ---|Created By: Dimitry
  ---|Description: This is the code used to make everything work
  ---|Change History
  ---|  <16.53 - Dimitry - Allversion Prior to 17.00 have been soley created by Dimitry
  ---|  17.00 - Sam James - Renamed File from "Limiter_16_53.ino", Added Title and Change History, Added Function comment and Descriptions, Minor code changes made that will not effect the main code.
  ---|  17.01 - Sam James - Remade speed and RPM calc into one function.
*/

//*************LIBRARYS*************
#include <TinyGPS++.h>
#include <Servo.h>
#include <SPI.h>
#include <OneWire.h>
#include <EEPROM.h>
#define _Digole_Serial_SPI_
#include <DigoleSerial.h>
#include <PID_v1.h>
//----------------------------------

//*************INITIALIZING DEFINITIONS*************
double Setpoint, Input, Output;

//Default PROPORTION ON MEASURE MODE//
//boolean adaptiveTuningMode = false;
PID myPID(&Input, &Output, &Setpoint,2,5,1,P_ON_M, DIRECT);
//------------------------------------------------------------

//ADAPTIVE TUNING MODE//
// boolean adaptiveTuningMode = true;
// double aggKp = 30, aggKi = 0.2, aggKd = 25; //NEED TO MAKE MENU OPTION FOR ADJUSTMENTS ON THE FLY
// double consKp = 15, consKi = 0.1, consKd = 12.5; //NEED TO MAKE MENU OPTION FOR ADJUSTMENTS ON THE FLY
// PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);
//--------------------------------------------------------------------
//TEST MENU
//----------___----__---___---___--___--___---__---___----__-
#define MOVECURSOR 1  // constants for indicating whether cursor should be redrawn
#define MOVELIST 2  // constants for indicating whether cursor should be redrawn
byte totalRows = 64;  // total rows of LCD
byte totalCols = 128;  // total columns of LCD
unsigned long timeoutTime = 0;  // this is set and compared to millis to see when the user last did something.
const int menuTimeout = 10000; // time to timeout in a menu when user doesn't do anything.
//----------___----__---___---___--___--___---__---___----__-


float fltCurrentVersion = 17.01;
TinyGPSPlus gps;                                //required for TinyGPSplus Library
DigoleSerialDisp mydisp(9, 8, 10);              //Pin Config SPI | 9: data | 8:clock | 10: SS | you can assign 255 to SS, and hard ground SS pin on module
unsigned long tachTime = 0, tachTimePrev = 0;   //keep track of tach time since power on
boolean debug = false;                          //Debug mode
boolean led = false;                            //LED on | off
float voltage = 0, volt1, volt2;                //voltage value
int cylCoeff = 4;                               //Number of Cylinders
boolean gpsMode = true;                         //GPS mode (true) or RPM mode (false)
int mode = 0;                                   //Calculations mode 0=off 1=RPM 2=MPH/KPH
int DisplayMode = 0;                            //display mode
//------------------------------------------------------------


//*************PIN CONFIGURATION*************
//0 and 1 = GPS  , int 2 and 3
const int PinCLK = 2;             //int 1 == pin 2 encoder second pin
const int PinDT = 3;              //int0 == pin3    // Used for reading DT signal of encoder
const int PinSW = 4;              //encoder button
const int servoPin = 5;           //servo PWM connected here
const int speedPin = 6;           //reserve    int4 == pin 7? speed sensor
const int rpmPin = 7;             //Engine currentRPM input connected here
//8 and 9 - LCD, defined above
//open - 10 - LCD SS?
const int tempWtrPin = 11;        //temperature water
const int tempAirPin = 12;        //temperature air
const int voltPin = A0;           //battery voltage divider
//OPEN A1, A2, A3 - relay out?
const int tempBoardPin = A4;      //temperature on control board
const int lightsPin = A5;         //A5;  // lights on voltage divider
//-------------------------------------------------------------------


//*************TEMP SENSOR INITIALIZATION*************
OneWire  ds(tempBoardPin);
OneWire  wtr(tempWtrPin);
OneWire  air(tempAirPin);
byte data[12];
byte addr[8];
byte present = 0;
int i;
int Temp, airTemp, wtrTemp;
long checkTemp = 0;
boolean celsius = true;
//---------------------END TEMP-----------------------


//*************SPEED INITIALIZATION*************
volatile unsigned long speedPulse = 0;
unsigned long speedPulsePrev = 0;
unsigned long speedPulseCurrent = 0;
unsigned long speedTime = 0;
unsigned long speedTimePrev = 0;
int speedValue = 0, speedWater = 0, speedGps = 0;
double speedGpsD = 0;
long speedCoeff = 2250;
int TargetSpeedInt, Target100 = 1250;
int goodSteps = 15, goodStepsTemp;
int speedBuffer[20];
int avSpeed100;
int SpeedCorrected = 0;
int PrevSpeed = 0;
int Speed100, delta100, pos100;  //int * 100 values
boolean mph = true;
unsigned long throttleCheck = 0;
//-----------------END SPEED--------------------


//*************RPM INITIALIZATION*************
long currentRPM = 0;
int rpmBuffer[20];
volatile unsigned long duration = 0; // accumulates pulse width
volatile unsigned long pulsecount = 0; //incremented by interrupt
volatile unsigned long previousMicros = 0;
int targetRPM = 3000;
//long periodRPM = 1500;
long checkRPM = 0;
long periodSpeed = 3000;
long checkSpeed = 0;
int avRPM;
//-----------------END RPM--------------------

//*************SERVO INITIALIZATION*************
static long virtualPosition = 0;
boolean Reverse = false;
Servo myservo;
int minServo = 1050;
int maxServo = 1950;
int pos = minServo;       //servo position
int S = 10;
//-----------------END SERVO--------------------


//*************ENCODER INITIALIZATION*************
volatile boolean TurnDetected = false;
volatile boolean up = false, prev_up = false;
unsigned long interruptTime = 0;
long prevtime = 0;
long threshold = 7;
int buttonTimes = 0;
int inMenuPress = 0;
int inMenuPressTarget = 7;
int buttonTarget = 7;
int buttonTimes2 = 0;
int menuItem = 1;
int idleCounter = 0, idleLimit = 2500;
boolean mainDisplay = true;
boolean modeOn = true;
volatile long encoder = 0;
int hours, minutes, seconds, hourOffset = 7;
boolean speedMode = true;

//-----------------END ENCODER--------------------


//*************FUNCTION INITIALIZATION*************
float readSpeed();                            //read GPS speed
int   readRpm();                              //read/calculate currentRPM and reset counters
void  rpmIntHandler();                        //called by interupt, increments counters
static void smartDelay(unsigned long ms);     //Reads Data from GPS device
void  isr();                                  //encoder - Interrupt service routine is executed when a HIGH to LOW transition is detected on CLK
void  PIDCalculations();                      //Calulate Speed Adjustments based on MPH or RPM depending on mode
void  MainDisplay();                          //Runs the Main LCD screen
void  Menu();                                 //runs the USER menu options
void  DebugOutput();                          //Outputs addition information by Serial output for diagnostics and Debuging
int read_encoder();                          //Reads knob up/down/press/longpress for menu cases
//-----------------END--------------------

//************************************************************************************************|
//*********** PERFORM STARTUP TASKS | DISPLAY STARTUP | GPS CONFIGURATION | READ MEMORY **********|
//************************************************************************************************|
void setup()
{
  if (debug)Serial.begin(9600);
  Serial1.begin(9600);   //GPS device
  delay (2000);
  //digole LCD init section with name and version
  mydisp.begin(); mydisp.clearScreen();  delay(100);   mydisp.print("  ................. "); delay(100); mydisp.clearScreen();  delay(100);   mydisp.print("."); delay(100);
  mydisp.print("   ");  delay(100);  mydisp.print("STEADYPASS");    delay(1000);
  //mydisp.setRot180(); //uncomment to rotate
  mydisp.setFont(10);
  mydisp.setPrintPos(0, 4);
  mydisp.print("     fw: ");
  mydisp.print(fltCurrentVersion);
  delay(7000);
  mydisp.clearScreen();
  mydisp.print("SATELLITE SEARCH..");
  pinMode(PinCLK, INPUT);                     //rotary encoder
  pinMode(PinDT, INPUT);                      //rotary encoder
  pinMode(PinSW, INPUT_PULLUP);               //rotary encoder button
  attachInterrupt (0, isr, RISING);           // interrupt 0 pin 3 for encoder
  //pinMode(7,INPUT);                         //RPM pin, high when no pulse.
  attachInterrupt(4, rpmIntHandler, CHANGE);  //  RPM - pin 7 is gronded on pulse
  myservo.attach(servoPin);                   // attaches the servo on pin  to the servo object
  myservo.writeMicroseconds(pos);


  //PID Variables//
  Input = Speed100;
  Setpoint = Target100;
  //Turn on PID//
  myPID.SetMode(AUTOMATIC);
  //----------------------


  //******************LOADING SAVED SETTINGS*******************
  if (EEPROM.read(11) != 255) cylCoeff = EEPROM.read(11);
//  if (EEPROM.read(12) != 255) aggKp = EEPROM.read(12);
//  if (EEPROM.read(13) != 255) aggKd = EEPROM.read(13);
//  if (EEPROM.read(14) != 255) aggKi = EEPROM.read(14);
//  if (EEPROM.read(22) != 255) consKp = EEPROM.read(22);
//  if (EEPROM.read(23) != 255) consKd = EEPROM.read(23);
//  if (EEPROM.read(24) != 255) consKi = EEPROM.read(24);
  if (EEPROM.read(15) != 255) hourOffset = 24 - EEPROM.read(15);
  if (EEPROM.read(16) != 255) maxServo = 10 * EEPROM.read(16);
  if (EEPROM.read(17) != 255) minServo = 10 * EEPROM.read(17);
  if (EEPROM.read(18) != 255) mph = EEPROM.read(18);
  if (EEPROM.read(29) != 255) celsius = EEPROM.read(29);
  if (EEPROM.read(30) != 255) Target100 = EEPROM.read(30) * 100;
  if (EEPROM.read(31) != 255) Reverse = EEPROM.read(31);
  //----------------LOADED VALUES COMPLETE-------------------

  //******************LOADING GPS CONFIGURATION*******************
  //Serial1.println("$PMTK314,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29");   //Sets GPS to GTV only
  Serial1.println("$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29");     //Sets GPS to RMC only
  Serial1.println("$PMTK220,200*2C");  //Set GPS to 5hz
  //----------------GPS CONFIGURATION COMPLETE-------------------
  if (debug) {
    Serial.println("SETUP COMPLETE");
  }
}

//================================================================================================|
//------------------------------------ STARTUP COMPLETE ------------------------------------------|
//================================================================================================|






//************************************************************************************************|
//*****************************************RUN MAIN***********************************************|
//************************************************************************************************|
void loop ()
{
  if (led)
  {
    digitalWrite(13, LOW);
    led = false;
  }
  else
  {
    digitalWrite(13, HIGH);
    led = true;
  }

  //
  smartDelay(1);
  if (gps.speed.isValid())
  {
    speedGpsD = gps.speed.mph();
  }
  else
  {
    //possible enhancment to switch to RPM calculation if GPS signal lost for extended time.
  }
  Speed100 = 100 * speedGpsD;
  if (debug) {
    Serial.print("Speed 100: ");
    Serial.println(Speed100);
  }

  currentRPM = readRpm();

  //Get Total Run time since powerup//
  if (currentRPM > 50)
  {
    tachTime += millis() - tachTimePrev;
    tachTimePrev = millis();
  }
  else
  {
    tachTimePrev = millis();
  }

  //THROTTLE OVER LIMIT warning//
  if (pos == maxServo)
  {
    if (throttleCheck < millis())
    {
      throttleCheck = millis() + 2000;
      mydisp.setFont(18);
      mydisp.setPrintPos(0, 0);
      mydisp.print(" ! DECREASE !     ");
      mydisp.setPrintPos(0, 1);
      mydisp.print(" ! THROTTLE !     ");
      smartDelay(1);
      if (gps.speed.isValid())
      {
        speedGpsD = gps.speed.mph();
      }
      Speed100 = 100 * speedGpsD;
      mydisp.setPrintPos(0, 0);
      mydisp.print("                  ");
      mydisp.setPrintPos(0, 1);
      mydisp.print("                  ");
    }
  }


  //STORE LAST 10 GPS SPEEDS//
  for (int i = 10; i > 0; i--)
  {
    speedBuffer[i] = speedBuffer[i - 1];
    speedBuffer[0] = Speed100;
    avSpeed100 = 0;
  }
  //CALCULATE AVERAGE SPEED//
  for (int i = 5; i >= 0; i--)
  {
    avSpeed100 += speedBuffer[i];
    avSpeed100 = avSpeed100 / 6;
  }
  //SOTRE LAST 10 currentRPM VALUES
  for (int i = 10; i > 0; i--)
  {
    rpmBuffer[i] = rpmBuffer[i - 1];
    rpmBuffer[0] = currentRPM;
  }

  //RUN CURRENT CALCULATION MODE//
  if (mode == 0)
  {
    pos = minServo;
  }
  else
  {
    if (mode == 1)
    {
      speedMode = false;
    }
    else
    {
      speedMode = true;
    }
    PIDCalculations();
  }

  if (Reverse) myservo.writeMicroseconds( minServo + maxServo - pos);
  else  myservo.writeMicroseconds(pos);

  if (debug)  DebugOutput();


  //DISPLAY MAIN OUTPUT section
  TargetSpeedInt = Target100 / 10;
  speedValue = Speed100 / 10; //use speedValue for display with 1 decimal

  if (mainDisplay) MainDisplay();  //DISPLAY MAIN OUTPUT



  if (TurnDetected)  // do this only if rotation was detected - change target
  {
    //interruptTime = millis();
    if (mode == 2) Target100 += 10 * encoder;
    if (mode == 1) targetRPM += 10 * encoder;
    encoder = 0;
    TurnDetected = false;          // do NOT repeat IF loop until new rotation detected
  }


  if (digitalRead(PinSW) == LOW)   //menu enter
  {
    buttonTimes++;
  }
  else {
    if (buttonTimes > 0)
    { mode++;  if (mode > 2) mode = 0;
      if (!mainDisplay) idleCounter = 0; mainDisplay = true; mydisp.clearScreen();
    }
    buttonTimes = 0;
  }

  if (buttonTimes > buttonTarget)     {
    mydisp.clearScreen();
    mydisp.setFont(10);
    delay(500);
  }

  while (buttonTimes > buttonTarget)      Menu();     //menu will loop untill buttonTimes is set to 0


  //SCREEN SAVER ACTIVATION PARAMETERS
  if (currentRPM < 50) idleCounter++; else {
    idleCounter = 0;
  }

  if ((idleCounter > idleLimit) && mainDisplay)
  {
    mainDisplay = false;
    mydisp.clearScreen();
  }
  if (!mainDisplay)
  {
    mydisp.setFont(120);
    mydisp.setPrintPos(0, 0);
    mydisp.setTextPosOffset(20, 15);
    mydisp.print(hours);
    mydisp.print(":");
    if (minutes < 10)
    {
      mydisp.print("0");
    }
    mydisp.print(minutes);
    mydisp.print("  ");
    mydisp.setFont(6);
    mydisp.setPrintPos(0, 2);
    mydisp.setTextPosOffset(0, 0);
    mydisp.setFont(10);
    mydisp.setPrintPos(0, 0);
    mydisp.print("Tach today ");
    mydisp.print(tachTime / 60000);
    mydisp.println(" min");
    delay(20);
    mydisp.setPrintPos(0, 6);
    mydisp.print("UNT ");
    mydisp.print(Temp);
    if (celsius)
    {
      mydisp.print("c");
    }
    else
    {
      mydisp.print("F");
    }
    mydisp.print(" Batt ");
    mydisp.print(voltage);
    mydisp.print("v ");
    mydisp.setPrintPos(0, 5);
    mydisp.print("WTR ");
    mydisp.print(wtrTemp);
    if (celsius)
    {
      mydisp.print("c");
    }
    else
    {
      mydisp.print("F");
    }
    mydisp.print(" AIR ");
    mydisp.print(airTemp);
    if (celsius)
    {
      mydisp.print("c ");
    }
    else
    {
      mydisp.print("F ");
    }

    if (currentRPM > 500)
    {
      idleCounter = 0;
      mainDisplay = true;
      mydisp.clearScreen();
    }

  }
  //SCREEN SAVER END

  /*
    //tweak on the fly  - for tuning from PC
      if(Serial.available())
    {
     char b = Serial.read();
     Serial.flush();

      if((b=='w'))  Target100 += 100;
      if((b=='s'))  Target100 -= 100;
      if((b=='e'))  Kd += 1;
      if((b=='d'))  Kd -= 1;
      if((b=='p'))  Kp += 1;
      if((b==';'))  Kp -= 1;
      if((b=='o'))  Ko += 1;
      if((b=='l'))  Ko -= 1;
      if((b=='r'))  targetRPM += 10;
      if((b=='f'))  targetRPM -= 10;
      if((b=='g'))  {gpsMode =!gpsMode;  targetRPM = currentRPM;}

    }*/

  //  Serial.print("Tgt: ");Serial.print(Target100); Serial.print(" ");
  //  Serial.print("Sp: ");Serial.print(Speed100); Serial.print(" ");
  //  Serial.print("Sp_Cor: ");Serial.print(SpeedCorrected); Serial.print(" ");
  //  Serial.print("Srv: ");Serial.print(180-pos); Serial.print(" ");

  //  Serial.print("Kp: ");Serial.print(Kp);Serial.print(" ");
  //  Serial.print("Kd: ");Serial.print(Kd);Serial.print(" ");
  //  Serial.print("Ko: ");Serial.print(Ko);Serial.println();
}
//================================================================================================|
//-------------------------------------------MAIN END---------------------------------------------|
//================================================================================================|





//*******************************************************************
/* FUNCTION: smartDelay()
   INPUT: N/A
   RETURN: N/A
   DESCTRIPTION: Reads Data from GPS device, Loops whiles data is transfering over Serial, ends loop once data is read.
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
  }
  while ( !(gps.speed.isUpdated())  );                  //while (millis() - start < ms);
}
//End Function
//-------------------------------------------------------------------


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
  long freqq = 60 * 1e5 * _pulsecount / _duration / cylCoeff;
  freqq *= 10;

  if (debug)Serial.print("freqq "); Serial.println(freqq);
  return (freqq);
}
//End Function
//-------------------------------------------------------------------


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
//-------------------------------------------------------------------


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
    up = true;up = true;
    encoder--;
  }
  else
  {
    up = false;
    encoder++;
  }
  TurnDetected = true;
  if (debug)Serial.println (encoder);
}
//End Function
//-------------------------------------------------------------------


//*******************************************************************
/* FUNCTION: PIDCalculations
   INPUT: Speed100 or currentRPM | target100 or targetRPM
   RETURN: pos
   DESCTRIPTION: Runs an equation to calculate Change in speed based off MPH | needs better description later
*/
void PIDCalculations()
{
  //PID INPUTS//
  if (speedMode)
  {
    Input = Speed100;
    Setpoint = Target100;
  }
  else
  {
    Input = currentRPM;
    Setpoint = targetRPM;
  }

  //ADAPTIVE TUNING CALULATIONS//
  // if (adaptiveTuningMode)
  // {
  //   double gap = abs(Setpoint - Input); //distance away from setpoint
  //   if (gap < 10)
  //   { //we're close to setpoint, use conservative tuning parameters
  //     myPID.SetTunings(consKp, consKi, consKd);
  //   }
  //   else
  //   {
  //     //we're far from setpoint, use aggressive tuning parameters
  //     myPID.SetTunings(aggKp, aggKi, aggKd);
  //   }
  //   myPID.Compute();
  //   pos100 = Output;
  // }
  //PROPORTION ON MEASURE MODE//
    myPID.Compute();
    pos100 = Output;
  //----------------------------

  pos100 = constrain(pos100, minServo * 10, maxServo * 10);
  pos = pos100 / 10;
}
//End Function
//-------------------------------------------------------------------




//*******************************************************************
/* FUNCTION:  MainDisplay()
   INPUT: Everything that displays on the main screen
   RETURN: OUtputs the main display
   DESCTRIPTION: This displays everything that you see
*/
void MainDisplay()
{
  //GPS Time Display//
  hours = gps.time.hour() - hourOffset;
  if (hours < 0)
    hours += 24;
  minutes = gps.time.minute();
  seconds = gps.time.second();
  mydisp.setFont(10);
  mydisp.setPrintPos(0, 1);
  mydisp.print(hours); mydisp.print(":"); if (minutes < 10) mydisp.print("0"); mydisp.print(minutes); mydisp.print(":"); if (seconds < 10) mydisp.print("0"); mydisp.print(seconds); mydisp.print("            ");

  //Blank SPace?//
  mydisp.setFont(10);
  mydisp.setPrintPos(10, 2);
  mydisp.setTextPosOffset(0, -6);
  mydisp.setFont(6);

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
  mydisp.print("POWER ");

  //Print word "MIN" or "FULL" or "  "//
  if (pos == maxServo)  mydisp.print("MIN ");
  else  if (pos == minServo)  mydisp.print("FULL ");
  else  mydisp.print(maxServo / 10 - pos / 10);
  mydisp.print("  ");

  //Print word depending on mode selection//
  // 0 Prints "OFF"
  // 1 Prints TargetRPM & "RPM"
  // 2 Prints TargetSpeedInt/10 & TargetSpeedInt%10 & either "MPH" or "KPH"
  mydisp.setFont(18);
  mydisp.setPrintPos(7, 0);
  if (mode == 0) mydisp.print("  OFF  ");
  if (mode == 1) {
    mydisp.print(targetRPM);
    mydisp.print("RPM");
  }
  if (mode == 2) {
    mydisp.print(TargetSpeedInt / 10);   mydisp.print(".");   mydisp.print(TargetSpeedInt % 10);
    if (mph) mydisp.print("MPH"); else mydisp.print("KPH");
  }

  //Prints Voltage
  mydisp.setFont(10);
  mydisp.setPrintPos(0, 2);
  mydisp.print("V ");  mydisp.print(voltage); mydisp.print("  ");

  // Prints Water Temp and Air Temp
  /* mydisp.setPrintPos(0, 3);
    mydisp.print("UNT "); mydisp.print(Temp); if (celsius) mydisp.print("c "); else mydisp.print("F ");
  */
  mydisp.setPrintPos(0, 4);
  mydisp.print("WTR "); mydisp.print(wtrTemp); if (celsius)  mydisp.print("c  "); else mydisp.print("F ");
  mydisp.setPrintPos(0, 5);
  mydisp.print("AIR "); mydisp.print(airTemp); if (celsius)  mydisp.print("c  "); else mydisp.print("F ");

  //Prints currentcurrentRPM reading
  mydisp.setPrintPos(0, 6);
  mydisp.print("RPM "); mydisp.print(currentRPM); mydisp.print("    ");


  /*NOT SURE WHAT THIS WAS FOR
    //Main speed output
    //mydisp.setFont(123);
    mydisp.setPrintPos(0, 0);
    //*/

  //Prints the Main Speed value On Display
  mydisp.setFont(203);
  mydisp.setPrintPos(0, 0);
  mydisp.setTextPosOffset(46, 22);
  if (speedValue < 100) mydisp.print('0');
  mydisp.print(speedValue / 10);
  //mydisp.setFont(51);
  mydisp.setFont(201);
  mydisp.setPrintPos(0, 0);
  mydisp.setTextPosOffset(111, 43);
  mydisp.print(speedValue % 10);



}
//End Function
//-------------------------------------------------------------------


void Menu(){
  
  byte topItemDisplayed = 0;  // stores menu item displayed at top of LCD screen
  byte cursorPosition = 0;  // where cursor is on screen, from 0 --> totalRows. 

  // redraw = 0  - don't redraw
  // redraw = 1 - redraw cursor
  // redraw = 2 - redraw list
  byte redraw = MOVELIST;  // triggers whether menu is redrawn after cursor move.
  byte i=0; // temp variable for loops.
  byte totalMenuItems = 0;  //a while loop below will set this to the # of menu items.

// Put the menu items here. Remember, the first item will have a 'position' of 0.
  char* menuItems[]={
    "Max Throttle", 
    "Min Throttle", 
    "Test Throttle Range",
    "Flip range"
    "Cylinders", 
    "Time zone",
    "Default Target Speed",
    "Speed",
    "Temp",
    "Debug",
  };

  while (menuItems[totalMenuItems] != ""){
    totalMenuItems++;  // count how many items are in list.
  }
  totalMenuItems--;  //subtract 1 so we know total items in array.

  mydisp.clearScreen();  // clear the screen so we can paint the menu.

  boolean stillSelecting = true;  // set because user is still selecting.

  timeoutTime = millis() + menuTimeout; // set initial timeout limit. 

  do   // loop while waiting for user to select.
  {
    /*
    IF YOU WANT OTHER CODE GOING ON IN THE BACKGROUND
    WHILE WAITING FOR THE USER TO DO SOMETHING, PUT IT HERE
    */
  } 
    switch(read_encoder()) 
    {  // analyze encoder response. Default is 0.


    case 1:  // ENCODER UP
      timeoutTime = millis()+menuTimeout;  // reset timeout timer
      //  move menu up one.
      if(cursorPosition == 0 && topItemDisplayed > 0)  //  Cursor is at top of LCD, and there are higher menu items still to be displayed.
      {
        topItemDisplayed--;  // move top menu item displayed up one. 
        redraw = MOVELIST;  // redraw the entire menu
      }
      // if cursor not at top, move it up one.
      if(cursorPosition>0)
      {
        cursorPosition--;  // move cursor up one.
        redraw = MOVECURSOR;  // redraw just cursor.
      }
      break;

    case 2:    //ENCODER DOWN
      timeoutTime = millis()+menuTimeout;  // reset timeout timer
      // this sees if there are menu items below the bottom of the LCD screen & sees if cursor is at bottom of LCD 
      if((topItemDisplayed + (totalRows-1)) < totalMenuItems && cursorPosition == (totalRows-1))
      {
        topItemDisplayed++;  // move menu down one
        redraw = MOVELIST;  // redraw entire menu
      }
      if(cursorPosition<(totalRows-1))  // cursor is not at bottom of LCD, so move it down one.
      {
        cursorPosition++;  // move cursor down one
        redraw = MOVECURSOR;  // redraw just cursor.
      }
      break;

    case 4:  // ENCODER BUTTON SHORT PRESS
      timeoutTime = millis()+menuTimeout;  // reset timeout timer
      switch(topItemDisplayed + cursorPosition) // adding these values together = where on menuItems cursor is.
      {
      //  put code to be run when specific item is selected in place of the Serial.print filler.
      // the Serial.print code can be removed, but DO NOT change the case & break structure. 
      // (Obviously, you should have as many case instances as you do menu items.)
      case 0:  // menu item 1 selected
        Serial.print("Menu item ");
        Serial.print(topItemDisplayed + cursorPosition);
        Serial.print(" selected - ");
        Serial.println(menuItems[topItemDisplayed + cursorPosition]);
        // menuSubMenu1();
        break;

      case 1:  // menu item 2 selected
        Serial.print("Menu item ");
        Serial.print(topItemDisplayed + cursorPosition);
        Serial.print(" selected - ");
        Serial.println(menuItems[topItemDisplayed + cursorPosition]);
        //menuSubMenu2();
        break;

      case 2:  // menu item 3 selected
        Serial.print("Menu item ");
        Serial.print(topItemDisplayed + cursorPosition);
        Serial.print(" selected - ");
        Serial.println(menuItems[topItemDisplayed + cursorPosition]);
        //menuSubMenu3();
        break;

      case 3:  // menu item 4 selected
        Serial.print("Menu item ");
        Serial.print(topItemDisplayed + cursorPosition);
        Serial.print(" selected - ");
        Serial.println(menuItems[topItemDisplayed + cursorPosition]);
        //menuSubMenu4();
        break;

      case 4:  // menu item 5 selected
        Serial.print("Menu item ");
        Serial.print(topItemDisplayed + cursorPosition);
        Serial.print(" selected - ");
        Serial.println(menuItems[topItemDisplayed + cursorPosition]);
        //menuSubMenu5();
        break;

      case 5:  // menu item 6 selected
        Serial.print("Menu item ");
        Serial.print(topItemDisplayed + cursorPosition);
        Serial.print(" selected - ");
        Serial.println(menuItems[topItemDisplayed + cursorPosition]);
        //menuSubMenu6();
        break;

      case 6:  // menu item 7 selected
        Serial.print("Menu item ");
        Serial.print(topItemDisplayed + cursorPosition);
        Serial.print(" selected - ");
        Serial.println(menuItems[topItemDisplayed + cursorPosition]);
        //menuSubMenu7();
        break;

        case 7:  // menu item 8 selected
        Serial.print("Menu item ");
        Serial.print(topItemDisplayed + cursorPosition);
        Serial.print(" selected - ");
        Serial.println(menuItems[topItemDisplayed + cursorPosition]);
        //menuSubMenu8();
        break;

        case 8:  // menu item 9 selected
        Serial.print("Menu item ");
        Serial.print(topItemDisplayed + cursorPosition);
        Serial.print(" selected - ");
        Serial.println(menuItems[topItemDisplayed + cursorPosition]);
        //menuSubMenu9();
        break;

        case 9:  // menu item10 selected
        Serial.print("Menu item ");
        Serial.print(topItemDisplayed + cursorPosition);
        Serial.print(" selected - ");
        Serial.println(menuItems[topItemDisplayed + cursorPosition]);
        //menuSubMenu10();
        break;
        // add as many "case #:" as items. Could put 
        //  line separators in menu and leave out the 
        //  corresponding case, which would mean that nothing
        // would be triggered when user selected the line separator.  
      }
      break;
      
    case 8:  // encoder button was pushed for long time. This corresponds to "Back" or "Cancel" being pushed.
      stillSelecting = false;
      Serial.println("Button held for a long time");
      break;

    }

    switch(redraw){  //  checks if menu should be redrawn at all.
    case MOVECURSOR:  // Only the cursor needs to be moved.
      redraw = false;  // reset flag.
      if (cursorPosition > totalMenuItems) // keeps cursor from moving beyond menu items.
        cursorPosition = totalMenuItems;
      for(i = 0; i < (totalRows); i++){  // loop through all of the lines on the LCD
        mydisp.setPrintPos(0, i);
        mydisp.print(" ");                      // and erase the previously displayed cursor
        mydisp.setPrintPos((totalCols-1), i);
        mydisp.print(" ");
      }
      mydisp.setPrintPos(0,cursorPosition);      // go to mydisp line where new cursor should be & display it.
      mydisp.print(">");
      mydisp.setPrintPos((totalCols-1), cursorPosition);
      mydisp.print("<");
      break;  // MOVECURSOR break.

    case MOVELIST:  // the entire menu needs to be redrawn
      redraw=MOVECURSOR;  // redraw cursor after clearing LCD and printing menu.
      mydisp.clearScreen(); // clear screen so it can be repainted.
      if(totalMenuItems>((totalRows-1))){  // if there are more menu items than LCD rows, then cycle through menu items.
        for (i = 0; i < (totalRows); i++){
          mydisp.setPrintPos(1,i);
          mydisp.print(menuItems[topItemDisplayed + i]);
        }
      }
      else{  // if menu has less items than LCD rows, display all available menu items.
        for (i = 0; i < totalMenuItems+1; i++){
          mydisp.setPrintPos(1,i);
          mydisp.print(menuItems[topItemDisplayed + i]);
        }
      }
      break;  // MOVELIST break
    }

    if (timeoutTime<millis()){  // user hasn't done anything in awhile
      stillSelecting = false;  // tell loop to bail out.
      /*
      in my main code, I had a function that
       displayed a default screen on the LCD, so
       I would put that function here, and it would
       bail out to the default screen.
       defaultScreen();
       */
    }
  } 


  while (stillSelecting == true);  //
}



//   mydisp.setFont(10);
//   mydisp.setPrintPos(0, 0);
//   if (menuItem == 1) {
//     mydisp.print(">>");
//     pos = maxServo;
//   }
//   mydisp.print("Min Throttle ");  if (Reverse) mydisp.print(minServo); else mydisp.print(maxServo);  mydisp.print("    ");
//   delay(10);
//   mydisp.setPrintPos(0, 1);
//   if (menuItem == 2)  {
//     mydisp.print(">>");
//     pos = minServo;
//   }
//   mydisp.print("Max Throttle ");   if (Reverse) mydisp.print(maxServo); else mydisp.print(minServo);   mydisp.print("  ");
//   delay(10);

//   mydisp.setPrintPos(0, 2);
//   if (menuItem == 3) mydisp.print(">>");
//   mydisp.print("Swing  ");   mydisp.print("("); if (Reverse) mydisp.print(minServo + maxServo - pos); else mydisp.print(pos); mydisp.print(")     ");
//   delay(10);
//   if (menuItem == 3)
//   {
//     if (pos <= minServo) S = 1;
//     if (pos >= maxServo) S = -1;
//     pos += 10 * S;
//   }
//   if (Reverse) myservo.writeMicroseconds( minServo + maxServo - pos);
//   else  myservo.writeMicroseconds(pos);

//   mydisp.setFont(6);
//   mydisp.setPrintPos(0, 5);
//   if (menuItem == 4) mydisp.print(">>");
//   mydisp.print("Cylinders ");  mydisp.print(cylCoeff);  mydisp.print("    ");
//   delay(10);

//   mydisp.setPrintPos(0, 6);
//   if (menuItem == 5) mydisp.print(">>");
//   mydisp.print("Time zone ");  mydisp.print(hourOffset);  mydisp.print("    ");
//   delay(10);

//   mydisp.setPrintPos(0, 7);
//   if (menuItem == 6) mydisp.print(">>");
//   mydisp.print("Reverse ");  mydisp.print(Reverse);  mydisp.print("   ");
//   delay(10);


//   mydisp.setPrintPos(0, 8);
//   if (menuItem == 7) mydisp.print(">>");
//   mydisp.print("Ovrst ");  mydisp.print(Kd);   mydisp.print("    ");
//   delay(10);

//   mydisp.setPrintPos(0, 9);
//   if (menuItem == 8) mydisp.print(">>");
//   mydisp.print("Rspns ");  mydisp.print(Ki);    mydisp.print("    ");
//   delay(10);

//   mydisp.setPrintPos(17, 5);
//   if (menuItem == 9) mydisp.print(">>");
//   mydisp.print("DFT SP ");   mydisp.print(Target100 / 100);
//   mydisp.print("  ");
//   delay(10);

//   mydisp.setPrintPos(17, 6);
//   if (menuItem == 10) mydisp.print(">> ");
//   mydisp.print("SPEED ");  if (mph) mydisp.print("MPH"); else mydisp.print("KPH");
//   mydisp.print("  ");
//   delay(10);

//   mydisp.setPrintPos(17, 7);
//   if (menuItem == 11) mydisp.print(">>");
//   mydisp.print("Temp ");  if (celsius) mydisp.print("C"); else mydisp.print("F");
//   mydisp.print("  ");
//   delay(10);

//   mydisp.setPrintPos(17, 8);
//   if (menuItem == 12) mydisp.print(">>");
//   mydisp.print("DEBUG ");    mydisp.print(debug);
//   delay(10);

//   mydisp.setPrintPos(17, 9);
//   mydisp.print("VER: "); mydisp.print(fltCurrentVersion);

//   mydisp.setPrintPos(29, 9);      mydisp.setFont(6);     mydisp.print(buttonTimes2); mydisp.print(" "); mydisp.print(menuItem);


//   if (TurnDetected)
//   { // do this only if rotation was detected
//     if ((millis() - prevtime) > threshold)
//     {
//       prevtime = millis();
//       if (up == prev_up)
//       {
//         if (up)
//         {
//           if (menuItem == 1)
//             if (Reverse)
//             {
//               minServo -= 10;
//             }
//             else
//             {
//               maxServo -= 10;
//             }
//           if (menuItem == 2)
//           {
//             if (Reverse)
//             {
//               maxServo -= 10;
//             }
//             else
//             {
//               minServo -= 10;
//             }
//             //if (menuItem == 3)
//             if (menuItem == 4) cylCoeff--;
//             if (menuItem == 5) hourOffset -= 1;
//             if (menuItem == 6) Reverse = !Reverse;
//             //if (menuItem == 7) Kd -= 1;
//             //if (menuItem == 8)  Ko -= 1;
//             if (menuItem == 9)  Target100 -= 100;
//             if (menuItem == 10) mph = !mph;
//             if (menuItem == 11) celsius = !celsius;
//             if (menuItem == 12) debug = !debug;

//           }
//           else
//           {
//             if (menuItem == 1)
//               if (Reverse)
//               {
//                 minServo += 10;
//               }
//               else
//               {
//                 maxServo += 10;
//               }
//             if (menuItem == 2)
//               if (Reverse)
//               {
//                 maxServo += 10;
//               }
//               else
//               {
//                 minServo += 10;
//               }
//             //if (menuItem == 3)
//             if (menuItem == 4) cylCoeff++;
//             if (menuItem == 5) hourOffset += 1;
//             if (menuItem == 6) Reverse = !Reverse;
//             //if (menuItem == 7) Kd += 1;
//             //if (menuItem == 8) Ko += 1;
//             if (menuItem == 9)  Target100 += 100;
//             if (menuItem == 10) mph = !mph;
//             if (menuItem == 11) celsius = !celsius;
//             if (menuItem == 12) debug = !debug;
//           }
//         }
//         else
//         {
//           prev_up = up;
//         }
//         TurnDetected = false;          // do NOT repeat IF_loop until new rotation detected
//       }
//     }

//     if (digitalRead(PinSW) == LOW)  //menu scroll
//     {
//       delay(250);
//         if (digitalRead(PinSW) == HIGH)   if (menuItem > 11) menuItem = 1; else menuItem++;
//       buttonTimes2++;
//     }
//     else {
//       buttonTimes2 = 0;
//     }

//     if (buttonTimes2 > 4)
//     {
//       buttonTimes = 0;  buttonTimes2 = 0;  menuItem = 1;  mydisp.clearScreen();
//     }    //exit procedure begin

//     if (EEPROM.read(11) != cylCoeff)
//       EEPROM.write(11, cylCoeff);
//     if (EEPROM.read(12) != aggKp      )
//     EEPROM.write(12, aggKp);
//     if (EEPROM.read(13) != aggKd      )
//     EEPROM.write(13, aggKd);
//     if (EEPROM.read(14) != aggKi      )
//     EEPROM.write(14, aggKi);
//     if (EEPROM.read(22) != consKp      )
//     EEPROM.write(22, aggKp);
//     if (EEPROM.read(23) != consKd      )
//     EEPROM.write(23, aggKd);
//     if (EEPROM.read(24) != consKi      )
//     EEPROM.write(24, aggKi);
//     if (EEPROM.read(15) != 24 - hourOffset)
//       EEPROM.write(15, (24 - hourOffset));
//     if (EEPROM.read(16) != maxServo / 10 )
//       EEPROM.write(16, (maxServo / 10));
//     if (EEPROM.read(17) != minServo / 10 )
//       EEPROM.write(17, (minServo / 10));
//     if (EEPROM.read(18) != mph     )
//       EEPROM.write(18, (mph));
//     if (EEPROM.read(29) != celsius )
//       EEPROM.write(29, (celsius));
//     if (EEPROM.read(30) != Target100 / 100 )
//     {
//       EEPROM.write(30, (Target100 / 100));   //check for accidental EEPROM write
//       Serial.print("EEPROM!: ");
//       Serial.println(millis());
//     }
//     if (EEPROM.read(31) != Reverse )
//       EEPROM.write(31, Reverse);

//   }  //while exit condition met
// }  //end menu



int read_encoder(){
  #define btnUp         1
  #define btnDown       2
  #define btnPress      4
  #define btnLongPress  8
  
  if (TurnDetected){
    if (up = true) return btnUp;
    if (up = false) return btnDown;
    TurnDetected = false;
  }
  if (digitalRead(PinSW) == LOW)
  {
    inMenuPress++;
  }
  else {
    if (inMenuPress > 0) {
      return btnPress;
      inMenuPress = 0;
    }
  }
  if (inMenuPress > inMenuPressTarget){
    return btnLongPress;
  }
}



void DebugOutput()
{
  Serial.print("Time: ");   Serial.print(millis());
  Serial.print(" GPS_Sp: ");  Serial.print(Speed100);
  Serial.print(" Sp_Targ100: ");  Serial.print(Target100);
  Serial.print("currentRPM: "); Serial.print(currentRPM);
  Serial.print(" RPM_target: "); Serial.print(targetRPM);
  Serial.print(" Srv: "); Serial.println(pos);
}


