/*
   _________ __                     .___       __________                        ____   ________
  /   _____//  |_  ____ _____     __| _/__.__. \______   \_____    ______ ______ \   \ /   /_   |
  \_____  \\   __\/ __ \\__  \   / __ <   |  |  |     ___/\__  \  /  ___//  ___/  \       / |   |
  /        \|  | \  ___/ / __ \_/ /_/ |\___  |  |    |     / __ \_\___ \ \___ \    \     /  |   |
 /_______  /|__|  \___  >____  /\____ |/ ____|  |____|    (____  /____  >____  >    \___/   |___|
================================================================================================
************************************************************************************************
  ---|FILE: SteadyPassV1.ino
  ---|Created By: Dimitry
  ---|Description: This is the code used to make everything work
  ---|
  ---|
  ---|Change History:
  ---|
  ---|	16.54	- Sam - Renamed File from "Limiter_16_53.ino", Added Title and Change History, Added Function comment and Descriptions, All previous Versions named "Limiter_xx_xx" are now in the Hands of Dmitry
*/



//*************LIBRARYS*************
#include <TinyGPS++.h>
#include <Servo.h>
#include <SPI.h>
#include <OneWire.h>
#include <EEPROM.h>
#define _Digole_Serial_SPI_
#include <DigoleSerial.h>
//----------------------------------

//*************INITIALIZING DEFINITIONS*************
float fltCurrentVersion = 16.54;
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
//--------------------------------------------------


//*************PIN CONFIGURATION*************
                                  //0 and 1 = GPS  , int 2 and 3
const int PinCLK = 2;             //int 1 == pin 2 encoder second pin
const int PinDT = 3;              //int0 == pin3    // Used for reading DT signal of encoder
const int PinSW = 4;              //encoder button
const int servoPin = 5;           //servo PWM connected here
const int speedPin = 6;           //reserve    int4 == pin 7? speed sensor
const int rpmPin = 7;             //Engine RPM input connected here
                                  //8 and 9 - LCD, defined above
                                  //open - 10 - LCD SS?
const int tempWtrPin = 11;        //temperature water
const int tempAirPin = 12;        //temperature air
const int voltPin = A0;           //battery voltage divider
                                  //OPEN A1, A2, A3 - relay out?
const int tempBoardPin = A4;      //temperature on control board
const int lightsPin = A5;         //A5;  // lights on voltage divider
//-------------------------------------------


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
long RPM = 0;
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
                      //int minServo = 50;
                      //int maxServo = 138;   //Linear
                      //  int minServo = 54;
                      // int maxServo = 100;
int minServo = 1050;
int maxServo = 1950;

                      // int minServo = 600;   //S3003
                      // int maxServo = 1500;
int pos = minServo;   //servo position
                      //int maxDelta = 8;
int S = 10;
//-----------------END SERVO--------------------


//*************ENCODER INITIALIZATION*************
volatile boolean TurnDetected = false;
volatile boolean up = false, prev_up = false;
unsigned long interruptTime = 0;
long prevtime = 0;
long threshold = 7;
int buttonTimes = 0;
int buttonTarget = 7;
int buttonTimes2 = 0;
int menuItem = 1;
int idleCounter = 0, idleLimit = 2500;
boolean mainDisplay = true;
boolean modeOn = true;
volatile long encoder = 0;
int hours, minutes, seconds, hourOffset = 7;
//-----------------END ENCODER--------------------


//*************FUNCTION INITIALIZATION*************
float readSpeed();                          // read GPS speed
int readRpm();                              //read/calculate RPM and reset counters
void rpmIntHandler();                       //called by interupt, increments counters
static void smartDelay(unsigned long ms);   //fuction reads temp/ voltage
void isr();                                 // encoder - Interrupt service routine is executed when a HIGH to LOW transition is detected on CLK
void SpeedModeCalculations();
void RPMModeCalculations();
void MainDisplay();
void Menu();
void DebugOutput();
//-----------------END--------------------


//*************PID variables*************
int Kp = 30, Kd = 25, Ko = 20; //Ki=0,   /*PID coefficients    eeprom*/
long deltaD, deltaP;
//---------------------------------------


//************************************************************************************************|
//*********** PERFORM STARTUP TASKS | DISPLAY STARTUP | GPS CONFIGURATION | READ MEMORY **********|
//************************************************************************************************|
void setup()
{
  Serial.begin(9600);
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


  //******************LOADING SAVED VALUES*******************
  if (EEPROM.read(11) != 255) cylCoeff = EEPROM.read(11);
  if (EEPROM.read(12) != 255) Kp =  EEPROM.read(12);
  if (EEPROM.read(13) != 255) Kd = EEPROM.read(13);
  if (EEPROM.read(14) != 255) Ko = EEPROM.read(14); // Ko = Ko/100;
  if (EEPROM.read(15) != 255) hourOffset = 24 - EEPROM.read(15);
  if (EEPROM.read(16) != 255) maxServo = 10 * EEPROM.read(16);
  if (EEPROM.read(17) != 255)  minServo = 10 * EEPROM.read(17);
  if (EEPROM.read(18) != 255) mph = EEPROM.read(18);
  if (EEPROM.read(29) != 255)  celsius = EEPROM.read(29);
  if (EEPROM.read(30) != 255)  Target100 = EEPROM.read(30) * 100;
  if (EEPROM.read(31) != 255)  Reverse = EEPROM.read(31);
  //----------------LOADED VALUES COMPLETE-------------------

  //******************LOADING GPS CONFIGURATION*******************
  //Serial1.println("$PMTK314,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29");   //Sets GPS to GTV only
  Serial1.println("$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29");     //Sets GPS to RMC only
  Serial1.println("$PMTK220,200*2C");  //Set GPS to 5hz
  //----------------GPS CONFIGURATION COMPLETE-------------------


 Serial.println("SETUP COMPLETE");
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
  { digitalWrite(13, LOW);
    led = false;
  }
  else
  { digitalWrite(13, HIGH);
    led = true;
  }
  smartDelay(1);              //read GPS
  if (gps.speed.isValid())
  {
    speedGpsD = gps.speed.mph();
  }
  else
  {
    //print N/A or searching where current MPH should be
    //possible enhancment to switch to RPM calculation if GPS signal lost for extended time.
  }
  Speed100 = 100 * speedGpsD;
  Serial.print("Speed 100: ");   Serial.println(Speed100);
  RPM = readRpm();
  
  //Get Total Run time since powerup//
  if (RPM > 50) 
  {
    tachTime += millis() - tachTimePrev;
    tachTimePrev = millis();
  }
  else 
  {
    tachTimePrev = millis();    //Current Running Time
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
          speedGpsD = gps.speed.mph();
        Speed100 = 100 * speedGpsD;
        mydisp.setPrintPos(0, 0);
        mydisp.print("                  ");
        mydisp.setPrintPos(0, 1);
        mydisp.print("                  ");
      }
    }



  for (int i = 10; i > 0; i--)  speedBuffer[i] = speedBuffer[i - 1];    //store GPS speeds
  speedBuffer[0] = Speed100;
  avSpeed100 = 0;

  for (int i = 5; i >= 0; i--)  {
    avSpeed100 += speedBuffer[i];
  }
  avSpeed100 = avSpeed100 / 6;                                        //speed average for last i+1 steps

  for (int i = 10; i > 0; i--) rpmBuffer[i] = rpmBuffer[i - 1];      //store RPM values in array
  rpmBuffer[0] = RPM;




  // if (gpsMode) speedValue = Speed100; else speedValue = 100*speedWater;  //use speedFloat for calculations


  //control adjustment calculation
  if (mode == 2)      SpeedModeCalculations();  ////////////speed mode ///////////////
  //PrevSpeed = Speed100;
  if (mode == 1)     RPMModeCalculations();   /////////////// RPM mode   /////////////////
  if (mode == 0)  pos = minServo;       //  OFF MODE


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


  //screensaver section
  if (RPM < 50) idleCounter++; else {
    idleCounter = 0;
  }

  if ((idleCounter > idleLimit) && mainDisplay)
  { //screensaver_temporarily_disabled    mainDisplay = false; mydisp.clearScreen();   //
  }


  if (!mainDisplay)     //Screensaver page
  {


    mydisp.setFont(120);
    mydisp.setPrintPos(0, 0);
    mydisp.setTextPosOffset(20, 15);
    mydisp.print(hours); mydisp.print(":"); if (minutes < 10) mydisp.print("0"); mydisp.print(minutes); mydisp.print("  ");

    mydisp.setFont(6);
    mydisp.setPrintPos(0, 2);
    mydisp.setTextPosOffset(0, 0);


    mydisp.setFont(10);
    mydisp.setPrintPos(0, 0);
    mydisp.print("Tach today "); mydisp.print(tachTime / 60000); mydisp.println(" min");
    delay(20);

    mydisp.setPrintPos(0, 6); mydisp.print("UNT "); mydisp.print(Temp); if (celsius) mydisp.print("c"); else mydisp.print("F"); mydisp.print(" Batt "); mydisp.print(voltage); mydisp.print("v ");

      mydisp.setPrintPos(0, 5); mydisp.print("WTR "); mydisp.print(wtrTemp); if (celsius) mydisp.print("c"); else mydisp.print("F");  mydisp.print(" AIR "); mydisp.print(airTemp); if (celsius) mydisp.print("c "); else mydisp.print("F ");





  //delay(100);

  if (RPM > 500)  {
    idleCounter = 0;
    mainDisplay = true;
    mydisp.clearScreen();
  }

}   //end screen saver

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
    if((b=='g'))  {gpsMode =!gpsMode;  targetRPM = RPM;}

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


static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (Serial1.available())
    {
      gps.encode(Serial1.read());   ;
    }
  }
  while ( !(gps.speed.isUpdated())  );                  //while (millis() - start < ms);
}







//-------------------------------------------------------------------
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

  //  Serial.print("freqq ");Serial.println(freqq);
  return (freqq);
}
//End Function
//-------------------------------------------------------------------


//*******************************************************************
/* FUNCTION: rpmIntHandler()
   INPUT: micros()
   RETURN: N/A
   DESCTRIPTION: it's a interrupt handler for RPMs need better knowledge for better description
*/
void rpmIntHandler() // interrupt handler
{
  unsigned long currentMicros = micros();
  duration += currentMicros - previousMicros;
  previousMicros = currentMicros;
  pulsecount++;
}
//End Function
//-------------------------------------------------------------------


//*******************************************************************
/* FUNCTION: rpmIntHandler()
   INPUT: micros()
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
  // Serial.println (encoder);
}
//End Function
//-------------------------------------------------------------------


//*******************************************************************
/* FUNCTION: SpeedModeCalculations()
   INPUT: Speed100 | speedBuffer
   RETURN: pos
   DESCTRIPTION: Runs an equation to calculate Change in speed based off MPH | needs better description later
*/
void SpeedModeCalculations()
{
  // checkSpeed = millis() + 1000;

  int SpeedDelta = Speed100 - speedBuffer[1];
  SpeedCorrected = Speed100 + SpeedDelta;
  deltaP = SpeedCorrected - Target100;
  deltaD = PrevSpeed - Speed100;
  // PrevSpeed = Speed100;
  if (abs(Speed100 - Target100) < 150)
    deltaD = deltaD * 4;
  if (abs(Speed100 - Target100) < 70)
    deltaD = deltaD * 5;
  deltaP = constrain(deltaP, -150, 150);

  delta100 = Ko * (Kp * (float)deltaP / 10 - Kd * (float)deltaD / 10) / 100;


  pos100 += delta100;

  pos100 = constrain(pos100, minServo * 10, maxServo * 10);
  pos = pos100 / 10;
}  //end SPEED Mode
//End Function
//-------------------------------------------------------------------


//*******************************************************************
/* FUNCTION: RPMModeCalculations()
   INPUT: Speed100 | speedBuffer
   RETURN: pos
   DESCTRIPTION: Runs an equation to calculate Change in speed based off RPM | needs better description later
*/
void RPMModeCalculations()
{
  // int rpmDelta = Speed100 - speedBuffer[3];
  //  SpeedCorrected = Speed100 + SpeedDelta;
  deltaD = avRPM - RPM;
  avRPM = 0;
  for (int i = 2; i >= 0; i--) avRPM += rpmBuffer[i];        //calculate average RPM for i+1 steps
  avRPM = avRPM / 3;
  //avRPM = RPM;   //cancelling averaging for test
  deltaP = avRPM - targetRPM;

  delta100 = Ko * (Kp * (float)deltaP / 10 - Kd * (float)deltaD / 10) / 100;
  pos100 += delta100;
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

  //Prints current RPM reading
  mydisp.setPrintPos(0, 6);
  mydisp.print("RPM "); mydisp.print(RPM); mydisp.print("    ");


  /*NOT SURE WHAT THIS WAS FOR
    //Main speed output
  	//mydisp.setFont(123);
  	mydisp.setPrintPos(0, 0);
    //
  */
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


void Menu()
{

  mydisp.setFont(10);
  mydisp.setPrintPos(0, 0);
  if (menuItem == 1) {
    mydisp.print(">>");
    pos = maxServo;
  }
  mydisp.print("Min Throttle ");  if (Reverse) mydisp.print(minServo); else mydisp.print(maxServo);  mydisp.print("    ");
  delay(10);
  mydisp.setPrintPos(0, 1);
  if (menuItem == 2)  {
    mydisp.print(">>");
    pos = minServo;
  }
  mydisp.print("Max Throttle ");   if (Reverse) mydisp.print(maxServo); else mydisp.print(minServo);   mydisp.print("  ");
  delay(10);

  mydisp.setPrintPos(0, 2);
  if (menuItem == 3) mydisp.print(">>");
  mydisp.print("Swing  ");   mydisp.print("("); if (Reverse) mydisp.print(minServo + maxServo - pos); else mydisp.print(pos); mydisp.print(")     ");
  delay(10);
  if (menuItem == 3)
  {
    //delay(50);
    if (pos <= minServo) S = 1;
    if (pos >= maxServo) S = -1;
    pos += 10 * S;
    //myservo.write(pos);
    // myservo.writeMicroseconds(pos);

  }
  if (Reverse) myservo.writeMicroseconds( minServo + maxServo - pos);
  else  myservo.writeMicroseconds(pos);

  mydisp.setFont(6);
  mydisp.setPrintPos(0, 5);
  if (menuItem == 4) mydisp.print(">>");
  mydisp.print("Cylinders ");  mydisp.print(cylCoeff);  mydisp.print("    ");
  delay(10);

  mydisp.setPrintPos(0, 6);
  if (menuItem == 5) mydisp.print(">>");
  mydisp.print("Time zone ");  mydisp.print(hourOffset);  mydisp.print("    ");
  delay(10);

  mydisp.setPrintPos(0, 7);
  if (menuItem == 6) mydisp.print(">>");
  mydisp.print("Reverse ");  mydisp.print(Reverse);  mydisp.print("   ");
  delay(10);
  //  if (menuItem == 6) menuItem ++;


  mydisp.setPrintPos(0, 8);
  if (menuItem == 7) mydisp.print(">>");
  mydisp.print("Ovrst ");  mydisp.print(Kd);   mydisp.print("    ");
  delay(10);


  mydisp.setPrintPos(0, 9);
  if (menuItem == 8) mydisp.print(">>");
  mydisp.print("Rspns ");  mydisp.print(Ko);    mydisp.print("    ");
  delay(10);

  mydisp.setPrintPos(17, 5);
  if (menuItem == 9) mydisp.print(">>");
  mydisp.print("DFT SP ");   mydisp.print(Target100 / 100);
  mydisp.print("  ");
  delay(10);


  mydisp.setPrintPos(17, 6);
  if (menuItem == 10) mydisp.print(">> ");
  mydisp.print("SPEED ");  if (mph) mydisp.print("MPH"); else mydisp.print("KPH");
  mydisp.print("  ");
  delay(10);

  mydisp.setPrintPos(17, 7);
  if (menuItem == 11) mydisp.print(">>");
  mydisp.print("Temp ");  if (celsius) mydisp.print("C"); else mydisp.print("F");
  mydisp.print("  ");
  delay(10);

  mydisp.setPrintPos(17, 8);
  if (menuItem == 12) mydisp.print(">>");
  mydisp.print("DEBUG ");    mydisp.print(debug);
  delay(10);

  mydisp.setPrintPos(17, 9);
  mydisp.print("VER: "); mydisp.print(fltCurrentVersion);

  mydisp.setPrintPos(29, 9);      mydisp.setFont(6);     mydisp.print(buttonTimes2); mydisp.print(" "); mydisp.print(menuItem);


  if (TurnDetected)
  { // do this only if rotation was detected
    if ((millis() - prevtime) > threshold) {
      prevtime = millis();
      if (up == prev_up) {
        if (up)
        {
            if (menuItem == 1) if (Reverse) minServo -= 10; else maxServo -= 10;
            if (menuItem == 2)  if (Reverse) maxServo -= 10; else minServo -= 10;
          //if (menuItem == 3)
          if (menuItem == 4)  cylCoeff--;
          if (menuItem == 5) hourOffset -= 1;
          if (menuItem == 6)  Reverse = !Reverse;
          if (menuItem == 7) Kd -= 1;
          if (menuItem == 8)  Ko -= 1;
          if (menuItem == 9)  Target100 -= 100;
          if (menuItem == 10)  mph = !mph;
          if (menuItem == 11) celsius = !celsius;
          if (menuItem == 12) debug = !debug;

        }
        else
        {
            if (menuItem == 1) if (Reverse) minServo += 10; else maxServo += 10;
            if (menuItem == 2)  if (Reverse) maxServo += 10; else minServo += 10;
          //if (menuItem == 3)
          if (menuItem == 4)  cylCoeff++;
          if (menuItem == 5) hourOffset += 1;
          if (menuItem == 6)  Reverse = !Reverse;
          if (menuItem == 7) Kd += 1;
          if (menuItem == 8)  Ko += 1;
          if (menuItem == 9)  Target100 += 100;
          if (menuItem == 10)  mph = !mph;
          if (menuItem == 11) celsius = !celsius;
          if (menuItem == 12) debug = !debug;
        }
      } else {
        prev_up = up;
      }
      TurnDetected = false;          // do NOT repeat IF_loop until new rotation detected
    }
  }

  if (digitalRead(PinSW) == LOW)  //menu scroll
  {
    delay(250);
      if (digitalRead(PinSW) == HIGH)   if (menuItem > 11) menuItem = 1; else menuItem++;
    buttonTimes2++;
  }
  else {
    buttonTimes2 = 0;
  }

  if (buttonTimes2 > 4)
  { buttonTimes = 0;  buttonTimes2 = 0;  menuItem = 1;  mydisp.clearScreen();    //exit procedure begin

    if (EEPROM.read(11) != cylCoeff)
      EEPROM.write(11, cylCoeff);
    if (EEPROM.read(12) != Kp      )
      EEPROM.write(12, Kp);
    if (EEPROM.read(13) != Kd      )
      EEPROM.write(13, Kd);
    if (EEPROM.read(14) != Ko      )
      EEPROM.write(14, Ko);
    if (EEPROM.read(15) != 24 - hourOffset)
      EEPROM.write(15, (24 - hourOffset));
    if (EEPROM.read(16) != maxServo / 10 )
      EEPROM.write(16, (maxServo / 10));
    if (EEPROM.read(17) != minServo / 10 )
      EEPROM.write(17, (minServo / 10));
    if (EEPROM.read(18) != mph     )
      EEPROM.write(18, (mph));
    if (EEPROM.read(29) != celsius )
      EEPROM.write(29, (celsius));
    if (EEPROM.read(30) != Target100 / 100 )
    {
      EEPROM.write(30, (Target100 / 100));   //check for accidental EEPROM write
      Serial.print("EEPROM!: ");
      Serial.println(millis());
    }
    if (EEPROM.read(31) != Reverse )
      EEPROM.write(31, Reverse);

  }  //while exit condition met
}  //end menu

void DebugOutput()
{
  Serial.print("Time: ");   Serial.print(millis());
  Serial.print(" GPS_Sp: ");  Serial.print(Speed100);
  Serial.print(" Sp_Targ100: ");  Serial.print(Target100);
  Serial.print(" RPM: "); Serial.print(RPM);
  Serial.print(" RPM_target: "); Serial.print(targetRPM);
  Serial.print(" Srv: "); Serial.println(pos);
}

