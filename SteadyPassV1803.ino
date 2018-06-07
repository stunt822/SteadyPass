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
  ---|  16.53 - Dimitry   - Allversion Prior to 17.00 have been soley created by Dimitry
  ---|  17.xx - Sam James - Renamed File from "Limiter_16_53.ino", Added Title and Change History, Added Function comment and Descriptions, Minor code changes made that will not effect the main code, Remade speed and RPM calc into one function, Created knob and button switches. Long press, short press,knob up, and knob down. Added basic scroll menu.
  ---|  18.00 - Sam James - New Menu, PID functioning correctly, optimizations
  ---|  18.01 - ''''''''' - Screen redraw fix, changed description on min/max servo, Kpid settings default text changed, exit from any >>>
  ---|  18.02 - ''''''''' - menu with long press, PID now pulls range change without reboot, created settings initialization on first boot
  ---|  18.03 - ''''''''' - Fixed Time Zone on boot and in menu setting, fixed servo pos change on setting change
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
//---------------------

double CurrentVersion = 18.03;

//*************INITIALIZING DEFINITIONS*************
boolean firstLoopOnStart = true;
boolean firstMainDispLoop =true;
boolean stillSelecting = true;
double Setpoint, Input, Output;
double aggKp = .25, aggKi = .25 , aggKd = .50; //NEED TO MAKE MENU OPTION FOR ADJUSTMENTS ON THE FLY
double consKp = .10, consKi = .10, consKd = .25; //NEED TO MAKE MENU OPTION FOR ADJUSTMENTS ON THE FLY
PID myPID(&Input, &Output, &Setpoint, aggKp, aggKp, aggKp, REVERSE);
#define MOVECURSOR 1  // constants for indicating whether cursor should be redrawn
#define MOVELIST 2  // constants for indicating whether cursor should be redrawn
byte totalRows = 6;  // total rows of LCD
byte totalCols = 1;  // total columns of LCD

//GPS speed Average Vars
const int numReadings = 5;
int readings[numReadings];
int readIndex = 0;
int total = 0;
int averagespeed100 = 0;

TinyGPSPlus gps;                                //required for TinyGPSplus Library
DigoleSerialDisp mydisp(9, 8, 10);              //Pin Config SPI | 9: data | 8:clock | 10: SS | you can assign 255 to SS, and hard ground SS pin on module
boolean led = false;                            //LED on | off
float voltage = 0, volt1, volt2;                //voltage value
byte cylCoeff = 6;                               //Number of Cylinders
boolean gpsMode = true;                         //GPS mode (true) or RPM mode (false)
byte mode = 0;                                   //Calculations mode 0=off 1=RPM 2=MPH/KPH


//*****PIN CONFIGURATION******
//0 and 1 = GPS
const byte PinCLK = 2;             //encoder second pin
const byte PinDT = 3;              // Used for reading DT signal of encoder
const byte PinSW = 4;              //encoder button
const byte servoPin = 5;           //servo PWM connected here
const byte rpmPin = 7;             //Engine RPM input connected here
//8 and 9 - LCD, defined above
//open - 10 - LCD SS?
const byte tempWtrPin = 11;        //temperature water
const byte tempAirPin = 12;        //temperature air
const byte voltPin = A0;           //battery voltage divider
//OPEN A1, A2, A3, A5
const byte tempBoardPin = A4;      //temperature on control board
//----------------------

//***TEMP SENSOR INITIALIZATION***
OneWire  ds(tempBoardPin);
OneWire  wtr(tempWtrPin);
OneWire  air(tempAirPin);
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
boolean mph = true;
unsigned long delayCheck = 0;
unsigned long throttleCheck = 0;
int targetSpeedWhole = TargetSpeedInt / 10;
int targetSpeedDecimal = TargetSpeedInt %10;

//*************RPM INITIALIZATION*************
volatile unsigned long duration = 0; // accumulates pulse width
volatile unsigned long pulsecount = 0; //incremented by interrupt
volatile unsigned long previousMicros = 0;
int targetRPM = 3000;

//*************SERVO INITIALIZATION*************
boolean Reverse = false;
Servo myservo;
int minServo = 1120;
int maxServo = 1660;
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
boolean speedMode = true;

//*************FUNCTION INITIALIZATION*************
int   readRpm();                              //read RPM and reset counters
void  rpmIntHandler();                        //called by interupt, increments counters
static void smartDelay(unsigned long ms);     //Reads Data from GPS device
void  isr();                                  //encoder - Interrupt service routine is executed when a HIGH to LOW transition is detected on CLK
void  PIDCalculations();                      //Calulate Speed Adjustments based on MPH or RPM depending on mode
void  MainDisplay();                          //Runs the Main LCD screen
void  Menu();                                 //runs the USER menu options
int read_encoder();                          //Reads knob up/down/press/longpress for menu cases

//************************************************************************************************|
//*********** PERFORM STARTUP TASKS | DISPLAY STARTUP | GPS CONFIGURATION | READ MEMORY **********|
//************************************************************************************************|
void setup()
{
  //Serial.begin(9600);
  //delay (100);
  Serial1.begin(9600);   //GPS device
  delay (100);
  mydisp.begin();delay(3000); mydisp.clearScreen();  delay(100);
  mydisp.setFont(30);
  mydisp.setPrintPos(0, 1);
  mydisp.print("   ");  delay(100);  mydisp.print("STEADYPASS");delay(1000);
  mydisp.setPrintPos(0, 3);
  mydisp.print(" Version: ");
  mydisp.print(CurrentVersion);
  delay(5000);
  mydisp.clearScreen();
  delay(500);
  mydisp.setFont(30);
  mydisp.setPrintPos(0, 1);

  //***CHECKING FOR FIRST BOOT SETTINGS***
    if (EEPROM.read(69) == 69 ) {}
    else{
    mydisp.setFont(30);
    mydisp.setPrintPos(0, 1);
    mydisp.print("   FIRST BOOT ");
    mydisp.setPrintPos(0, 3);
    mydisp.print(" INITIALIZE MEM  ");
    delay(2000);
    EEPROM.write(11,6);
    EEPROM.write(12,0);
    EEPROM.write(13,25);
    EEPROM.write(14,0);
    EEPROM.write(15,25);
    EEPROM.write(16,0);
    EEPROM.write(17,50);
    EEPROM.write(18,1);
    EEPROM.write(19,12);
    EEPROM.write(20,255);
    EEPROM.write(21,0);
    EEPROM.write(29,1);
    EEPROM.write(30,10);
    EEPROM.write(31,0);
    EEPROM.write(69,69);
    mydisp.clearScreen();
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
  myservo.writeMicroseconds(pos);
  //PID Variables//
  Input = averagespeed100;
  Setpoint = Target100;
  //Turn on PID//
  myPID.SetMode(AUTOMATIC);
 //Average GPS speed vars
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
  readings[thisReading] = 0;
  }

  



  //******************LOADING SAVED SETTINGS*******************
    cylCoeff = EEPROM.read(11);
    aggKp = readWord(12) / 100.00;
    aggKi = readWord(14) / 100.00;
    aggKd = readWord(16) / 100.00;
    hourOffset = EEPROM.read(19);
    maxServo = 10 * EEPROM.read(20);
    minServo = 10 * EEPROM.read(21);
    mph = EEPROM.read(18);
    celsius = EEPROM.read(29);
    Target100 = EEPROM.read(30) * 100;
    Reverse = EEPROM.read(31);
  //----------------LOADED VALUES COMPLETE-------------------

  myPID.SetOutputLimits(minServo, maxServo);

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
  if (led) {digitalWrite(13, LOW); led = false;}
  else {digitalWrite(13, HIGH); led = true;}
  smartDelay(50);
  //delay(200);
  if (firstLoopOnStart) {mydisp.clearScreen();delay(100);}
  if (gps.speed.isValid()){speed100 = 100 * gps.speed.mph();}
  //speed100 = 100 * speedGpsD;
  //Average Speeds reading over last x cycles
  total = total - readings[readIndex];
  readings[readIndex] = speed100;
  total = total + readings[readIndex];
  readIndex = readIndex + 1;
  if (readIndex >= numReadings) {
    readIndex = 0;
  }
  // calculate the average:
  averagespeed100 = total / numReadings;

  //THROTTLE OVER LIMIT warning//
  if (pos == maxServo){
    if (throttleCheck < millis()){
      throttleCheck = millis() + 2000;
      mydisp.setFont(18);
      mydisp.setPrintPos(0, 0);
      mydisp.print(" ! DECREASE !     ");
      mydisp.setPrintPos(0, 1);
      mydisp.print(" ! THROTTLE !     ");
      smartDelay(50);
      //delay(200);
      if (gps.speed.isValid())
      {
        speed100 = 100 * gps.speed.mph();
      }
      //Speed100 = 100 * speedGpsD;
      mydisp.setPrintPos(0, 0);
      mydisp.print("                  ");
      mydisp.setPrintPos(0, 1);
      mydisp.print("                  ");
    }
  }
 
  //RUN CURRENT CALCULATION MODE//
  if (mode == 0) {pos = minServo;}
  else{
    if (mode == 1) {speedMode = false;}
    else {speedMode = true;}
    PIDCalculations();
  }
  if (Reverse) myservo.writeMicroseconds( minServo + maxServo - pos);
  else  myservo.writeMicroseconds(pos);

  //DISPLAY MAIN OUTPUT section
  TargetSpeedInt = Target100 / 10;
  speedValue = speed100 / 10; 
  if (!mph) speedValue *= 1.61;

  if (mainDisplay) MainDisplay();  //DISPLAY MAIN OUTPUT

  if (TurnDetected){
    if (mode == 2) Target100 += 10 * encoder;
    if (mode == 1) targetRPM += 10 * encoder;
    encoder = 0;
    TurnDetected = false; 
  }

  //menu enter
  if (digitalRead(PinSW) == LOW) {buttonTimes++;}
  else {
    if (buttonTimes > 0){ 
      mode++;  
      if (mode > 2) mode = 0;
    }
    buttonTimes = 0;
  }

  if (buttonTimes > buttonTarget){
    mydisp.clearScreen();
    mydisp.setFont(10);
    mydisp.setPrintPos(0, 0);
    delay(500);
  }
  while (buttonTimes > buttonTarget) {mydisp.print("Main Menu"); delay(500); Menu();}
  if (firstLoopOnStart) {firstLoopOnStart = false;}
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
  //PID INPUTS//

  if (speedMode){
    Input = speed100;
    Setpoint = Target100;
  }
  else{
    Input = readRpm();
    Setpoint = targetRPM;
  }
//    Serial.print(Input);
//    Serial.print("<speed|");
//    Serial.print(Setpoint);
//    Serial.print("<target|");
    double gap = abs(Setpoint - Input);
    gap = gap / 10;
//    Serial.print(gap);
//    Serial.print("<gap|");
    if (gap < 50)
    {
      myPID.SetTunings(aggKp, aggKi, aggKd);
    }
    else
    {
      myPID.SetTunings(aggKp, aggKi, aggKd);
    }
    myPID.Compute();
//     Serial.print(Output);
//     Serial.print("<ServoPos|");
    pos = Output;
}
//End Function

//*******************************************************************
/* FUNCTION:  MainDisplay()
   DESCTRIPTION: This displays everything that you see
*/
void MainDisplay()
{
  //GPS Time Display//
  hours = gps.time.hour() + hourOffset - 12;
  if (hours < 0) hours += 24;
  if (hours > 24) hours -= 24;
  minutes = gps.time.minute();
  seconds = gps.time.second();
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
  mydisp.print("POWER ");
  if (delayCheck < millis()){
    delayCheck = millis() + 500;
    throttlePer = maxServo - pos;
    throttlePer = throttlePer / (maxServo - minServo);
    throttlePer *= 100;
    //roundedThrottlePer = (int)(throttlePer);
    mydisp.print((int)(throttlePer)); mydisp.print("  ");
  }
  
  //Print word depending on mode selection//
  mydisp.setFont(18);
  mydisp.setPrintPos(6, 0);
  if (mode == 0) mydisp.print("  OFF ");
  if (mode == 1) {
    mydisp.print(targetRPM);
    mydisp.print("RPM ");
  }
  if (mode == 2) {
    printTargetSpeed();
  }

  //Prints Voltage
  mydisp.setFont(10);
  mydisp.setPrintPos(0, 2);
  mydisp.print("V ");  mydisp.print(voltage); mydisp.print("  ");

  // Prints Water Temp and Air Temp
  mydisp.setPrintPos(0, 4);
  mydisp.print("WTR "); mydisp.print(wtrTemp); if (celsius)  mydisp.print("c  "); else mydisp.print("F ");
  mydisp.setPrintPos(0, 5);
  mydisp.print("AIR "); mydisp.print(airTemp); if (celsius)  mydisp.print("c  "); else mydisp.print("F ");

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

void Menu(){
  while (digitalRead(PinSW) == LOW){}
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
    "Min Throttle", 
    "Max Throttle", 
    "Test Servo",
    "Servo Mode",
    "Engine Cylinders", 
    "Time Zone Offset",
    "Startup Target Speed",
    "Unit of Measure",
    "Temp Sensor",
    "Proportional Gain",
    "Integral Gain",
    "Derivative Gain",
     ""
  };
  while (menuItems[totalMenuItems] != ""){
    totalMenuItems++;}
  totalMenuItems--;
  mydisp.clearScreen();
  stillSelecting = true;
  do{
    switch(read_encoder()) {  
      case 1:  // ENCODER UP
        if(cursorPosition == 0 && topItemDisplayed > 0){
          topItemDisplayed--;
          redraw = MOVELIST;
        }
        if(cursorPosition>0){
          cursorPosition--;
          redraw = MOVECURSOR;
        }
      break;

      case 2:    //ENCODER DOWN
        if((topItemDisplayed + (totalRows-1)) < totalMenuItems && cursorPosition == (totalRows-1)){
          topItemDisplayed++;
          redraw = MOVELIST;
        }
        if(cursorPosition<(totalRows-1)){
          cursorPosition++;
          redraw = MOVECURSOR;
        }
      break;

      case 4:  // ENCODER BUTTON SHORT PRESS
        switch(topItemDisplayed + cursorPosition){
          case 0:
            minThrottleMenu();
            redraw = MOVELIST;
          break;
          case 1:  // menu item 2 selected
            maxThrottleMenu();
            redraw = MOVELIST;
          break;
          case 2:  // menu item 3 selected
            testThrottleMenu();
            redraw = MOVELIST;
          break;
          case 3:  // menu item 4 selected
            reverseServoModeMenu();
            redraw = MOVELIST;
          break;
          case 4:  // menu item 5 selected
          cylinderCountMenu();
            redraw = MOVELIST;
          break;
          case 5:  // menu item 6 selected
            timeZoneMenu();
            redraw = MOVELIST;
          break;
          case 6:  // menu item 7 selected
            defaultTargetSpeedMenu();
            redraw = MOVELIST;
          break;
          case 7:  // menu item 8 selected
            unitMeasureMenu();
            redraw = MOVELIST;
          break;
          case 8:  // menu item 9 selected
            tempUnitMenu();
          redraw = MOVELIST;
          break;
          case 9:  // menu item 10 selected
            PIDKpmenu();
            redraw = MOVELIST;
          break;
          case 10:  // menu item 11 selected
            PIDKimenu();
            redraw = MOVELIST;
          break;
          case 11:  // menu item 12 selected
            PIDKdmenu();
            redraw = MOVELIST;
          break;
        }
      break;
        
      case 8:  //LONG PRESS
        returnToMainDisp();
      break;

      case 16:
      break;
    }

    switch(redraw){
      case MOVECURSOR:
        redraw = false;
        if (cursorPosition > totalMenuItems) cursorPosition = totalMenuItems;
        for(i = 0; i < (totalRows); i++){
          mydisp.setFont(10);
          mydisp.setPrintPos(0, i);
          mydisp.print(" ");
          mydisp.setPrintPos((totalCols-1), i);
          mydisp.print(" ");
        }
        mydisp.setFont(10);
        mydisp.setPrintPos(0,cursorPosition);
        mydisp.print(">");
      break;

      case MOVELIST:
        redraw=MOVECURSOR;
        mydisp.clearScreen();
        if(totalMenuItems>((totalRows-1))){
          for (i = 0; i < (totalRows); i++){
            mydisp.setFont(10);
            mydisp.setPrintPos(1,i);
            mydisp.print(menuItems[topItemDisplayed + i]);
          }
        }
        else{
          for (i = 0; i < totalMenuItems+1; i++){
            mydisp.setFont(10);
            mydisp.setPrintPos(1,i);
            mydisp.print(menuItems[topItemDisplayed + i]);
          }
        }
      break;
    }
  } 
  while (stillSelecting == true);
}



int read_encoder(){
  #define btnUp         1
  #define btnDown       2
  #define btnPress      4
  #define btnLongPress  8
  #define btnNull       16
  
  if (TurnDetected){
    if (encoder > 0) {
      encoder = 0;
      TurnDetected = false;
      return btnUp;
    }
    else if (encoder < 0){
      encoder = 0;
      TurnDetected = false;
      return btnDown;
    }
    else { 
      TurnDetected = false;
      return btnNull;}
  }
  else{
    if (digitalRead(PinSW) == LOW)
      {
         inMenuPress++;
          delay(200);
      }
    else if (inMenuPress > 0) {
        inMenuPress = 0;
          mydisp.clearScreen();
          mydisp.setFont(20);
          mydisp.setPrintPos(0,0);
          mydisp.print("ENTER");
          delay(500);
          mydisp.clearScreen();
          return btnPress;
      }
    else { return btnNull; }
    
    if (inMenuPress > buttonTarget){
        inMenuPress = 0;
          mydisp.clearScreen();
          mydisp.setFont(20);
          mydisp.setPrintPos(0,0);
          mydisp.print("BACK");
        while (digitalRead(PinSW) == LOW){};
          mydisp.clearScreen();
        return btnLongPress;
    }
  }
}

void minThrottleMenu(){
  boolean stillSelecting = true;
  pos = maxServo;
  mydisp.clearScreen();
  mydisp.setFont(10);
  mydisp.setPrintPos(0, 0);
  mydisp.print("Min Throttle ");  
  mydisp.setPrintPos(0, 1);
  mydisp.print("Servo high position");  
  mydisp.setPrintPos(0, 2); 
  mydisp.print("Gives max cable slack");
  mydisp.setPrintPos(0, 4);
  mydisp.print("Position: ");
  if (Reverse) mydisp.print(minServo); else mydisp.print(maxServo);  mydisp.print("    ");
 do{  
    myservo.writeMicroseconds(pos); 
    switch(read_encoder())
    {
      case 1:  // ENCODER UP
        if (Reverse) {
          if (minServo >= maxServo){minServo = maxServo - 10;}
          else {minServo +=10;}
        } 
        else {maxServo +=10;}
        mydisp.setPrintPos(0, 4);
        mydisp.print("Position: ");
        if (Reverse) mydisp.print(minServo); else mydisp.print(maxServo);  mydisp.print("    ");
        if (Reverse) pos = minServo; else pos = maxServo;
      break;

      case 2:    //ENCODER DOWN
        if(!Reverse){
          if (maxServo <= minServo){maxServo = minServo + 10;}
          else {maxServo -=10;}
        }
        else {minServo -=10;}
        mydisp.setPrintPos(0, 4);
        mydisp.print("Position: ");
        if (Reverse) mydisp.print(minServo); else mydisp.print(maxServo);  mydisp.print("    ");
        if (Reverse) pos = minServo; else pos = maxServo;
      break;

      case 4:  // ENCODER BUTTON SHORT PRESS
        stillSelecting = false;
         myPID.SetOutputLimits(minServo, maxServo);
        if (Reverse) EEPROM.write(21,minServo /10); else EEPROM.write(20,maxServo /10);
        break;

      case 8:  // ENCODER BUTTON LONG PRESS
        returnToMainDisp();
        break;

      case 16:  // ENCODER BUTTON NULL
        break;

    }
  }
   while (stillSelecting == true);
}

void maxThrottleMenu(){
  boolean stillSelecting = true;
  pos = minServo;
  mydisp.clearScreen();
  mydisp.setFont(10);
  mydisp.setPrintPos(0, 0);
  mydisp.print("Max Throttle ");  
  mydisp.setPrintPos(0, 1);
  mydisp.print("Servo low position");
  mydisp.setPrintPos(0, 2); 
  mydisp.print("Gives NO cable slack");
  mydisp.setPrintPos(0, 4);
  mydisp.print("Position: ");
  if (Reverse) mydisp.print(maxServo); else mydisp.print(minServo);  mydisp.print("    ");
 do{
    myservo.writeMicroseconds(pos);
    switch(read_encoder())
    {
      case 1:  // ENCODER UP
        if(!Reverse){
          if (minServo >= maxServo){minServo = maxServo - 10;}
          else {minServo +=10;}
        }
        else {maxServo +=10;}
        mydisp.setPrintPos(0, 4);
        mydisp.print("Position: ");
        if (Reverse) mydisp.print(maxServo); else mydisp.print(minServo);  mydisp.print("    ");
        if (Reverse) pos = maxServo; else pos = minServo;
        break;

      case 2:    //ENCODER DOWN
        if (Reverse) {
          if (maxServo <= minServo){maxServo = minServo + 10;}
          else {maxServo -=10;}
        } 
        else {minServo -=10;}
        mydisp.setPrintPos(0, 4);
        mydisp.print("Position: ");
        if (Reverse) mydisp.print(maxServo); else mydisp.print(minServo);  mydisp.print("    ");
        if (Reverse) pos = maxServo; else pos = minServo;
        break;

      case 4:  // ENCODER BUTTON SHORT PRESS
        stillSelecting = false;
        if (Reverse) EEPROM.write(20,maxServo /10); else EEPROM.write(21,minServo /10);
         myPID.SetOutputLimits(minServo, maxServo);
        break;

      case 8:  // ENCODER BUTTON LONG PRESS
        returnToMainDisp();
        break;

      case 16:  // ENCODER BUTTON NULL
        break;

    }
  }
   while (stillSelecting == true);
}

void testThrottleMenu(){
  boolean stillSelecting = true;
  pos = minServo;
  mydisp.clearScreen();
  mydisp.setFont(10);
  mydisp.setPrintPos(0, 0);
  mydisp.print("Test Servo Range ");  
 do{
    mydisp.setPrintPos(0, 1);
    mydisp.print("("); if (Reverse) mydisp.print(minServo + maxServo - pos); else mydisp.print(pos); mydisp.print(")     ");
    delay(100);
    if (pos <= minServo) S = 1;
    if (pos >= maxServo) S = -1;
    pos += 10 * S;
    if (Reverse) myservo.writeMicroseconds( minServo + maxServo - pos);
    else  myservo.writeMicroseconds(pos);

    switch(read_encoder())
    {
      case 1:  // ENCODER UP

        break;
      case 2:    //ENCODER DOWN

        break;

      case 4:  // ENCODER BUTTON SHORT PRESS

        stillSelecting = false;
        break;

      case 8:  // ENCODER BUTTON LONG PRESS
        returnToMainDisp();
        break;

      case 16:  // ENCODER BUTTON NULL
        break;

    }
  }
   while (stillSelecting == true);
}

void reverseServoModeMenu(){
  boolean stillSelecting = true;
  mydisp.clearScreen();
  mydisp.setFont(10);
  mydisp.setPrintPos(0, 0);
  mydisp.print("Servo Mode");  
  mydisp.setPrintPos(0, 1);
  if (Reverse) mydisp.print("Reverse Mode");
  if (!Reverse) mydisp.print("Normal Mode");

 do{
    switch(read_encoder())
    {
      case 1:  // ENCODER UP
        mydisp.setPrintPos(0, 1);
        if (Reverse){
          mydisp.print("Normal Mode  ");
          Reverse = false;}
        else {
          mydisp.print("Reverse Mode");
          Reverse = true;}
        break;

      case 2:    //ENCODER DOWN
        mydisp.setPrintPos(0, 1);
        if (Reverse){
          mydisp.print("Normal Mode");
          Reverse = false;}
        else {
          mydisp.print("Reverse Mode");
          Reverse = true;}
        break;

      case 4:  // ENCODER BUTTON SHORT PRESS
        stillSelecting = false;
        if (Reverse) EEPROM.write(31,1); else EEPROM.write(31,0);
        break;

      case 8:  // ENCODER BUTTON LONG PRESS
        returnToMainDisp();        
        break;

      case 16:  // ENCODER BUTTON NULL
        break;
    }
  }
   while (stillSelecting == true);
}

void cylinderCountMenu(){
  boolean stillSelecting = true;
  mydisp.clearScreen();
  mydisp.setFont(10);
  mydisp.setPrintPos(0, 0);
  mydisp.print("Engine Cylinders ");  
  mydisp.setPrintPos(0, 1);
  mydisp.print(cylCoeff);
 do{

    switch(read_encoder())
    {
      case 1:  // ENCODER UP
        mydisp.setPrintPos(0, 1);
        if (cylCoeff < 8){
          cylCoeff = cylCoeff + 2;
          mydisp.print(cylCoeff);
          mydisp.setPrintPos(0, 2);
          mydisp.print("                ");
        }
        else {
          cylCoeff = 8;
          mydisp.print(cylCoeff);
          mydisp.setPrintPos(0, 2);
          mydisp.print("Max Cylinders");
        }
        break;

      case 2:    //ENCODER DOWN
        mydisp.setPrintPos(0, 1);
        if (cylCoeff > 4){
          cylCoeff = cylCoeff - 2;
          mydisp.print(cylCoeff);
          mydisp.setPrintPos(0, 2);
          mydisp.print("                ");
        }
        else {
          cylCoeff = 4;
          mydisp.print(cylCoeff);
          mydisp.setPrintPos(0, 2);
          mydisp.print("Min Cylinders");}
        break;

      case 4:  // ENCODER BUTTON SHORT PRESS
        stillSelecting = false;
        EEPROM.write(11,cylCoeff);
        break;

      case 8:  // ENCODER BUTTON LONG PRESS
        returnToMainDisp();       
        break;

      case 16:  // ENCODER BUTTON NULL
        break;

    }
  }
   while (stillSelecting == true);
}

void timeZoneMenu(){
  boolean stillSelecting = true;
  mydisp.clearScreen();
  mydisp.setFont(10);
  mydisp.setPrintPos(0, 0);
  mydisp.print("Time Zone Offset ");
  mydisp.setPrintPos(0, 1);
  mydisp.print(hours); mydisp.print(":"); if (minutes < 10) mydisp.print("0"); mydisp.print(minutes); mydisp.print(":"); if (seconds < 10) mydisp.print("0"); mydisp.print(seconds); mydisp.print("   ");
  mydisp.setPrintPos(0, 3);
  mydisp.print(hourOffset - 12); mydisp.print("   ");
  do{  
    switch(read_encoder())
    {
      case 1:  // ENCODER UP
        if (hourOffset < 24){hourOffset += 1;}
        else {hourOffset = 24;}
        hours = gps.time.hour(); + hourOffset - 12;
        if (hours < 0) hours += 24;
        if (hours > 24) hours -= 24;
        mydisp.setPrintPos(0, 1);
        mydisp.print(hours);
        mydisp.setPrintPos(0, 3);
        mydisp.print(hourOffset - 12); mydisp.print("   ");
        
      break;

      case 2:  //ENCODER DOWN
        if (hourOffset > 0){hourOffset -= 1;}
        else {hourOffset = 0;}
        hours = gps.time.hour(); + hourOffset - 12;
        if (hours < 0) hours += 24;
        if (hours > 24) hours -= 24;
        mydisp.setPrintPos(0, 1);
        mydisp.print(hours);
        mydisp.setPrintPos(0, 3);
        mydisp.print(hourOffset - 12); mydisp.print("   ");
      break;

      case 4:  // ENCODER BUTTON SHORT PRESS
        stillSelecting = false;
        EEPROM.write(19,hourOffset);
        break;

      case 8:  // ENCODER BUTTON LONG PRESS
        returnToMainDisp();       
        break;

      case 16:  // ENCODER BUTTON NULL
        break;

      if (hours < 0) hours += 24;
      if (hours > 24) hours -= 24;
      mydisp.print(hours); mydisp.print(":"); if (minutes < 10) mydisp.print("0"); mydisp.print(minutes); mydisp.print(":"); if (seconds < 10) mydisp.print("0"); mydisp.print(seconds); mydisp.print("            ");
    }
  }
   while (stillSelecting == true);
}

void defaultTargetSpeedMenu(){
  boolean stillSelecting = true;
  mydisp.clearScreen();
  mydisp.setFont(10);
  mydisp.setPrintPos(0, 0);
  mydisp.print("Startup Target Speed");  
  mydisp.setPrintPos(0, 1);
  printTargetSpeed();
 do{
    switch(read_encoder())
    {
      case 1:  // ENCODER UP
        Target100 = Target100 + 10;
        mydisp.setPrintPos(0, 1);
        printTargetSpeed();
        break;

      case 2:    //ENCODER DOWN
        Target100 = Target100 - 10;
        mydisp.setPrintPos(0, 1);
        printTargetSpeed();
        break;

      case 4:  // ENCODER BUTTON SHORT PRESS
        stillSelecting = false;
        EEPROM.write(30,Target100 /100);
        break;

      case 8:  // ENCODER BUTTON LONG PRESS
        returnToMainDisp();       
        break;

      case 16:  // ENCODER BUTTON NULL
        break;

    }
  }
   while (stillSelecting == true);
}
   
void unitMeasureMenu(){
  boolean stillSelecting = true;
  mydisp.clearScreen();
  mydisp.setFont(10);
  mydisp.setPrintPos(0, 0);
  mydisp.print("Unit of Measure");  
  mydisp.setPrintPos(0, 1);
  if (mph) mydisp.print("MPH "); else mydisp.print("KPH ");
 do{
    switch(read_encoder())
    {
      case 1:  // ENCODER UP
        mph = !mph;
        mydisp.setPrintPos(0, 1);
        if (mph) mydisp.print("MPH "); else mydisp.print("KPH ");
      break;

      case 2:    //ENCODER DOWN
        mph = !mph;
        mydisp.setPrintPos(0, 1);
        if (mph) mydisp.print("MPH "); else mydisp.print("KPH ");
      break;

      case 4:  // ENCODER BUTTON SHORT PRESS
        stillSelecting = false;
        if (mph) EEPROM.write(18,1); else EEPROM.write(18,0);
      break;

      case 8:  // ENCODER BUTTON LONG PRESS
        returnToMainDisp();       
      break;

      case 16:  // ENCODER BUTTON NULL
      break;

    }
  }
   while (stillSelecting == true);
}

void tempUnitMenu(){
  boolean stillSelecting = true;
  mydisp.clearScreen();
  mydisp.setFont(10);
  mydisp.setPrintPos(0, 0);
  mydisp.print("Tempturature Mode ");  
  mydisp.setPrintPos(0, 1);
  if (celsius) mydisp.print("Celsius     "); else mydisp.print("Farenheit ");
 do{
    switch(read_encoder())
     {
      case 1:  // ENCODER UP
        celsius = !celsius;
        mydisp.setPrintPos(0, 1);
        if (celsius) mydisp.print("Celsius    "); else mydisp.print("Farenheit ");
      break;

      case 2:    //ENCODER DOWN
        celsius = !celsius;
        mydisp.setPrintPos(0, 1);
        if (celsius) mydisp.print("Celsius    "); else mydisp.print("Farenheit ");
      break;

      case 4:  // ENCODER BUTTON SHORT PRESS
        stillSelecting = false;
        if (celsius) EEPROM.write(29,1); else EEPROM.write(29,0);
      break;

      case 8:  // ENCODER BUTTON LONG PRESS
        returnToMainDisp(); 
      break;

      case 16:  // ENCODER BUTTON NULL
      break;
    }
  }
   while (stillSelecting == true);
}

void PIDKpmenu(){
  boolean stillSelecting = true;
  mydisp.clearScreen();
  mydisp.setFont(10);
  mydisp.setPrintPos(0, 0);
  mydisp.print("Proportional Gain"); 
  mydisp.setPrintPos(0, 1); 
  mydisp.print("Default rate = .25");  
  mydisp.setPrintPos(0, 3);
  mydisp.print("Rate: ");
  mydisp.print(aggKp);
  int Kp100 = aggKp * 100;
 do{
    switch(read_encoder())
     {
     case 1:  // ENCODER UP
        mydisp.setPrintPos(0, 3);
        mydisp.print("Rate: ");
        aggKp = aggKp * 100;
        Kp100 = aggKp;
        aggKp = aggKp + 1;
        aggKp /= 100;
        mydisp.print(aggKp);mydisp.print("  ");
      break;

      case 2:    //ENCODER DOWN
        mydisp.setPrintPos(0, 3);
        mydisp.print("Rate: ");
        aggKp = aggKp * 100;
        Kp100 = aggKp;
        aggKp = aggKp - 1;
        aggKp /= 100;
        mydisp.print(aggKp);mydisp.print("  ");
       break;

      case 4:  // ENCODER BUTTON SHORT PRESS
        stillSelecting = false;
        writeWord(12,Kp100);
      break;

      case 8:  // ENCODER BUTTON LONG PRESS
        returnToMainDisp(); 
      break;

      case 16:  // ENCODER BUTTON NULL
      break;

    }
  }
   while (stillSelecting == true);
}

void PIDKimenu(){
  boolean stillSelecting = true;
  mydisp.clearScreen();
  mydisp.setFont(10);
  mydisp.setPrintPos(0, 0);
  mydisp.print("Integral Gain");
  mydisp.setPrintPos(0, 1);
  mydisp.print("default = .25");
  mydisp.setPrintPos(0, 3);
  mydisp.print("Speed: ");
  mydisp.print(aggKi);
  int Ki100 = aggKi * 100;
 do{
    switch(read_encoder())
     {
     case 1:  // ENCODER UP
        mydisp.setPrintPos(0, 3);
        mydisp.print("Speed: ");
        aggKi = aggKi * 100;
        Ki100 = aggKi;
        aggKi = aggKi + 1;
        aggKi = aggKi / 100;
        mydisp.print(aggKi);mydisp.print("   ");
      break;

      case 2:    //ENCODER DOWN
        mydisp.setPrintPos(0, 3);
        mydisp.print("Speed: ");
        aggKi = aggKi * 100;
        Ki100 = aggKi;
        aggKi = aggKi - 1;
        aggKi = aggKi / 100;
        mydisp.print(aggKi);mydisp.print("     ");
       break;

      case 4:  // ENCODER BUTTON SHORT PRESS
        stillSelecting = false;
        writeWord(14,Ki100);
       break;

      case 8:  // ENCODER BUTTON LONG PRESS
        returnToMainDisp();
      break;

      case 16:  // ENCODER BUTTON NULL
       break;

    }
  }
   while (stillSelecting == true);
}

void PIDKdmenu(){
  boolean stillSelecting = true;
  mydisp.clearScreen();
  mydisp.setFont(10);
  mydisp.setPrintPos(0, 0);
  mydisp.print("Derivative Gain");  
  mydisp.setPrintPos(0, 1);
  mydisp.print("Default = .50");
  mydisp.setPrintPos(0, 3);
  mydisp.print("Rate: ");
  mydisp.print(aggKd);
  int Kd100 = aggKd * 100;
 do{
    switch(read_encoder())
     {
     case 1:  // ENCODER UP
        mydisp.setPrintPos(0, 3);
        mydisp.print("Rate: ");
        aggKd = aggKd * 100;
        Kd100 = aggKd;
        aggKd = aggKd + 1;
        aggKd /= 100;
        mydisp.print(aggKd);mydisp.print("  ");
      break;

      case 2:    //ENCODER DOWN
        mydisp.setPrintPos(0, 3);
        mydisp.print("Rate: ");
        aggKd = aggKd * 100;
        Kd100 = aggKd;
        aggKd = aggKd - 1;
        aggKd /= 100;
        mydisp.print(aggKd);mydisp.print("  ");
       break;

      case 4:  // ENCODER BUTTON SHORT PRESS
        stillSelecting = false;
        writeWord(16,Kd100);
      break;

      case 8:  // ENCODER BUTTON LONG PRESS
        returnToMainDisp();
      break;

      case 16:  // ENCODER BUTTON NULL
      break;
    }
  }
   while (stillSelecting == true);
}


void printTargetSpeed()
{
  TargetSpeedInt = Target100 / 10;
  targetSpeedWhole = TargetSpeedInt / 10;
  targetSpeedDecimal = TargetSpeedInt %10;
  int c = targetSpeedDecimal + targetSpeedWhole;
  if (c < 0) mydisp.print ("-");
  else mydisp.print (" ");
  if (targetSpeedWhole < 0) targetSpeedWhole = -targetSpeedWhole;
  mydisp.print (targetSpeedWhole);
  mydisp.print (".");
  if (targetSpeedDecimal<0)targetSpeedDecimal = -targetSpeedDecimal;
  mydisp.print (targetSpeedDecimal);
  if (mph) mydisp.print("MPH "); else mydisp.print("KPH ");
}

void returnToMainDisp()
{
  stillSelecting = false;
  buttonTimes = 0;
  firstMainDispLoop = true;
  loop ();

}


void writeWord(unsigned address, unsigned value)
  {
  EEPROM.write(address, highByte(value));
  EEPROM.write(address+1, lowByte(value));
  }

unsigned readWord(unsigned address)
   {
   return word(EEPROM.read(address), EEPROM.read(address+1));
   }
