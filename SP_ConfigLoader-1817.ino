/*
  _________ __                     .___       __________                        ____   ________
  /   _____//  |_  ____ _____     __| _/__.__. \______   \_____    ______ ______ \   \ /   /_   |
  \_____  \\   __\/ __ \\__  \   / __ <   |  |  |     ___/\__  \  /  ___//  ___/  \       / |   |
  /        \|  | \  ___/ / __ \_/ /_/ |\___  |  |    |     / __ \_\___ \ \___ \    \     /  |   |
  /_______  /|__|  \___  >____  /\____ |/ ____|  |____|    /____  /____  >____  >    \___/   |___|
  ================================================================================================
************************************************************************************************
*/
//******LIBRARYS******
#include <TinyGPS++.h>
#include <Servo.h>
#include <SPI.h>
#include <OneWire.h>
#include <EEPROM.h>
#define _Digole_Serial_SPI_
#include <DigoleSerial.h>
//---------------------

//*************INITIALIZING DEFINITIONS*************
boolean stillSelecting = true;
double aggKp = 0.37, aggKi = 0.25 , aggKd = 0.08; //NEED TO MAKE MENU OPTION FOR ADJUSTMENTS ON THE FLY
#define MOVECURSOR 1  // constants for indicating whether cursor should be redrawn
#define MOVELIST 2  // constants for indicating whether cursor should be redrawn
byte totalRows = 6;  // total rows of LCD
byte totalCols = 1;  // total columns of LCD
TinyGPSPlus gps;                                //required for TinyGPSplus Library
DigoleSerialDisp mydisp(9, 8, 10);              //Pin Config SPI | 9: data | 8:clock | 10: SS | you can assign 255 to SS, and hard ground SS pin on module
boolean led = false;                            //LED on | off
byte cylCoeff = 6;                               //Number of Cylinders
const byte PinCLK = 2;             //encoder second pin
const byte PinDT = 3;              // Used for reading DT signal of encoder
const byte PinSW = 4;              //encoder button
const byte servoPin = 5;           //servo PWM connected here
byte i;
boolean celsius = true;
boolean mph = true;
int Contrast;
int targetRPM = 3000;
boolean Reverse = false;
Servo myservo;
int minServo = 950;
int maxServo = 1950;
int pos = minServo;
int S = 10;
volatile boolean TurnDetected = false;
volatile boolean up = false;
byte buttonTimes = 0;
byte inMenuPress = 0;
byte buttonTarget = 7;
volatile long encoder = 0;
int hours, minutes, seconds, hourOffset = 12;
static void smartDelay(unsigned long ms);     //Reads Data from GPS device
void  isr();                                  //encoder - Interrupt service routine is executed when a HIGH to LOW transition is detected on CLK
void  Menu();                                 //runs the USER menu options
int read_encoder();                          //Reads knob up/down/press/longpress for menu cases
int TargetSpeedInt, Target100 = 500;
int targetSpeedWhole = TargetSpeedInt / 10;
int targetSpeedDecimal = TargetSpeedInt %10;
double accelDampKd = .08;
int gpsDegree,gpsHDOP,gpsSats,gpsLat,gpsLng,gpsAlt;
char gpsCourse[2];
int speed100;
//************************************************************************************************|
//*********** PERFORM STARTUP TASKS | DISPLAY STARTUP | GPS CONFIGURATION | READ MEMORY **********|
//************************************************************************************************|
void setup()
{
  Serial1.begin(9600);
  delay (3000);
  mydisp.begin();
  mydisp.clearScreen();  delay(100);
  mydisp.setFont(30);
  mydisp.setPrintPos(0, 1);
  mydisp.print("   ");  delay(100);  mydisp.print("STEADYPASS");
  mydisp.setPrintPos(0, 2);
  mydisp.print("   ");  delay(100);  mydisp.print("INITIALIZE ");
  mydisp.setPrintPos(0, 3);
  mydisp.setFont(10);
  mydisp.print(" wait for GPS signal ");
  delay(5000);
  mydisp.clearScreen();
  delay(500);
  mydisp.setFont(30);
  mydisp.setPrintPos(0, 1);

  //  ***CHECKING FOR FIRST BOOT SETTINGS***
  //18.16 using reset code 1
  if (EEPROM.read(200) == 1 ) {}
  else {
    mydisp.setFont(30);
    mydisp.setPrintPos(0, 2);
    mydisp.print(" INITIALIZE MEM  ");
    mydisp.print("  FOR V 18.16  ");
    delay(2000);
    EEPROM.write(11, 6);
    EEPROM.write(12, 0);
    EEPROM.write(13, 37);
    EEPROM.write(14, 0);
    EEPROM.write(15, 25);
    EEPROM.write(16, 0);
    EEPROM.write(17, 8);
    EEPROM.write(18, 1);
    EEPROM.write(19, 12);
    EEPROM.write(20, 195);
    EEPROM.write(21, 95);
    EEPROM.write(22, 0);
    EEPROM.write(23, 8);
    EEPROM.write(24, 0);
    EEPROM.write(25, 0);
    EEPROM.write(29, 1);
    EEPROM.write(30, 10);
    EEPROM.write(31, 1);
    EEPROM.write(32, 0);
    EEPROM.write(33, 30);
    EEPROM.write(200, 1);
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


  //******************LOADING SAVED SETTINGS*******************
  cylCoeff = EEPROM.read(11);
  aggKp = readWord(12) / 100.00;
  // 13 taken
  aggKi = readWord(14) / 100.00;
  // 15 taken
  aggKd = readWord(16) / 100.00;
  // 17 taken
  mph = EEPROM.read(18);
  hourOffset = EEPROM.read(19);
  maxServo = 10 * EEPROM.read(20);
  minServo = 10 * EEPROM.read(21);
  celsius = EEPROM.read(29);
  Target100 = readWord(30);
  Reverse = EEPROM.read(32);
  Contrast = EEPROM.read(33);
  //----------------LOADED VALUES COMPLETE-------------------

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
  mydisp.setContrast(Contrast);
  smartDelay(50);
  //delay(200);
  if (gps.speed.isValid()){speed100 = 100 * gps.speed.mph();}
  if (gps.course.isValid()){gpsDegree = gps.course.deg();}
  if (gps.hdop.isValid()){gpsHDOP = gps.hdop.hdop();} 
  if (gps.location.isValid()){gpsLat = gps.location.lat();}
  if (gps.location.isValid()){gpsLng = gps.location.lng();}
  if (gps.satellites.isValid()){gpsSats = gps.satellites.value();}
  if (gps.altitude.isValid()){gpsAlt = gps.altitude.meters();}


    if (gpsDegree  > 337){gpsCourse[9] = "North";} 
    if (gpsDegree <=  22){gpsCourse[9] = "North";}
    if (gpsDegree  > 22){
      if (gpsDegree <=  67){gpsCourse[9] = "NorthEast";}
    }
    if (gpsDegree  > 67){
      if (gpsDegree <= 112){gpsCourse[9] = "East";}
    }
    if (gpsDegree  > 112){
      if (gpsDegree <= 157){gpsCourse[9] = "SouthEast";}
    }
    if (gpsDegree  > 157){
      if (gpsDegree <= 202){gpsCourse[9] = "South";}
    }
    if (gpsDegree  > 202){
      if (gpsDegree <= 247){gpsCourse[9] = "SouthWest";}
    }
    if (gpsDegree  > 247){
      if (gpsDegree <= 292){gpsCourse[9] = "West";}
    }
    if (gpsDegree  > 292){
      if (gpsDegree <= 337){gpsCourse[9] = "NorthWest";}
      }
  Serial.print("speed:");
    Serial.print(speed100);
    Serial.print(" ||||| ");
    Serial.print("HDOP:");
    Serial.print(gpsHDOP);
    Serial.print(" ||||| ");
    Serial.print("Lat coord:");
    Serial.print(gpsLat);
    Serial.print(" ||||| ");
    Serial.print("Lng coord:");
    Serial.print(gpsLng);
    Serial.print(" ||||| ");
    Serial.print("Sat #:");
    Serial.print(gpsSats);
    Serial.print(" ||||| ");
    Serial.print("Alt:");
    Serial.print(gpsAlt);
    Serial.print(" ||||| ");
    Serial.print("course:");
    Serial.print(gpsCourse);
    Serial.print(" ||||| ");
    Serial.print("degree:");
    Serial.print(gpsDegree);
    Serial.print(" +++++ ");
  Menu();
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
  while ( (!gps.time.isUpdated())  );
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
/* FUNCTION: Menu()
   DESCTRIPTION: Displays the main scrolling menu
*/
void Menu() {
  while (digitalRead(PinSW) == LOW) {}
  byte topItemDisplayed = 0;  // stores menu item displayed at top of LCD screen
  byte cursorPosition = 0;  // where cursor is on screen, from 0 --> totalRows.
  // redraw = 0  - don't redraw
  // redraw = 1 - redraw cursor
  // redraw = 2 - redraw list
  byte redraw = MOVELIST;  // triggers whether menu is redrawn after cursor move.
  byte i = 0; // temp variable for loops.
  byte totalMenuItems = 0;  //a while loop below will set this to the # of menu items.

  // Put the menu items here. Remember, the first item will have a 'position' of 0.
  char* menuItems[] = {
    "Min Throttle",
    "Max Throttle",
    "Test Servo",
    "Servo Mode",
    "Engine Cylinders",
    "Time Zone Offset",
    "Startup Target Speed",
    "Unit of Measure",
    "Temp Sensor",
    "Contrast",
    "Proportional Gain",
    "Integral Gain",
    "Derivative Gain",
//    "Accel Dampener",
    ""
  };
  while (menuItems[totalMenuItems] != "") {
    totalMenuItems++;
  }
  totalMenuItems--;
  mydisp.clearScreen();
  stillSelecting = true;
  do {
    switch (read_encoder()) {
      case 1:  // ENCODER UP
        if (cursorPosition == 0 && topItemDisplayed > 0) {
          topItemDisplayed--;
          redraw = MOVELIST;
        }
        if (cursorPosition > 0) {
          cursorPosition--;
          redraw = MOVECURSOR;
        }
        break;

      case 2:    //ENCODER DOWN
        if ((topItemDisplayed + (totalRows - 1)) < totalMenuItems && cursorPosition == (totalRows - 1)) {
          topItemDisplayed++;
          redraw = MOVELIST;
        }
        if (cursorPosition < (totalRows - 1)) {
          cursorPosition++;
          redraw = MOVECURSOR;
        }
        break;

      case 4:  // ENCODER BUTTON SHORT PRESS
        switch (topItemDisplayed + cursorPosition) {
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
          case 9:  // menu item 11 selected
            contrastMenu();
            redraw = MOVELIST;
            break;
          case 10:  // menu item 12 selected
            PIDKpmenu();
            redraw = MOVELIST;
            break;
          case 11:  // menu item 13 selected
            PIDKimenu();
            redraw = MOVELIST;
            break;
          case 12:  // menu item 14 selected
            PIDKdmenu();
            redraw = MOVELIST;
            break;
//          case 13:  // menu item 14 selected
//            PIDKdOVRmenu();
//            redraw = MOVELIST;
//          break;
        }
        break;

      case 8:  //LONG PRESS
        returnToMainDisp();
        break;

      case 16:
        break;
    }

    switch (redraw) {
      case MOVECURSOR:
        redraw = false;
        if (cursorPosition > totalMenuItems) cursorPosition = totalMenuItems;
        for (i = 0; i < (totalRows); i++) {
          mydisp.setFont(10);
          mydisp.setPrintPos(0, i);
          mydisp.print(" ");
          mydisp.setPrintPos((totalCols - 1), i);
          mydisp.print(" ");
        }
        mydisp.setFont(10);
        mydisp.setPrintPos(0, cursorPosition);
        mydisp.print(">");
        break;

      case MOVELIST:
        redraw = MOVECURSOR;
        mydisp.clearScreen();
        if (totalMenuItems > ((totalRows - 1))) {
          for (i = 0; i < (totalRows); i++) {
            mydisp.setFont(10);
            mydisp.setPrintPos(1, i);
            mydisp.print(menuItems[topItemDisplayed + i]);
          }
        }
        else {
          for (i = 0; i < totalMenuItems + 1; i++) {
            mydisp.setFont(10);
            mydisp.setPrintPos(1, i);
            mydisp.print(menuItems[topItemDisplayed + i]);
          }
        }
        break;
    }
  }
  while (stillSelecting == true);
}



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
      mydisp.setFont(20);
      mydisp.setPrintPos(0, 0);
      mydisp.print("ENTER");
      delay(500);
      mydisp.clearScreen();
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

void minThrottleMenu() {
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
  mydisp.setPrintPos(0, 6);
  mydisp.print("Clck:Save Hold:Cancel");
  
  do{
    myservo.writeMicroseconds(pos); 
    switch(read_encoder())
    {
      case 1:  // ENCODER UP
        if (Reverse) {
          if (minServo >= maxServo){minServo = maxServo - 10;}
          else {minServo +=10;}
        } 
        else {
          if (maxServo >= 1950){maxServo = 1950;}
          else {maxServo +=10;}
        }
        mydisp.setPrintPos(0, 4);
        mydisp.print("Position: ");
        if (Reverse) mydisp.print(minServo); else mydisp.print(maxServo);  mydisp.print("    ");
        if (Reverse) pos = minServo; else pos = maxServo;
      break;

      case 2:    //ENCODER DOWN
        if (Reverse) {
          if (minServo <= 950){minServo = 950;}
          else {minServo -=10;}
        } 
        else {
          if (maxServo <= minServo){maxServo = minServo + 10;}
          else {maxServo -=10;}
        }
        mydisp.setPrintPos(0, 4);
        mydisp.print("Position: ");
        if (Reverse) mydisp.print(minServo); else mydisp.print(maxServo);  mydisp.print("    ");
        if (Reverse) pos = minServo; else pos = maxServo;
      break;

      case 4:  // ENCODER BUTTON SHORT PRESS
        stillSelecting = false;
        if (Reverse) EEPROM.write(21, minServo / 10); else EEPROM.write(20, maxServo / 10);
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

void maxThrottleMenu() {
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
  mydisp.setPrintPos(0, 6);
  mydisp.print("Clck:Save Hold:Cancel");
  
 do{
    myservo.writeMicroseconds(pos);
    switch(read_encoder())
    {
      case 1:  // ENCODER UP
        if(!Reverse){
          if (minServo >= maxServo){minServo = maxServo - 10;}
          else {minServo +=10;}
        }
        else {
          if (maxServo >= 1950) {maxServo = 1950;}
          else {maxServo += 10;}
        }
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
        else {
          if (minServo <= 950){minServo = 950;}
          else {minServo -= 10;}

          }
        mydisp.setPrintPos(0, 4);
        mydisp.print("Position: ");
        if (Reverse) mydisp.print(maxServo); else mydisp.print(minServo);  mydisp.print("    ");
        if (Reverse) pos = maxServo; else pos = minServo;
        break;

      case 4:  // ENCODER BUTTON SHORT PRESS
        stillSelecting = false;
        if (Reverse) EEPROM.write(20, maxServo / 10); else EEPROM.write(21, minServo / 10);
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
  mydisp.setPrintPos(0, 6);
  mydisp.print("Clck:Save Hold:Cancel"); 
 do{
    mydisp.setPrintPos(0, 1);
    mydisp.print("("); if (Reverse) mydisp.print(minServo + maxServo - pos); else mydisp.print(pos); mydisp.print(")     ");
    delay(100);
    if (pos <= minServo) S = 1;
    if (pos >= maxServo) S = -1;
    pos += 10 * S;

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

void reverseServoModeMenu() {
  boolean stillSelecting = true;
  mydisp.clearScreen();
  mydisp.setFont(10);
  mydisp.setPrintPos(0, 0);
  mydisp.print("Servo Mode");
  mydisp.setPrintPos(0, 1);
  if (Reverse) mydisp.print("Reverse Mode");
  if (!Reverse) mydisp.print("Normal Mode");
  mydisp.setPrintPos(0, 6);
  mydisp.print("Clck:Save Hold:Cancel");
  

  do {
    switch (read_encoder())
    {
      case 1:  // ENCODER UP
        mydisp.setPrintPos(0, 1);
        if (Reverse) {
          mydisp.print("Normal Mode  ");
          Reverse = false;
        }
        else {
          mydisp.print("Reverse Mode");
          Reverse = true;
        }
        break;

      case 2:    //ENCODER DOWN
        mydisp.setPrintPos(0, 1);
        if (Reverse) {
          mydisp.print("Normal Mode");
          Reverse = false;
        }
        else {
          mydisp.print("Reverse Mode");
          Reverse = true;
        }
        break;

      case 4:  // ENCODER BUTTON SHORT PRESS
        stillSelecting = false;
        if (Reverse) EEPROM.write(32, 1); else EEPROM.write(32, 0);
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

void cylinderCountMenu() {
  boolean stillSelecting = true;
  mydisp.clearScreen();
  mydisp.setFont(10);
  mydisp.setPrintPos(0, 0);
  mydisp.print("Engine Cylinders ");
  mydisp.setPrintPos(0, 1);
  mydisp.print(cylCoeff);
  if (cylCoeff == 1) {
    mydisp.setPrintPos(0, 2);
    mydisp.print("Ignition Coil Mode");
  }
  mydisp.setPrintPos(0, 6);
  mydisp.print("Clck:Save Hold:Cancel");
  do {

    switch (read_encoder())
    {
      case 1:  // ENCODER UP
        mydisp.setPrintPos(0, 1);
        if (cylCoeff == 1) {
          cylCoeff = 4;
          mydisp.print(cylCoeff);
          mydisp.setPrintPos(0, 2);
          mydisp.print("                     ");
        }
        else if (cylCoeff < 8) {
          cylCoeff = cylCoeff + 2;
          mydisp.print(cylCoeff);
          mydisp.setPrintPos(0, 2);
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
        if (cylCoeff > 4) {
          cylCoeff = cylCoeff - 2;
          mydisp.print(cylCoeff);
          mydisp.setPrintPos(0, 2);
          mydisp.print("                     ");
        }
        else {
          cylCoeff = 1;
          mydisp.print(cylCoeff);
          mydisp.setPrintPos(0, 2);
          mydisp.print("Ignition Coil Mode");
        }
        //        else {
        //          cylCoeff = 1;
        //          mydisp.print(cylCoeff);
        //          mydisp.setPrintPos(0, 2);
        //          mydisp.print("Min Cylinders");
        //        }
        break;

      case 4:  // ENCODER BUTTON SHORT PRESS
        stillSelecting = false;
        EEPROM.write(11, cylCoeff);
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

void timeZoneMenu() {
  gps.encode(Serial1.read());
  boolean stillSelecting = true;
  hours = gps.time.hour() + hourOffset - 12;
  minutes = gps.time.minute();
  seconds = gps.time.second();
  mydisp.clearScreen();
  mydisp.setFont(10);
  mydisp.setPrintPos(0, 0);
  mydisp.print("Time Zone Offset ");
  mydisp.setPrintPos(0, 1);
  mydisp.print(hours); mydisp.print(":"); if (minutes < 10) mydisp.print("0"); mydisp.print(minutes); mydisp.print(":"); if (seconds < 10) mydisp.print("0"); mydisp.print(seconds); mydisp.print("   ");
  mydisp.setPrintPos(0, 3);
  mydisp.print(hourOffset - 12); mydisp.print("   ");
  mydisp.setPrintPos(0, 6);
  mydisp.print("Clck:Save Hold:Cancel");
  do {
    gps.encode(Serial1.read());
    hours = gps.time.hour() + hourOffset - 12;
    minutes = gps.time.minute();
    seconds = gps.time.second();
    mydisp.setPrintPos(0, 1);
    delay(50);
    switch (read_encoder())
    {
      case 1:  // ENCODER UP
        if (hourOffset < 24) {
          hourOffset += 1;
        }
        else {
          hourOffset = 24;
        }
        hours = gps.time.hour() + hourOffset - 12;
        minutes = gps.time.minute();
        seconds = gps.time.second();
        if (hours < 0) hours += 24;
        if (hours > 24) hours -= 24;
        mydisp.setPrintPos(0, 3);
        mydisp.print(hourOffset - 12); mydisp.print("   ");

        break;

      case 2:  //ENCODER DOWN
        if (hourOffset > 0) {
          hourOffset -= 1;
        }
        else {
          hourOffset = 0;
        }
        hours = gps.time.hour() + hourOffset - 12;
        if (hours < 0) hours += 24;
        if (hours > 24) hours -= 24;
        mydisp.setPrintPos(0, 3);
        mydisp.print(hourOffset - 12); mydisp.print("   ");
        break;

      case 4:  // ENCODER BUTTON SHORT PRESS
        stillSelecting = false;
        EEPROM.write(19, hourOffset);
        break;

      case 8:  // ENCODER BUTTON LONG PRESS
        returnToMainDisp();
        break;

      case 16:  // ENCODER BUTTON NULL
        break;

    }

    if (hours < 0) hours += 24;
    if (hours > 24) hours -= 24;
    mydisp.setPrintPos(0, 1);
    mydisp.print(hours); mydisp.print(":"); if (minutes < 10) mydisp.print("0"); mydisp.print(minutes); mydisp.print(":"); if (seconds < 10) mydisp.print("0"); mydisp.print(seconds); mydisp.print("   ");
  }
  while (stillSelecting == true);
}

void defaultTargetSpeedMenu() {
  boolean stillSelecting = true;
  mydisp.clearScreen();
  mydisp.setFont(10);
  mydisp.setPrintPos(0, 0);
  mydisp.print("Startup Target Speed");
  mydisp.setPrintPos(0, 1);
  printTargetSpeed();
  mydisp.setPrintPos(0, 6);
  mydisp.print("Clck:Save Hold:Cancel");
  do {
    switch (read_encoder())
    {
      case 1:  // ENCODER UP
        if (Target100 >= 5000){Target100 = 5000;}
        else {Target100 += 10;}
        mydisp.setPrintPos(0, 1);
        printTargetSpeed();
        break;

      case 2:    //ENCODER DOWN
        if (Target100 <= 500){Target100 = 500;}
        else {Target100 -= 10;}
        mydisp.setPrintPos(0, 1);
        printTargetSpeed();
        break;

      case 4:  // ENCODER BUTTON SHORT PRESS
        stillSelecting = false;
        writeWord(30,Target100);
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

void unitMeasureMenu() {
  boolean stillSelecting = true;
  mydisp.clearScreen();
  mydisp.setFont(10);
  mydisp.setPrintPos(0, 0);
  mydisp.print("Unit of Measure");
  mydisp.setPrintPos(0, 1);
  if (mph) mydisp.print("MPH "); else mydisp.print("KPH ");
  mydisp.setPrintPos(0, 6);
  mydisp.print("Clck:Save Hold:Cancel");
  do {
    switch (read_encoder())
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
        if (mph) EEPROM.write(18, 1); else EEPROM.write(18, 0);
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

void tempUnitMenu() {
  boolean stillSelecting = true;
  mydisp.clearScreen();
  mydisp.setFont(10);
  mydisp.setPrintPos(0, 0);
  mydisp.print("Tempturature Mode ");
  mydisp.setPrintPos(0, 1);
  if (celsius) mydisp.print("Celsius     "); else mydisp.print("Farenheit ");
  mydisp.setPrintPos(0, 6);
  mydisp.print("Clck:Save Hold:Cancel");
  do {
    switch (read_encoder())
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
        if (celsius) EEPROM.write(29, 1); else EEPROM.write(29, 0);
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

void contrastMenu() {
  boolean stillSelecting = true;
  mydisp.clearScreen();
  mydisp.setFont(10);
  mydisp.setPrintPos(0, 0);
  mydisp.print("Contrast   ");
  mydisp.setPrintPos(0, 1);
  mydisp.print("Set to your liking");
  mydisp.setPrintPos(0, 4);
  mydisp.print("Contrast: ");
  mydisp.print(Contrast);
  mydisp.setPrintPos(0, 6);
  mydisp.print("Clck:Save Hold:Cancel");
  do {
    mydisp.setContrast(Contrast);
    switch (read_encoder())
    {
      case 1:  // ENCODER UP
        if (Contrast >= 40) {
          Contrast = 40;
        }
        else {
          Contrast += 1;
        }
        mydisp.setPrintPos(0, 4);
        mydisp.print("Contrast: ");
        mydisp.print(Contrast);
        break;

      case 2:    //ENCODER DOWN
        if (Contrast <= 20) {
          Contrast = 20;
        }
        else {
          Contrast -= 1;
        }
        mydisp.setPrintPos(0, 4);
        mydisp.print("Contrast: ");
        mydisp.print(Contrast);
        break;

      case 4:  // ENCODER BUTTON SHORT PRESS
        stillSelecting = false;
        EEPROM.write(33, Contrast);
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

void PIDKpmenu() {
  boolean stillSelecting = true;
  mydisp.clearScreen();
  mydisp.setFont(10);
  mydisp.setPrintPos(0, 0);
  mydisp.print("Proportional Gain");
  mydisp.setPrintPos(0, 1);
  mydisp.print("Default rate = .37");
  mydisp.setPrintPos(0, 3);
  mydisp.print("Rate: ");
  mydisp.print(aggKp);
  mydisp.setPrintPos(0, 6);
  mydisp.print("Clck:Save Hold:Cancel");
  int Kp100 = aggKp * 100;
  do {
    switch (read_encoder())
    {
      case 1:  // ENCODER UP
        mydisp.setPrintPos(0, 3);
        mydisp.print("Rate: ");
        aggKp = aggKp * 100;
        if (aggKp >= 100) {
          aggKp = 100;
        }
        else {
          aggKp += 1;
        }
        Kp100 = aggKp;
        aggKp /= 100;
        mydisp.print(aggKp); mydisp.print("  ");
        break;

      case 2:    //ENCODER DOWN
        mydisp.setPrintPos(0, 3);
        mydisp.print("Rate: ");
        aggKp = aggKp * 100;
        if (aggKp <= 0) {
          aggKp = 0;
        }
        else {
          aggKp -= 1;
        }
        Kp100 = aggKp;
        aggKp /= 100;
        mydisp.print(aggKp); mydisp.print("  ");
        break;

      case 4:  // ENCODER BUTTON SHORT PRESS
        stillSelecting = false;
        writeWord(12, Kp100);
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

void PIDKimenu() {
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
  mydisp.setPrintPos(0, 6);
  mydisp.print("Clck:Save Hold:Cancel");
  int Ki100 = aggKi * 100;
  do {
    switch (read_encoder())
    {
      case 1:  // ENCODER UP
        mydisp.setPrintPos(0, 3);
        mydisp.print("Speed: ");
        aggKi = aggKi * 100;
        if (aggKi >= 100) {
          aggKi = 100;
        }
        else {
          aggKi += 1;
        }
        Ki100 = aggKi;
        aggKi = aggKi / 100;
        mydisp.print(aggKi); mydisp.print("   ");
        break;

      case 2:    //ENCODER DOWN
        mydisp.setPrintPos(0, 3);
        mydisp.print("Speed: ");
        aggKi = aggKi * 100;
        if (aggKi <= 0) {
          aggKi = 0;
        }
        else {
          aggKi -= 1;
        }
        Ki100 = aggKi;
        aggKi = aggKi / 100;
        mydisp.print(aggKi); mydisp.print("     ");
        break;

      case 4:  // ENCODER BUTTON SHORT PRESS
        stillSelecting = false;
        writeWord(14, Ki100);
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

void PIDKdmenu() {
  boolean stillSelecting = true;
  mydisp.clearScreen();
  mydisp.setFont(10);
  mydisp.setPrintPos(0, 0);
  mydisp.print("Derivative Gain");
  mydisp.setPrintPos(0, 1);
  mydisp.print("Default = .08");
  mydisp.setPrintPos(0, 3);
  mydisp.print("Rate: ");
  mydisp.print(aggKd);
  mydisp.setPrintPos(0, 6);
  mydisp.print("Clck:Save Hold:Cancel");
  int Kd100 = aggKd * 100;
  do {
    switch (read_encoder())
    {
      case 1:  // ENCODER UP
        mydisp.setPrintPos(0, 3);
        mydisp.print("Rate: ");
        aggKd = aggKd * 100;
        if (aggKd >= 100) {
          aggKd = 100;
        }
        else {
          aggKd += 1;
        }
        Kd100 = aggKd;
        aggKd /= 100;
        mydisp.print(aggKd); mydisp.print("  ");
        break;

      case 2:    //ENCODER DOWN
        mydisp.setPrintPos(0, 3);
        mydisp.print("Rate: ");
        aggKd = aggKd * 100;
        if (aggKd <= 0) {
          aggKd = 0;
        }
        else {
          aggKd -= 1;
        }
        Kd100 = aggKd;
        aggKd /= 100;
        mydisp.print(aggKd); mydisp.print("  ");
        break;

      case 4:  // ENCODER BUTTON SHORT PRESS
        stillSelecting = false;
        writeWord(16, Kd100);
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

//void PIDKdOVRmenu(){
//  boolean stillSelecting = true;
//  mydisp.clearScreen();
//  mydisp.setFont(10);
//  mydisp.setPrintPos(0, 0);
//  mydisp.print("Accel Dampener");  
//  mydisp.setPrintPos(0, 1);
//  mydisp.print("Default = .08");
//  mydisp.setPrintPos(0, 3);
//  mydisp.print("Rate: ");
//  mydisp.print(accelDampKd);
//  mydisp.setPrintPos(0, 6);
//  mydisp.print("Clck:Save Hold:Cancel");
//  int accelDampKd100 = accelDampKd * 100;
//  
// do{
//    switch(read_encoder())
//     {
//     case 1:  // ENCODER UP
//        mydisp.setPrintPos(0, 3);
//        mydisp.print("Rate: ");
//        accelDampKd = accelDampKd * 100;
//        if (accelDampKd >= 100) {accelDampKd = 100;}
//        else {accelDampKd +=1;}
//        accelDampKd100 = accelDampKd * 100;
//        accelDampKd /= 100;
//        mydisp.print(accelDampKd);mydisp.print("  ");
//      break;
//
//      case 2:    //ENCODER DOWN
//        mydisp.setPrintPos(0, 3);
//        mydisp.print("Rate: ");
//        accelDampKd = accelDampKd * 100;
//        if (accelDampKd <= 0) {accelDampKd = 0;}
//        else {accelDampKd -=1;}
//        accelDampKd100 = accelDampKd * 100;
//        accelDampKd /= 100;
//        mydisp.print(accelDampKd);mydisp.print("  ");
//       break;
//
//      case 4:  // ENCODER BUTTON SHORT PRESS
//        stillSelecting = false;
//        writeWord(22,accelDampKd100);
//      break;
//
//      case 8:  // ENCODER BUTTON LONG PRESS
//        returnToMainDisp();
//      break;
//
//      case 16:  // ENCODER BUTTON NULL
//      break;
//    }
//  }
//   while (stillSelecting == true);
//}


void returnToMainDisp()
{
  stillSelecting = false;
  buttonTimes = 0;
  loop ();

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

void printTargetSpeed()
{
  if (!mph) {
    TargetSpeedInt = Target100 * 1.61;
    TargetSpeedInt = TargetSpeedInt / 10;
  }
  else{
    TargetSpeedInt = Target100 / 10;
  }
  targetSpeedWhole = TargetSpeedInt / 10;
  targetSpeedDecimal = TargetSpeedInt %10;
  int c = targetSpeedDecimal + targetSpeedWhole;
  if (c < 0) mydisp.print (" -");
  else if (targetSpeedWhole < 10) mydisp.print ("  ");
  else mydisp.print (" ");
  if (targetSpeedWhole < 0) targetSpeedWhole = -targetSpeedWhole;
  mydisp.print (targetSpeedWhole);
  mydisp.print (".");
  if (targetSpeedDecimal<0)targetSpeedDecimal = -targetSpeedDecimal;
  mydisp.print (targetSpeedDecimal);
  if (mph) mydisp.print("MPH "); else mydisp.print("KPH ");
}
