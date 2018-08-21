/*
  _________ __                     .___       __________                       
 /   _____//  |_  ____ _____     __| _/__.__. \______   \_____    ______ ______
 \_____  \\   __\/ __ \\__  \   / __ <   |  |  |     ___/\__  \  /  ___//  ___/
 /        \|  | \  ___/ / __ \_/ /_/ |\___  |  |    |     / __ \_\___ \ \___ \
/_______  /|__|  \___  >____  /\____ |/ ____|  |____|    /____  /____  >____  >
===============================================================================
*******************************************************************************
 STEADYPASS EEPROM LOADER FOR FIRST BOOTUP OF UNIT.
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
//*************INITIALIZING DEFINITIONS*************
byte totalRows = 6;  // total rows of LCD
byte totalCols = 1;  // total columns of LCD


DigoleSerialDisp mydisp(9, 8, 10);              //Pin Config SPI | 9: data | 8:clock | 10: SS | you can assign 255 to SS, and hard ground SS pin on module


void setup()
{
  //Serial.begin(9600);
  //delay (100);
  Serial1.begin(9600);   //GPS device
  delay (3000);
  mydisp.begin();delay(3000); mydisp.clearScreen();  delay(100);
  mydisp.setFont(30);
  mydisp.setPrintPos(0, 1);
  mydisp.print("   ");  delay(100);  mydisp.print("STEADYPASS");delay(500);
  mydisp.setPrintPos(0, 2);
  mydisp.print("      ");  delay(100);  mydisp.print("EEPROM");delay(500);
  mydisp.setPrintPos(0, 3);
  mydisp.print(" LOADER");
  delay(5000);
  mydisp.clearScreen();
  delay(500);
  mydisp.setFont(30);
  mydisp.setPrintPos(0, 1);

  //***CHECKING FOR FIRST BOOT SETTINGS***
    if (EEPROM.read(69) == 69 ) {
      mydisp.setFont(30);
      mydisp.setPrintPos(0, 1);
      mydisp.print("   ALREADY ");
      mydisp.setPrintPos(0, 3);
      mydisp.print(" INITIALIZED  ");
      delay(90000);
    }
    else{
    mydisp.setFont(30);
    mydisp.setPrintPos(0, 1);
    mydisp.print("   FIRST BOOT ");
    mydisp.setPrintPos(0, 3);
    mydisp.print(" INITIALIZE MEM  ");
    delay(2000);
    EEPROM.write(11,6);
    EEPROM.write(12,0);
    EEPROM.write(13,37);
    EEPROM.write(14,0);
    EEPROM.write(15,25);
    EEPROM.write(16,0);
    EEPROM.write(17,8);
    EEPROM.write(18,1);
    EEPROM.write(19,12);
    EEPROM.write(20,255);
    EEPROM.write(21,0);
    EEPROM.write(29,1);
    EEPROM.write(30,10);
    EEPROM.write(31,0);
    EEPROM.write(69,69);
    mydisp.clearScreen();
    mydisp.setFont(30);
    mydisp.setPrintPos(0, 1);
    mydisp.print("   COMPLETE ");
    mydisp.setPrintPos(0, 3);
    mydisp.print("LOAD STEADYPASS");
    delay(90000);
    }
}

//**RUN MAIN**|
void loop(){}
