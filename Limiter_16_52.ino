

#include <Servo.h> 
#include <SPI.h>
#include <Wire.h>
#include <OneWire.h>
#include <TinyGPS.h>
#include <EEPROM.h>

#define _Digole_Serial_SPI_
#include <DigoleSerial.h>

 
DigoleSerialDisp mydisp(9,8,255);  //SPI:Pin 9: data, 8:clock, 10: SS, you can assign 255 to SS, and hard ground SS pin on module
long mytime = 0;
float ver=16.52;  //no screensaver

//byte val=2;
//unsigned long PPP=0;
unsigned long tachTime = 0, tachTimePrev = 0;   //keep track of tach time since power on
// unsigned long rpmPulse=0;  //RPM pulse counter old

boolean debug = false;
boolean led = false;

float voltage=0, volt1, volt2;   //voltage value
int cylCoeff = 4;
boolean gpsMode = true;     //GPMS or RPM mode
//boolean gpsDone = false;    //track transition to RPM from GPS
int mode = 0;  //calculations mode
int DisplayMode = 0;   //display mode

//pins
//0 and 1 = GPS  , int 2 and 3
const int PinDT= 3;               //int0 == pin3    // Used for reading DT signal of encoder
const int PinCLK= 2;              // int 1 == pin 2 encoder second pin
const int PinSW = 4;              //encoder button 
int servoPin = 5;                 //servo PWM connected here
int rpmPin = 7;                    //  Engine RPM input connected here
const int speedPin = 6;           //int4 == pin 7 speed sensor
// 8 and 9 - LCD, defined above

const int tempBoardPin = A4;  //temperature on control board
const int voltPin = A0;  // battery voltage divider
const int lightsPin = A5; //  A5;  // lights on voltage divider
const int tempWtrPin = 11;  //temperature water
const int tempAirPin = 12;  //temperature air



 
//temp section
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
  
// end temp section


//SPEED section
 volatile unsigned long speedPulse = 0;
 unsigned long speedPulsePrev = 0;
 unsigned long speedPulseCurrent = 0;
 unsigned long speedTime = 0;
 unsigned long speedTimePrev = 0;
 int speedValue = 0, speedWater =0, speedGps=0;
 long speedCoeff = 2250;
 int TargetSpeedInt, Target100 = 1250;
 int goodSteps=15, goodStepsTemp;
 int speedBuffer[20];
 int avSpeed100;
 int SpeedCorrected=0;
 int PrevSpeed=0;  
 int Speed100, delta100, pos100;  //int * 100 values
 boolean mph = true;
 unsigned long throttleCheck=0;


//RPM section
long RPM=0;   //rpm value
int rpmBuffer[20];
volatile unsigned long duration=0; // accumulates pulse width
volatile unsigned long pulsecount=0;   //incremented by interrupt
volatile unsigned long previousMicros=0;
int targetRPM = 3000;
//long periodRPM = 1500;
long checkRPM = 0;
long periodSpeed = 3000;
long checkSpeed = 0;
int avRPM;

 
//Servo section
 static long virtualPosition=0;
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
  int pos = minServo; //servo position
 //int maxDelta = 8;
 int S = 10;



//Encoder section
volatile boolean TurnDetected = false;
volatile boolean up = false, prev_up = false;
unsigned long interruptTime=0;
long prevtime = 0;
long threshold = 7;
int buttonTimes = 0;
int buttonTarget = 7;
int buttonTimes2 = 0;
int menuItem = 1;
int idleCounter = 0,idleLimit = 2500;

boolean mainDisplay = true;
boolean modeOn = true;
volatile long encoder=0;

//long tt=0;
int hours, minutes, seconds, hourOffset = 7;

//functions definition section
float readSpeed();  // read GPS speed
int readRpm();         //read/calculate RPM and reset counters
void rpmIntHandler();  //called by interupt, increments counters
static void smartdelay(unsigned long ms);  //fuction reads temp/ voltage
void isr ();       // encoder - Interrupt service routine is executed when a HIGH to LOW transition is detected on CLK
void SpeedModeCalculations();
void RPMModeCalculations();
void MainDisplay();
void Menu();
void DebugOutput();

int Kp=30, Kd=25, Ko=20;  //Ki=0,   //PID coefficients    //eeprom 
long deltaD, deltaP;


 void setup()
 {
 Serial.begin(9600);
 Serial1.begin(9600);
  delay (200);
//digole LCD init section
  mydisp.begin();  delay(100);  mydisp.print("Display warm-up");    delay(2000);       mydisp.print("Done");   delay(100);  mydisp.clearScreen();   delay (100);
  //mydisp.setRot180(); //uncomment to rotate

  pinMode(PinCLK,INPUT);            //rotary encoder
  pinMode(PinDT,INPUT);             //rotary encoder
  pinMode(PinSW,INPUT_PULLUP);      //rotary encoder button
  attachInterrupt (0,isr,RISING);   // interrupt 0 pin 3 for encoder
  

  
  //pinMode(7,INPUT);   ///RPM pin, high when no pulse.
  
  attachInterrupt(4, rpmIntHandler, CHANGE);  //  RPM - pin 7 is gronded on pulse

  myservo.attach(servoPin);  // attaches the servo on pin  to the servo object
  myservo.writeMicroseconds(pos);
  

//load saved value here.
 if (EEPROM.read(11)!=255) cylCoeff = EEPROM.read(11);
 if (EEPROM.read(12)!=255) Kp =  EEPROM.read(12);
 if (EEPROM.read(13)!=255) Kd = EEPROM.read(13);
 if (EEPROM.read(14)!=255) Ko = EEPROM.read(14); // Ko = Ko/100;
 if (EEPROM.read(15)!=255) hourOffset = 24-EEPROM.read(15);
 if (EEPROM.read(16)!=255) maxServo = 10*EEPROM.read(16);
 if (EEPROM.read(17)!=255)  minServo = 10*EEPROM.read(17);
 if (EEPROM.read(18)!=255) mph = EEPROM.read(18);
 if (EEPROM.read(29)!=255)  celsius = EEPROM.read(29);
 if (EEPROM.read(30)!=255)  Target100 = EEPROM.read(30)*100;
 if (EEPROM.read(31)!=255)  Reverse = EEPROM.read(31);

  //Serial1.println("$PMTK314,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29");  //GTV only 
  Serial1.println("$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29");  // RMC only
  Serial1.println("$PMTK220,200*2C");  //5hz 
  
}





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

   if (millis()>checkTemp)
   {
    checkTemp = millis()+5000;
    smartdelay(1);              //read temp/voltage
    } 
   ///TEST
 
 //smartdelay(1); 

RPM = readRpm();            //read RPM
//Speed100 = readSpeed()*100; //read speed in GTV  mode

  //test  
  Speed100 = readGPS100();
//Speed100 = 10;
// delay(180);

if (Speed100 > PrevSpeed + 600) Speed100 = PrevSpeed;
//delay(200);

if (RPM > 50) {tachTime += millis()-tachTimePrev; tachTimePrev = millis();}
else { tachTimePrev = millis();}  //total time counter
  
  
  if (pos == maxServo)
 if (throttleCheck < millis())
  {
   throttleCheck = millis() + 2000;
   mydisp.setFont(18);
   mydisp.setPrintPos(0, 0);
   mydisp.print(" ! DECREASE !     ");
    mydisp.setPrintPos(0, 1);
   mydisp.print(" ! THROTTLE !     ");
   Speed100 = readGPS100();
   Speed100 = readGPS100();


   mydisp.setPrintPos(0, 0);
   mydisp.print("                  ");
    mydisp.setPrintPos(0, 1);
   mydisp.print("                  ");
  }
  


for (int i = 10; i>0; i--)  speedBuffer[i] = speedBuffer[i-1];        //store GPS speeds
speedBuffer[0] = Speed100;
avSpeed100 = 0;
for (int i = 5; i>=0; i--)  {avSpeed100 += speedBuffer[i];}
avSpeed100 = avSpeed100 / 6;                                        //speed average for last i+1 steps

for (int i = 10; i>0; i--) rpmBuffer[i] = rpmBuffer[i-1];          //store RPM values 
rpmBuffer[0] = RPM;

/*
if (abs(Target100 - Speed100)<100)
 {
   if (goodStepsTemp>0) 
   {
    rpmHistory[goodStepsTemp] = RPM;
    goodStepsTemp --;  
   }
 }
 else (goodStepsTemp = goodSteps);

if (goodStepsTemp == 0) gpsMode = false;
*/

/*
if (!gpsMode && !gpsDone)
{
  targetRPM = RPM;
  gpsDone = true;
}
*/

//Serial.print ("sp100 ");  Serial.println (Speed100); 
//Serial.print ("sp100 ");  Serial.println (Speed100); 





  
// if (gpsMode) speedValue = Speed100; else speedValue = 100*speedWater;  //use speedFloat for calculations

 
//control adjustment calculation
if (mode == 2)      SpeedModeCalculations();  //speed mode
PrevSpeed = Speed100;
if (mode == 1)     RPMModeCalculations();   /////////////// RPM mode   /////////////////
if (mode == 0)  pos = minServo;       //  OFF MODE


if (Reverse) myservo.writeMicroseconds( minServo + maxServo - pos);
  else  myservo.writeMicroseconds(pos);
 
if (debug)  DebugOutput();


//DISPLAY MAIN OUTPUT section
TargetSpeedInt = Target100/10;
speedValue = Speed100/10;  //use speedValue for display
if (mainDisplay) MainDisplay();



if (TurnDetected)  // do this only if rotation was detected
    {	
      //interruptTime = millis();   	    
      if (mode == 2) Target100 += 10*encoder;
      if (mode == 1) targetRPM += 10*encoder;
      encoder = 0;
      TurnDetected = false;          // do NOT repeat IF loop until new rotation detected
    }


if (digitalRead(PinSW) == LOW)   //menu enter
  {           
    buttonTimes++;
  } 
  else { if (buttonTimes > 0) 
            { mode++;  if (mode > 2) mode = 0;  
            if (!mainDisplay) idleCounter = 0; mainDisplay = true; mydisp.clearScreen();
            }  
         buttonTimes = 0; 
       }

if (buttonTimes > buttonTarget)     { mydisp.clearScreen();   mydisp.setFont(10); delay(500);}



while (buttonTimes > buttonTarget)      Menu();//menu begin
 



//screensaver section
 if (RPM<50) idleCounter++; else {idleCounter=0; }
 
if ((idleCounter > idleLimit) && mainDisplay)   
  { //mainDisplay = false; mydisp.clearScreen();
   }


if (!mainDisplay)     //Screensaver page
 {
   
    {  
      mydisp.setFont(120);
      mydisp.setPrintPos(0,0);
      mydisp.setTextPosOffset(20,15);
      mydisp.print(hours); mydisp.print(":"); if (minutes<10) mydisp.print("0"); mydisp.print(minutes); mydisp.print("  "); 
  } 
  
   mydisp.setFont(6);
   mydisp.setPrintPos(0, 2);  
   mydisp.setTextPosOffset(0,0); 
   mydisp.print(millis()-mytime);   mydisp.print("   ");    mytime = millis();    mydisp.print(millis()/1000); mydisp.print("   ");
   

  mydisp.setFont(10);
  mydisp.setPrintPos(0,0); 
  mydisp.print("Tach today "); mydisp.print(tachTime/60000); mydisp.println(" min");
  delay(20);

  mydisp.setPrintPos(0,6); mydisp.print("UNT "); mydisp.print(Temp); if (celsius) mydisp.print("c"); else mydisp.print("F"); mydisp.print(" Batt ");mydisp.print(voltage); mydisp.print("v ");
      
  mydisp.setPrintPos(0,5); mydisp.print("WTR "); mydisp.print(wtrTemp); if (celsius) mydisp.print("c"); else mydisp.print("F");  mydisp.print(" AIR ");mydisp.print(airTemp); if (celsius) mydisp.print("c "); else mydisp.print("F ");
  
  



    //delay(100);

     if (RPM > 500)  {idleCounter = 0; mainDisplay = true; mydisp.clearScreen();}  
     
 }   //end screen saver 

/*
//tweak on the fly
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

 
 } //end main loop  
  

static void smartdelay(unsigned long ms)
{
   volt1 = voltage; 
  for (i=1; i<3; i++)  voltage = analogRead(0);
    
  voltage =  (float)voltage/35.2 ;
  
 // unsigned long start = millis();
 // return;
  
  //temp section  
 /*  internal temp disabled
 if ( !ds.search(addr)) 
     {       ds.reset_search();     }
  ds.reset();
  ds.select(addr);
  ds.write(0x44);   
  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);      // Read Scratchpad
    for ( i = 0; i < 9; i++) 
    { data[i] = ds.read();  }
    Temp=(data[1]<<8)+data[0];//take the two bytes from the response relating to temperature
  Temp=Temp>>4;//divide by 16 to get pure celcius readout
 if (Temp == -1) Temp = 0; 
   else if (!celsius)  Temp = Temp * 9 / 5 +32;
  */
  
  if ( !air.search(addr)) 
     {       air.reset_search();     }
  air.reset();
  air.select(addr);
  air.write(0x44);   
  present = air.reset();
  air.select(addr);    
  air.write(0xBE);      // Read Scratchpad
    for ( i = 0; i < 9; i++) 
    { data[i] = air.read();  }
    airTemp=(data[1]<<8)+data[0];//take the two bytes from the response relating to temperature
  airTemp=airTemp>>4;//divide by 16 to get pure celcius readout
   if (airTemp == -1) airTemp = 0; 
   else if (!celsius)  airTemp = airTemp * 9 / 5 +32;


   if ( !wtr.search(addr)) 
     {       wtr.reset_search();     }
  wtr.reset();
  wtr.select(addr);
  wtr.write(0x44);   
  present = wtr.reset();
  wtr.select(addr);    
  wtr.write(0xBE);      // Read Scratchpad
    for ( i = 0; i < 9; i++) 
    { data[i] = wtr.read();  }
    wtrTemp=(data[1]<<8)+data[0];//take the two bytes from the response relating to temperature
  wtrTemp = wtrTemp>>4;//divide by 16 to get pure celcius readout
   if (wtrTemp == -1) wtrTemp = 0; 
   else if (!celsius) wtrTemp = wtrTemp * 9 / 5 +32;
    
// end TEMP section
}


int readGPS100()    //for RMC
{
  char c;
  int zap=0, pos=0;
  char charTime[8];
  char charSpeed[6];
  
  c = Serial1.read();
  while (c!='$') if (Serial1.available()) c = Serial1.read(); 
    else delay(1); 
  //Serial.print(c);
  while (c!=',') if (Serial1.available()) c = Serial1.read(); 
    else delay(1); 
 // Serial.print(c);
  while (Serial1.available()<6) delay(1);
charTime[0] = Serial1.read();
charTime[1] = Serial1.read();
hours = atoi(charTime); 

charTime[0] = Serial1.read();
charTime[1] = Serial1.read();
minutes = atoi(charTime); 

charTime[0] = Serial1.read();
charTime[1] = Serial1.read();
seconds = atoi(charTime); 

hours -=hourOffset;
if (hours <0) hours += 24; 


  while (zap < 6) 
  {
   if (Serial1.available()) 
    { 
     c = Serial1.read(); 
     //Serial.print(c);
     if (c == ',') zap++; 
    }
   else delay(1);
  }
  
 // Serial.print(c);
  while (Serial1.available()<6) delay(1);
  
  c = Serial1.read(); 
  //Serial.print(c);
  while (c!=',')  
  {
  charSpeed[pos] = c;
  if (c == '.')  pos--;//Serial.print("~!!!!");
     
  pos++;
   c = Serial1.read(); 
   //Serial.print(c);
  }
   //Serial.println(" "); 
//  for (int i = 0; i<8; i++)
//  Serial.print(charTime[i]);
//  Serial.println(" - time");

//  for (int i = 0; i<pos; i++)
//  Serial.print(charSpeed[i]);
//  Serial.println(" - speed array");
  
// return(1);
  
  int tempSpeed = atoi(charSpeed);
//  Serial.print("date char ");Serial.println(charTime);
//  Serial.print("speed char ");Serial.println(charSpeed);
//  Serial.print("speed# ");Serial.println(tempSpeed);

  tempSpeed *= 1.15078;
  if (!mph) tempSpeed *= 1.609344;
        
  return(tempSpeed);
  
//  while(Serial1.available()) Serial1.read();  //flush it to avoid congestion  
}



float readSpeed()
{
int m=0,n=0;
float ParsedSpeed=-1;
boolean Parsed = false;
char c;
 String inString = "";    // string to hold input
do 
 {
  //Parsed = false;
  while (Serial1.available()) 
   {
   c = Serial1.read();
   if (c != '\n') 
    { 
       inString += c;
    }
    else 
       {
     // Serial.print("sting before ");Serial.println(inString);
         m = inString.indexOf('M');
       n = inString.indexOf('N');
       if (m>0 && n>0) 
        {
        inString = inString.substring(m+2,n-1);
       // Serial.print("sting after ");Serial.println(inString);
        char buffer[5];
        inString.toCharArray(buffer, 5); 
        inString = "";    
        ParsedSpeed = atof(buffer);
        ParsedSpeed *= 1.15078;
        if (!mph) ParsedSpeed *= 1.609344;
    //    Serial.print(" number + 5 ");Serial.println(5+ParsedSpeed); 
        //m=0; 
        //n=0; 
        Parsed = true;
        }
       }
    }
  } while (!Parsed);
  
while(Serial1.available()) Serial1.read();  //flush it to avoid congestion  
return (ParsedSpeed); 
}



int readRpm()
{
    unsigned long _duration = duration;
    unsigned long _pulsecount = pulsecount;
    duration = 0; // clear counters
    pulsecount = 0;
      long freqq=60*1e5*_pulsecount/_duration/cylCoeff;
      freqq *=10; 
      
      //  Serial.print("freqq ");Serial.println(freqq);
    return(freqq);
  
}


void rpmIntHandler() // interrupt handler
{
  unsigned long currentMicros = micros();
  duration += currentMicros - previousMicros;
  previousMicros = currentMicros;
  pulsecount++;
}

void isr ()         // encoder - Interrupt service routine is executed when a HIGH to LOW transition is detected on CLK
{
  if (digitalRead(PinCLK) == HIGH)
    {
     up = true;
    encoder--;}
  else
    {
     up = false;
    encoder++;
    }
  TurnDetected = true;
   // Serial.println (encoder); 
}

void SpeedModeCalculations()
 {
  // checkSpeed = millis() + 1000;
   
  int SpeedDelta = Speed100 - speedBuffer[1];
   SpeedCorrected = Speed100 + SpeedDelta;
  deltaP = SpeedCorrected - Target100;
  deltaD = PrevSpeed - Speed100;
 // PrevSpeed = Speed100;
  if (abs(Speed100-Target100)<150)    
    deltaD = deltaD * 4;
  if (abs(Speed100-Target100)<70)    
    deltaD = deltaD * 5;
  deltaP = constrain(deltaP,-150,150);
 
  delta100 = Ko*(Kp*(float)deltaP/10 - Kd*(float)deltaD/10)/100; 
   

 pos100 += delta100;

 pos100 = constrain(pos100,minServo*10,maxServo*10);
 pos = pos100/10;
}  //end SPEED Mode 


void RPMModeCalculations()
{
 // int rpmDelta = Speed100 - speedBuffer[3];
 //  SpeedCorrected = Speed100 + SpeedDelta;


 deltaD = avRPM - RPM;
     avRPM = 0;
     for (int i = 2; i>=0; i--) avRPM += rpmBuffer[i];          //calculate average RPM for i+1 steps
     avRPM = avRPM/3;
 //avRPM = RPM;   //cancelling averaging for test
 deltaP = avRPM - targetRPM;
  
 delta100 = Ko*(Kp*(float)deltaP/10 - Kd*(float)deltaD/10)/100; 
 pos100 += delta100;
 pos100 = constrain(pos100,minServo*10,maxServo*10);
   pos = pos100/10;
 ///  END OF RPM mode ////////////////////////////////////////////////////////////
}

void MainDisplay()
{

  //time output
   mydisp.setFont(10);
   mydisp.setPrintPos(0, 1); 
   mydisp.print(hours); mydisp.print(":"); if (minutes<10) mydisp.print("0"); mydisp.print(minutes);mydisp.print(":"); if (seconds<10) mydisp.print("0"); mydisp.print(seconds);mydisp.print("            ");


  mydisp.setFont(10);
   mydisp.setPrintPos(10, 2);  
   mydisp.setTextPosOffset(0,-6); 
      mydisp.setFont(6);
//heartbeat output
   //mydisp.print(millis()-mytime);   mydisp.print("   ");    mytime = millis();    mydisp.print(millis()/1000);  mydisp.print(" beat ");    //heart beat for debug purpose
   mydisp.setFont(10);  
   mydisp.setPrintPos(18, 3);   
   mydisp.setTextPosOffset(2,-2);   
   mydisp.print("GPS");
   mydisp.setPrintPos(18, 4);   
   mydisp.setTextPosOffset(2,-3);      
   if (mph) mydisp.print("MPH"); else mydisp.print("KPH");
  
   mydisp.setPrintPos(0, 0); 
   mydisp.print("POWER ");     
   if (pos == maxServo)  mydisp.print("MIN "); 
   else  if (pos == minServo)  mydisp.print("FULL "); 
   else  mydisp.print(maxServo/10 - pos/10);    
   mydisp.print("  "); 
      
  
   mydisp.setFont(18);
   mydisp.setPrintPos(7, 0);
   if (mode == 0) mydisp.print("  OFF  ");
   if (mode == 1) { mydisp.print(targetRPM);  mydisp.print("RPM"); }
   if (mode == 2) {  mydisp.print(TargetSpeedInt/10);   mydisp.print(".");   mydisp.print(TargetSpeedInt%10);   
     if (mph) mydisp.print("MPH"); else mydisp.print("KPH");   } 
   

   mydisp.setFont(10);   
   mydisp.setPrintPos(0, 2); 
   mydisp.print("V ");  mydisp.print(voltage); mydisp.print("  ");
  
  /* mydisp.setPrintPos(0, 3); 
   mydisp.print("UNT "); mydisp.print(Temp); if (celsius) mydisp.print("c "); else mydisp.print("F ");
   */
   
   mydisp.setPrintPos(0, 4);  
   mydisp.print("WTR "); mydisp.print(wtrTemp); if (celsius)  mydisp.print("c  ");else mydisp.print("F ");
   mydisp.setPrintPos(0, 5); 
   mydisp.print("AIR "); mydisp.print(airTemp); if (celsius)  mydisp.print("c  ");else mydisp.print("F ");
   mydisp.setPrintPos(0, 6); 
   mydisp.print("RPM "); mydisp.print(RPM); mydisp.print("    ");
  
  //Main speed output   
    //mydisp.setFont(123);      
    mydisp.setPrintPos(0, 0); 
  //  

mydisp.setFont(203);  
mydisp.setPrintPos(0, 0); 
mydisp.setTextPosOffset(46,22);   //main speed

    if (speedValue<100) mydisp.print('0'); 
    mydisp.print(speedValue/10);  
    //mydisp.setFont(51);    
    mydisp.setFont(201);
    mydisp.setPrintPos(0,0); 
    mydisp.setTextPosOffset(111,43); 
    mydisp.print(speedValue%10); 



}    
//end MAIN display section

void Menu()
 {

  mydisp.setFont(10);
  mydisp.setPrintPos(0, 0); 
  if (menuItem == 1) { mydisp.print(">>");    pos = maxServo; }
 mydisp.print("Min Throttle ");  if (Reverse) mydisp.print(minServo); else mydisp.print(maxServo);  mydisp.print("    ");  
 delay(10); 
  mydisp.setPrintPos(0, 1);
   if (menuItem == 2)  { mydisp.print(">>");    pos = minServo; }
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
   pos += 10*S;
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
  mydisp.print("DFT SP ");   mydisp.print(Target100/100); 
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
     mydisp.print("VER: "); mydisp.print(ver);
  
  mydisp.setPrintPos(29,9);      mydisp.setFont(6);     mydisp.print(buttonTimes2);mydisp.print(" "); mydisp.print(menuItem);
  
 
 if (TurnDetected)  
  {		    // do this only if rotation was detected
    if ((millis() - prevtime) > threshold) {
      prevtime = millis();
      if (up == prev_up) {
        if (up)
          {
          if (menuItem == 1) if (Reverse) minServo -=10; else maxServo -=10;  
          if (menuItem == 2)  if (Reverse) maxServo -=10; else minServo -=10;  
          //if (menuItem == 3) 
          if (menuItem == 4)  cylCoeff--;  
          if (menuItem == 5) hourOffset -=1;
          if (menuItem == 6)  Reverse = !Reverse;
          if (menuItem == 7) Kd -=1;
          if (menuItem == 8)  Ko -=1;
          if (menuItem == 9)  Target100 -=100;
          if (menuItem == 10)  mph=!mph;
          if (menuItem == 11) celsius = !celsius;
          if (menuItem == 12) debug = !debug;
          
          }
        else
          {
       if (menuItem == 1) if (Reverse) minServo +=10; else maxServo+=10;  
          if (menuItem == 2)  if (Reverse) maxServo +=10; else minServo+=10;  
          //if (menuItem == 3) 
          if (menuItem == 4)  cylCoeff++;  
          if (menuItem == 5) hourOffset +=1;
          if (menuItem == 6)  Reverse = !Reverse;
          if (menuItem == 7) Kd +=1;
          if (menuItem == 8)  Ko +=1;
          if (menuItem == 9)  Target100 += 100;
          if (menuItem == 10)  mph=!mph;
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
    if (digitalRead(PinSW) == HIGH)   if (menuItem >11) menuItem = 1; else menuItem++;
    buttonTimes2++;
  } 
  else { buttonTimes2 = 0;} 
  
  if (buttonTimes2>4)  {buttonTimes = 0;  buttonTimes2 = 0;  menuItem = 1;  mydisp.clearScreen();    //exit procedure

  if (EEPROM.read(11)!= cylCoeff)    
  EEPROM.write(11,cylCoeff);
  if (EEPROM.read(12)!= Kp      )    
  EEPROM.write(12,Kp);
  if (EEPROM.read(13)!= Kd      )    
  EEPROM.write(13,Kd);
  if (EEPROM.read(14)!= Ko      )    
  EEPROM.write(14,Ko);
  if (EEPROM.read(15)!= 24-hourOffset) 
  EEPROM.write(15,(24-hourOffset));
  if (EEPROM.read(16)!= maxServo/10 ) 
  EEPROM.write(16,(maxServo/10)); 
  if (EEPROM.read(17)!= minServo/10 ) 
  EEPROM.write(17,(minServo/10)); 
  if (EEPROM.read(18)!= mph     )     
  EEPROM.write(18,(mph)); 
  if (EEPROM.read(29)!= celsius )
  EEPROM.write(29,(celsius)); 
  


  EEPROM.write(30,(Target100/100)); 
  EEPROM.write(31,Reverse);
    Serial.print("EEPROM!: ");   Serial.println(millis()); 
    
    }  //while exit condition met
 }  //end while 
 
 void DebugOutput()
 {
   Serial.print("Time: ");   Serial.print(millis()); 
  Serial.print(" GPS_Sp: ");  Serial.print(Speed100);
  Serial.print(" Sp_Targ100: ");  Serial.print(Target100);
  Serial.print(" RPM: "); Serial.print(RPM); 
  Serial.print(" RPM_target: "); Serial.print(targetRPM); 
  Serial.print(" Srv: ");Serial.println(180-pos);
 }
