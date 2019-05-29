#include <Arduino.h>

#include "main.h"

#include <Wire.h>
#include "RTCLib.h"
#include <SPI.h>
#include <SD.h>

// this system uses a delimited text file system where data entries are separated by tabs (ASCII 009)
// define packet lengths for each variable
#define Voltage1_LEN 1
#define Voltage2_LEN 1
#define Current_LEN 1
#define RPM_LEN 2
#define Temp_LEN 2
#define GPSx_LEN 8
#define GPSy_LEN 8
#define Speed_LEN 8
#define Date_LEN 4
#define Time_LEN 4

// global pin definitions
#define RS485_State 5     // controls direction of data transfer on the RS485 Bus
#define Baud_Rate 9600
#define slave_timeout 200

#define Poll_Interval 500
#define Write_Interval 5000

// Master Address

const int Return_LEN = 32;   // for now this number must be less than 60 to prevent a buffer overflow

// Text Arrays
const char Voltage[] = {86,111,108,116,97,103,101};
const char Current[] = {67,117,114,114,101,110,116};
const char Error[] = {45,45,45,45,45,69,82,82,79,82,33,45,45,45,45,45};
const char Problem[] = {45,45,45,45,80,82,79,66,76,69,77,33,45,45,45,45};

// global variables
uint8_t command_word = 0, LEN_1 = 0, LEN_2 = 0;
uint16_t Packet_LEN = 0;
unsigned long Prev_Poll;
unsigned long Prev_Write;
String filename;

// datalogging variables
uint8_t voltage1;
uint8_t voltage2;
uint8_t current;
uint16_t RPM;
uint16_t temperature;
double lattitde;
double longditude;
double speed;
uint32_t GPS_date;
uint32_t GPS_time;

union typeconv32to8            // setup a Union for type conversions (32 bit)
{
  uint32_t LongVar;
  uint8_t Bytes[4];
}
 type32to8; 

 union typeconv64to8           // setup a Union for type conversions (64 bit)
{
  double DoubleVar;
  uint8_t Bytes[8];
}
 type64to8; 

RTC_DS1307 RTC;               // create RTC object

// system states
typedef enum SYSTEM_STATE {
WORKING,
PROBLEM,
FATAL_ERROR
}
SystemState_t;

SystemState_t SystemState;    // State Machine variable

File myFile;    // create SD Object

void setup() {
  //pinMode(LED_BUILTIN, OUTPUT);       // debugging only
  //digitalWrite(LED_BUILTIN, LOW);
  
  // initialise RS485 Connection
  pinMode(RS485_State, OUTPUT);
  digitalWrite(RS485_State, LOW);     // sets MAX485 to recieve data
  delay(3000);
  Serial.begin(Baud_Rate);
  while (!SD.begin(4)) {
    SystemState = FATAL_ERROR;         // displays error message if SD is not inserted.
    PollModule3();
  }
  PollModule3();         // grabs date from the GPS module
  uint16_t year = GPS_date % 100;
  uint8_t month = (GPS_date % 10000 - GPS_date %100)/100;
  uint8_t day = (GPS_date - GPS_date % 10000)/10000;
  uint8_t sec = (GPS_time % 10000)/100;
  uint8_t min = (GPS_time % 1000000 - GPS_time % 10000)/10000;
  uint8_t hour = (GPS_time - GPS_time % 1000000)/1000000;

  RTC.begin();
  // Check to see if the RTC is keeping time.  If it is, load the time from your computer.
  while (! RTC.isrunning()) {
    SystemState = FATAL_ERROR;        // displays error message if RTC is not responding.
    PollModule3();
  }
  if(GPS_time != 0) {
    RTC.adjust(DateTime(year, month, day, hour,  min, sec));
  }
  DateTime now = RTC.now();
  filename = String(now.day()) + "-" + String(now.month()) + "-" + String(now.year() - 2000) + ".TXT";
  myFile = SD.open(filename, FILE_WRITE);    // creates new text file with date string yy/mm/dd or accesses existing file

  if (myFile) {
    myFile.print("timeGPS");
    myFile.write(9);                        // 'tab' in ASCII
    myFile.print("RTC_Unix_Time");
    myFile.write(9);   
    myFile.print("v1");
    myFile.write(9);
    myFile.print("v2");
    myFile.write(9);
    myFile.print("I");
    myFile.write(9);
    myFile.print("RPM");
    myFile.write(9);
    myFile.print("temp");
    myFile.write(9);
    myFile.print("lattitude");
    myFile.write(9);
    myFile.print("longditude");
    myFile.write(9);
    myFile.print("speed");
    myFile.println();
    myFile.close();
  } else {
    SystemState = FATAL_ERROR; 
  }
  SystemState = WORKING;
}

void loop() {
  if (millis() - Prev_Poll >= Poll_Interval) {
    PollModule1();
    delayMicroseconds(200);
    PollModule2();
    delayMicroseconds(200);
    PollModule3();
    delayMicroseconds(200);
    Prev_Poll = millis();
  }
  SystemState = WORKING;
  if (millis() - Prev_Write >= Write_Interval) {
    myFile = SD.open(filename, FILE_WRITE);    // creates new text file with date string yy/mm/dd or accesses existing file
    DateTime now = RTC.now();
    if (myFile) {
      myFile.print(GPS_time/100);
      myFile.write(9);
      myFile.print(now.unixtime(), DEC);
      myFile.write(9);
      myFile.print(voltage1);
      myFile.write(9);
      myFile.print(voltage2);
      myFile.write(9);
      myFile.print(current);
      myFile.write(9);
      myFile.print(RPM);
      myFile.write(9);
      myFile.print(temperature);
      myFile.write(9);
      myFile.print(lattitde, 9);
      myFile.write(9);
      myFile.print(longditude, 9);
      myFile.write(9);
      myFile.print(speed, 4);
      myFile.println();
      myFile.close();
    }
    else {
      SystemState = FATAL_ERROR;
    }
    Prev_Write = millis();
  }
}

void PollModule1() {
  digitalWrite(RS485_State, HIGH);     // sets MAX485 to send data
  Serial.write(0b00000001);            // address write
  Serial.write(0);   // write out MSB (null)
  Serial.write(0);     // write out LSB  (null)
  Serial.flush();                     // ensures tx buffer is empty before setting the MAX485 to listen
  digitalWrite(RS485_State, LOW);     // sets MAX485 to recieve data
  Error_Routine();
}

void PollModule2() {
  digitalWrite(RS485_State, HIGH);     // sets MAX485 to send data
  Serial.write(0b00000010);            // address write
  Serial.write(0);   // write out MSB (null)
  Serial.write(0);     // write out LSB  (null)
  Serial.flush();                     // ensures tx buffer is empty before setting the MAX485 to listen
  digitalWrite(RS485_State, LOW);     // sets MAX485 to recieve data
  Error_Routine();
}

void PollModule3() {
  digitalWrite(RS485_State, HIGH);     // sets MAX485 to send data
  Serial.write(0b00000011);            // address write
  Serial.write( (Return_LEN >> 8) );   // write out MSB
  Serial.write(Return_LEN & 0xff);     // write out LSB
  LCD_Print();                         // sends LCD bit array
  Serial.flush();                      // ensures tx buffer is empty before setting the MAX485 to listen
  digitalWrite(RS485_State, LOW);      // sets MAX485 to recieve data
  Error_Routine();
}

void Error_Routine() {
  bool status;
  status = CheckResponse();
  if(status == false) {
    SystemState = PROBLEM;
  }
  delayMicroseconds(500);
}

int CheckResponse() {
  unsigned long response_time = millis();
  while(Serial.available() < 3) {
    if (millis() - response_time >= slave_timeout) {
      return false;
    }
  }
  command_word = Serial.read();     // read first 3 command/length bits
  LEN_1 = Serial.read();
  LEN_2 = Serial.read();
  Packet_LEN = LEN_1*256 + LEN_2;   // combines LEN_1 and LEN_2
  while(Serial.available() < LEN_2 ) {
      // prevents the buffer being cleared before it is filled
    }
  ReadData();                // empties the current command from the serial buffer if no data is parsed
  return true;
}

void ReadData() {
  switch (command_word)
  {
    case 1: {
      voltage1 = Serial.read();
      voltage2 = Serial.read();
      current = Serial.read();
      for(;Packet_LEN > (Voltage1_LEN + Voltage2_LEN + Current_LEN); Packet_LEN--) {
        Serial.read();    // reads any unused packet lengths (prevents buffer overflows)
      }
    }
    break;
      
    case 2: {
      int highByte = Serial.read();
      int lowByte = Serial.read();
      RPM = highByte*256 + lowByte;
      highByte = Serial.read();
      lowByte = Serial.read();
      temperature = highByte*256 + lowByte;
      for(;Packet_LEN > (RPM_LEN + Temp_LEN); Packet_LEN--) {
        Serial.read();    // reads any unused packet lengths (prevents buffer overflows)
      }
    }  
    break;

    case 3: {
      for (int i = 0; i < GPSy_LEN; i++) {
        type64to8.Bytes[i] = Serial.read();
      }
      lattitde = type64to8.DoubleVar;
      for (int i = 0; i < GPSx_LEN; i++) {
        type64to8.Bytes[i] = Serial.read();
      }
      longditude = type64to8.DoubleVar;
      for (int i = 0; i < Speed_LEN; i++) {
        type64to8.Bytes[i] = Serial.read();
      }
      speed = type64to8.DoubleVar;
      for (int i = 0; i < Date_LEN; i++) {
        type32to8.Bytes[i] = Serial.read();
      }
      GPS_date = type32to8.LongVar;
      for (int i = 0; i < Time_LEN; i++) {
        type32to8.Bytes[i] = Serial.read();
      }
      GPS_time = type32to8.LongVar;
      for(;Packet_LEN > (GPSx_LEN + GPSy_LEN + Speed_LEN + Date_LEN + Time_LEN); Packet_LEN--) {
        Serial.read();    // reads any unused packet lengths (prevents buffer overflows)
      }
    }
    break;
  }
}

void LCD_Print() {
  switch (SystemState)
  {
    case WORKING:
      for (int i = 0; i < Return_LEN - 32; i++)  {    // shifts any leftover packets: room for extra expansion
        Serial.write(83);
      }
      for (unsigned int i = 0; i < sizeof(Voltage)/sizeof(Voltage[0]); i++)  {
        Serial.write(Voltage[i]);
      }
      for (unsigned int i = 0; i < 16 - sizeof(Voltage)/sizeof(Voltage[0]); i++)  {
        Serial.write(32);
      }
      for (unsigned int i = 0; i < sizeof(Current)/sizeof(Current[0]); i++)  {
        Serial.write(Current[i]);
      }
      for (unsigned int i = 0; i < 16 - sizeof(Current)/sizeof(Current[0]); i++)  {
        Serial.write(32);
      }
    break;
  
    case PROBLEM:
      for (int i = 0; i < Return_LEN - 32; i++)  {    // shifts any leftover packets: room for extra expansion
        Serial.write(85);
      }
      for (int i = 0; i < 16; i++) {
        Serial.write(Problem[i]);
      }
      for (int i = 0; i < 16; i++) {
        Serial.write(45);
      }
    break;

    case FATAL_ERROR:
      for (int i = 0; i < Return_LEN - 32; i++)  {    // shifts any leftover packets: room for extra expansion
        Serial.write(85);
      }
      for (int i = 0; i < 16; i++) {
        Serial.write(Error[i]);
      }
      for (int i = 0; i < 16; i++) {
        Serial.write(45);
      }
    break;
  }
}