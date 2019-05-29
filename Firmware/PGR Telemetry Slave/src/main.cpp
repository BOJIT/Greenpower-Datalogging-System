#include <Arduino.h>
//!!!!!!Modified Rx Buffer Size in HardwareSerial.h!!!!!!!!

// slave module definition
//#define Module1     // Voltage, Current
#define Module2     // Temperature, RPM
//#define Module3     // LCD, GPS

#include "main.h"

// global pin definitions
#define RS485_State 5     // controls direction of data transfer on the RS485 Bus

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
#define Baud_Rate 9600

// module libraries
#ifdef Module1

  #define address 0b00000001  //address is 8 bit

  uint8_t Return_Array[Voltage1_LEN + Voltage2_LEN + Current_LEN];

  uint8_t battery1;           // 10 storage locations + 1 for average
  #define battery1_Scaler 1
  #define battery1_Offset 0
  uint8_t battery2;
  #define battery2_Scaler 1
  #define battery2_Offset 0
  //uint8_t batteryAvg = sizeof(battery1)/sizeof(battery1[0]);
  #define batterySample_Period 1000
  uint8_t current;
  #define current_Scaler 1
  #define current_Offset 0
  //uint8_t currentAvg = sizeof(current)/sizeof(current[0]);
  #define currentSample_Period 1000
  #define battery1Pin A0
  #define battery2Pin A1
  #define currentPin A6
#endif

#ifdef Module2

  #define address 0b00000010  //address is 8 bit

  uint8_t Return_Array[RPM_LEN + Temp_LEN];

  const int Thresh_U = 500;
  const int Thresh_L = 460;

  #define Temp_Scaler 1
  #define Temp_Offset 0
  #define TemperatureSample_Period 1000
  #define HallPin A6
  #define TempPin A7
#endif

#ifdef Module3

  #define address 0b00000011  //address is 8 bit

  uint8_t Return_Array[GPSx_LEN + GPSy_LEN + Speed_LEN + Date_LEN + Time_LEN];

  #include <SoftwareSerial.h>
  #include <TinyGPS++.h>
  #include <LiquidCrystal.h>
  #define GPS_RXD 2
  #define GPS_TXD 3
  #define GPS_Baud_Rate 9600

  SoftwareSerial ss(GPS_TXD,GPS_RXD);     // create SoftwareSerial Object

  TinyGPSPlus gps;      // NMEA Parser Object

  #define LCDSample_Period 500
  #define GPSSample_Period 100

  uint8_t LCD_Buffer[32];

  unsigned long LCD_Update;

  const int rs = 9, en = 10, d4 = A5, d5 = A4, d6 = A3, d7 = A2;    // define LCD control pins
  LiquidCrystal lcd(rs, en, d4, d5, d6, d7);                        // create LCD object

#endif

// global variables
uint8_t command_word = 0, LEN_1 = 0, LEN_2 = 0;
uint16_t Packet_LEN = 0;

union typeconv32to8            //setup a Union for type conversions (32 bit)
{
  uint32_t LongVar;
  uint8_t Bytes[4];
}
 type32to8; 

 union typeconv64to8           //setup a Union for type conversions (64 bit)
{
  double DoubleVar;
  uint8_t Bytes[8];
}
 type64to8; 

void setup() {

  // initialise RS485 Connection
  pinMode(RS485_State, OUTPUT);
  digitalWrite(RS485_State, LOW);     // sets MAX485 to receive data
  Serial.begin(Baud_Rate);
  
  #ifdef Module1

  #endif
  
  #ifdef Module2

  #endif

  #ifdef Module3
    for (int i = 0;  i < 32; i++) {
      LCD_Buffer[i] = 32;
    }
    ss.begin(GPS_Baud_Rate);
    lcd.begin(16, 2);
    lcd.setCursor(0,0);
    lcd.print("Poole Grammar");
    lcd.setCursor(0,1);
    lcd.print("Racing");
    delay(2000);
    lcd.clear();
    LCD_Update = millis() + 1000;
  #endif

}

void loop() {

ParseData();    // checks serial port for incoming commands

#ifdef Module1
Voltage_Sample();
Current_Sample();
#endif
  
#ifdef Module2
Temp_Sample();
RPM_Calc();
#endif

#ifdef Module3
LCD_Print();
GPS_Sample();
#endif

}

void ParseData() {

  if(Serial.available() > 2)  {
    command_word = Serial.read();     // read first 3 command/length bits
    LEN_1 = Serial.read();
    LEN_2 = Serial.read();
    Packet_LEN = LEN_1*256 + LEN_2;   // combines LEN_1 and LEN_2

    while(Serial.available() < LEN_2 ) {
      // prevents the buffer being cleared before it is filled
    }

    if(command_word == address)  {
      #ifdef Module3
      for(;Packet_LEN > 32; Packet_LEN--) {     // empties buffer up to the last 32 bytes
        Serial.read();
      }
      for(;Packet_LEN > 0; Packet_LEN--) {     // empties buffer up to the last 32 bytes
        LCD_Buffer[32 - Packet_LEN] = Serial.read();
      }
      #endif
      #ifndef Module3
      for(;Packet_LEN > 0; Packet_LEN--) {
        Serial.read();                // empties the current command from the serial buffer if no data is parsed
      }
      #endif
      Report_Vals();
    }
    else
    {
      for(;Packet_LEN > 0; Packet_LEN--) {
        Serial.read();                // empties the current command from the serial buffer if no data is parsed
      }
    }
  }

}

void Report_Vals() {  // reports the datalogger values back to the master
  //delayMicroseconds(50);
  int Return_LEN = sizeof(Return_Array)/sizeof(Return_Array[0]);
  digitalWrite(RS485_State, HIGH);     // sets MAX485 to send data
  Serial.write(address);            // address write
  Serial.write( (Return_LEN >> 8) );   // write out MSB
  Serial.write(Return_LEN & 0xff);     // write out LSB
  for (int i = 0; i < Return_LEN; i++) {
    Serial.write(Return_Array[i]);
  }
  Serial.flush();                     // ensures tx buffer is empty before setting the MAX485 to listen
  digitalWrite(RS485_State, LOW);     // sets MAX485 to receive data
}

// Module 1 Subroutines
#ifdef Module1

  void Voltage_Sample() {
    Return_Array[0] = (uint8_t)(analogRead(battery1Pin)/4 * battery1_Scaler + battery1_Offset);
    Return_Array[1] = (uint8_t)(analogRead(battery2Pin)/4 * battery2_Scaler + battery2_Offset);
    /*batteryAvg--;
    if(batteryAvg == 0) {
      for (unsigned int i = 0; i < sizeof(battery1)/sizeof(battery1[0]) - 1; i++) {
        battery1[sizeof(battery1)/sizeof(battery1[0]) - 1] = battery1[sizeof(battery1)/sizeof(battery1[0]) - 1] + battery1[i];
        battery2[sizeof(battery2)/sizeof(battery2[0]) - 1] = battery2[sizeof(battery2)/sizeof(battery2[0]) - 1] + battery2[i];
      }
      batteryAvg = sizeof(battery1)/sizeof(battery1[0]);
    }*/
  }

  void Current_Sample() {
    Return_Array[2] = (uint8_t)(analogRead(currentPin)/4 * current_Scaler + current_Offset);
    /*currentAvg--;
    if(currentAvg == 0) {
      for (unsigned int i = 0; i < sizeof(current)/sizeof(current[0]) - 1; i++) {
        current[sizeof(current)/sizeof(current[0]) - 1] = current[sizeof(current)/sizeof(current[0]) - 1] + current[i];
      }
      currentAvg = sizeof(current)/sizeof(current[0]);
    }*/
  }

#endif

// Module 2 Subroutines
#ifdef Module2
  
  void Temp_Sample() {
    uint16_t Temperature = analogRead(TempPin) * Temp_Scaler + Temp_Offset;
    Return_Array[2] = (Temperature >> 8);
    Return_Array[3] = Temperature & 0xFF;
  }

  void RPM_Calc() {
    bool Hall_State;
    bool Prev_Hall_State = 1;
    unsigned long RPM_Timer = 0;
    if(analogRead(HallPin) >= Thresh_U || analogRead(HallPin) <= Thresh_L) {
      Hall_State = 1;
      if (Prev_Hall_State == 0) {
        uint16_t RPM = (uint16_t)(60/((micros() - RPM_Timer)*1000000));
        Return_Array[0] = (RPM >> 8);
        Return_Array[1] = RPM & 0xFF;
        RPM_Timer = micros();
      }
    }
    else
    {
      Hall_State = 0;
    }
    Prev_Hall_State = Hall_State;
  }

#endif

// Module 3 Subroutines
#ifdef Module3

  void LCD_Print() {
    if (millis() > LCD_Update ) {
      LCD_Update = LCD_Update + LCDSample_Period;
      lcd.clear();
      lcd.setCursor(0,0);
      for(int i = 0; i < 16; i++) {     // LCD top line
        lcd.write(LCD_Buffer[i]);
      }
      lcd.setCursor(0,1);
      for(int i = 0; i < 16; i++) {      // LCD bottom line
        lcd.write(LCD_Buffer[i+16]);
      }
    }
  }

  void GPS_Sample() {
    while (ss.available() > 0){
      // get the byte data from the GPS
      gps.encode(ss.read());
    }
    type64to8.DoubleVar = gps.location.lat();
    for (int i = 0; i < GPSy_LEN; i++) {
      Return_Array[i] = type64to8.Bytes[i];
    }
    type64to8.DoubleVar = gps.location.lng();
    for (int i = 0; i < GPSx_LEN; i++) {
      Return_Array[i + GPSy_LEN] = type64to8.Bytes[i];
    }
    type64to8.DoubleVar = gps.speed.mph();
    for (int i = 0; i < Speed_LEN; i++) {
      Return_Array[i + GPSy_LEN + GPSx_LEN] = type64to8.Bytes[i];
    }
    type32to8.LongVar = gps.date.value();
    for (int i = 0; i < Date_LEN; i++) {
      Return_Array[i + GPSy_LEN + GPSx_LEN + Speed_LEN] = type32to8.Bytes[i];
    }
    type32to8.LongVar = gps.time.value();
    for (int i = 0; i < Time_LEN; i++) {
      Return_Array[i + GPSy_LEN + GPSx_LEN + Speed_LEN + Date_LEN] = type32to8.Bytes[i];
    }
  }

#endif