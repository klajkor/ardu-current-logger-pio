/**********************************************
** Arduino current meter and logger
** Modules used:
**  - INA219, I2C
**  - DS3231 RTC, I2C
**  - SSD1306 OLED dsiplay, I2C
**  - SD card, SPI
**********************************************/

/* Arduino Nano pinout connections

 ** I2C bus:
 ** SCK - pin A5
 ** SDA - pin A4

 ** SD card on SPI bus:
 ** MOSI - pin D11
 ** MISO - pin D12
 ** CLK - pin D13
 ** CS - pin D4
*/

#include <Arduino.h>
#include <avr/pgmspace.h>
#include <math.h>
#include <uRTCLib.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"

/* Declarations and initializations */

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
const int OLED_I2C_ADDR = 0x3C; // Address 0x3C for 128x32

SSD1306AsciiWire display;

// Current and voltage sensor
Adafruit_INA219 ina219_monitor;

// DS3231 RTC modul
const int RTC_I2C_addr = 0x68;
uRTCLib rtc(RTC_I2C_addr);

// SD card modul
#define SDCARD_CHIP_SELECT 4


/* Global variables */

// RTC global variables
uint8_t rtc_second = 0;
uint8_t rtc_minute = 0;
uint8_t rtc_hour = 0;
uint8_t rtc_day = 0;
uint8_t rtc_month = 0;
uint8_t rtc_year = 0;

// Current sensor variables
float f_BusVoltage_V;
float f_ShuntCurrent_mA;


// General variables
char DateStampString[] = "2000.99.88";
char TimeStampString[] = "00:00:00";
char logFileName[] = "mmddHHMM.txt";
char VoltString[] = "99.999 ";
char CurrentString[] = "9999.999 ";
bool SD_log_enabled = false;

File dataFile;


// Function definitions
//void setup();
//void loop();
//bool Log_To_SD_card(const char *_logfile);
bool Log_To_SD_card();
void setTimeStampString();


//setup()
void setup()
{
  Serial.begin(115200);
  Wire.begin();
  delay(100);
  
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  //display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR);
  Serial.print(F("SSD1306 init..."));
  #if OLED_RESET >= 0
  display.begin(&Adafruit128x32, I2C_ADDRESS, RST_PIN);
  #else // RST_PIN >= 0
  display.begin(&Adafruit128x32, OLED_I2C_ADDR);
  #endif // RST_PIN >= 0

  display.setFont(Adafruit5x7);

  Serial.println(F(" OK"));
  display.clear();
  display.set1X();
  display.setRow(0);
  
  
  ina219_monitor.begin();
  //Serial.println(F("INA219 begin done"));
  // begin calls:
  // configure() with default values RANGE_32V, GAIN_8_320MV, ADC_12BIT, ADC_12BIT, CONT_SH_BUS
  // calibrate() with default values D_SHUNT=0.1, D_V_BUS_MAX=32, D_V_SHUNT_MAX=0.2, D_I_MAX_EXPECTED=2
  // in order to work directly with ADAFruit's INA219B breakout

  Serial.print(F("SD card "));
  display.println(F("SD card "));

  // see if the card is present and can be initialized:
  if (!SD.begin(SDCARD_CHIP_SELECT)) {
    Serial.println(F(" failed"));
    display.println(F(" failed"));
    SD_log_enabled = false;
  }
  else {
    SD_log_enabled = true;
    Serial.println(F(" OK"));
    display.println(F(" OK"));    
  }
  delay(500);
  setTimeStampString();
  
  delay(2500);
  display.clear();

} 

void loop()
{
  setTimeStampString();
  
  // clear display
  //display.clear();
  //display.set1X();
  display.setRow(0);
  display.setCol(0);
  
  //display time stamp
  Serial.print(DateStampString);
  Serial.print(F(" "));
  Serial.print(TimeStampString);
  Serial.print(F(" "));
  display.print(DateStampString);
  display.print(F(" "));
  display.println(TimeStampString);
  //Serial.println(logFileName);

  //measure voltage and current
  f_ShuntCurrent_mA=ina219_monitor.getCurrent_mA();
  f_BusVoltage_V=ina219_monitor.getBusVoltage_V();
  
  //convert to text
  dtostrf((f_ShuntCurrent_mA),8,3,CurrentString);
  dtostrf(f_BusVoltage_V,6,3,VoltString);
  //CurrentString=String(f_ShuntCurrent_mA*1000, 4);
  //CurrentString+=F(" mA");
  //VoltString="";
  //VoltString=String(f_BusVoltage_V,4);
  //VoltString+=F(" V");
  
  //display volt
  Serial.print(VoltString);
  Serial.print(F(" V"));
  display.print(VoltString);
  display.println(F(" V"));

  //display current
  Serial.print(CurrentString);
  Serial.println(F(" mA "));
  display.print(CurrentString);
  display.println(F(" mA"));

  if(SD_log_enabled) {
    Serial.print(F("SD log: "));
    Serial.print(logFileName);
    if (Log_To_SD_card()) {
      Serial.println(F(" OK"));
      display.print(F("SD log OK"));
    }
    else {
      Serial.println(F(" failed!"));
      display.print(F("SD log failed"));      
    }
    //display.print(logFileName);
    //display.display();
  }

  delay(9500);

}

void setTimeStampString()
{
  // get time stamp, convert to a string
  rtc.refresh();
  rtc_second = rtc.second();
  rtc_minute = rtc.minute();
  rtc_hour = rtc.hour();
  rtc_day = rtc.day();
  rtc_month = rtc.month();
  rtc_year = rtc.year();

  DateStampString[2] = (char) ((rtc_year / 10)+0x30);
  DateStampString[3] = (char) ((rtc_year % 10)+0x30);
  DateStampString[5] = (char) ((rtc_month / 10)+0x30);
  DateStampString[6] = (char) ((rtc_month % 10)+0x30);
  DateStampString[8] = (char) ((rtc_day / 10)+0x30);
  DateStampString[9] = (char) ((rtc_day % 10)+0x30);
  
  TimeStampString[0] = (char) ((rtc_hour / 10)+0x30);
  TimeStampString[1] = (char) ((rtc_hour % 10)+0x30);
  TimeStampString[3] = (char) ((rtc_minute / 10)+0x30);
  TimeStampString[4] = (char) ((rtc_minute % 10)+0x30);
  TimeStampString[6] = (char) ((rtc_second / 10)+0x30);
  TimeStampString[7] = (char) ((rtc_second % 10)+0x30);

  logFileName[0] = DateStampString[0];
  logFileName[1] = DateStampString[1];
  logFileName[2] = DateStampString[2];
  logFileName[3] = DateStampString[3];
  logFileName[4] = DateStampString[5];
  logFileName[5] = DateStampString[6];
  logFileName[6] = DateStampString[8];
  logFileName[7] = DateStampString[9];
  
  
}

//bool Log_To_SD_card(const char *_logfile)
bool Log_To_SD_card()
{
  bool FileOpenSuccess = false;
  
  dataFile = SD.open(logFileName, FILE_WRITE);
  //dataFile = SD.open(_logfile, FILE_WRITE);
  
  // if the file is available, write to it:
  if (dataFile) {
    FileOpenSuccess=true;
  }
  else {
    FileOpenSuccess=false;
    Serial.println(F(" logfile open error!"));
  }
  
  if (FileOpenSuccess) {
    dataFile.print(DateStampString);
    dataFile.print(F(","));
    dataFile.print(TimeStampString);
    dataFile.print(F(","));
    dataFile.print(VoltString);
    dataFile.print(F(",V,"));
    dataFile.print(CurrentString);
    dataFile.print(F(","));
    dataFile.println(F("mA"));
    dataFile.close();
    //Serial.println(F(" logfile closed"));
  }
  
  return FileOpenSuccess;
}
