/**
* Arduino current meter and logger
*
* @author Robert Klajko
* @url https://github.com/klajkor/ardu-current-logger-pio
*
* Board: Arduino Nano
*
* Extension modules used:
*  - INA219, I2C
*  - DS3231 RTC, I2C
*  - SSD1306 OLED dsiplay, I2C
*  - SD card, SPI
* Libraries used:
*  - Adafruit INA219 by Adafruit - Copyright (c) 2012, Adafruit Industries
*  - SD by Adafruit Industries - Copyright (c) 2012, Adafruit Industries
*  - SSD1306Ascii by Bill Greiman - Copyright (c) 2019, Bill Greiman
*  - uRTCLib by Naguissa - Copyright (c) 2019, Naguissa
*
* BSD license, all text here must be included in any redistribution.
*
*/

/** 
* Arduino Nano pinout connections
*
* I2C bus:
* - SCK - pin A5
* - SDA - pin A4
*
* SD card on SPI bus:
* - MOSI - pin D11
* - MISO - pin D12
* - CLK - pin D13
* - CS - pin D4
*/

/**
 * Toolchain: VSCode + Platform.IO 
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
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
const int OLED_I2C_ADDR = 0x3C; // Address 0x3C for 128x32

SSD1306AsciiWire display;

// Current and voltage sensor class
Adafruit_INA219 ina219_monitor;

// DS3231 RTC modul I2C address
const int RTC_I2C_addr = 0x68;
// RTC class
uRTCLib rtc(RTC_I2C_addr);

// SD card modul chip select
#define SDCARD_CHIP_SELECT 4


/* Global variables */

/* RTC global variables */
uint8_t rtc_second = 0;
uint8_t rtc_minute = 0;
uint8_t rtc_hour = 0;
uint8_t rtc_day = 0;
uint8_t rtc_month = 0;
uint8_t rtc_year = 0;

// Current sensor variables
float f_BusVoltage_V; /** Measured bus voltage */
float f_ShuntCurrent_mA; /** Measured shunt current */


// General variables
char DateStampString[] = "2000.99.88"; /** String to store date value */
char TimeStampString[] = "00:00:00"; /** String to store time value */
char logFileName[] = "mmddHHMM.txt"; /** String to store log file name */
char VoltString[] = "99.999 "; /** String to store measured voltage value */
char CurrentString[] = "9999.999 "; /** String to store measured current value */
bool SD_log_enabled = false; /** Enabling SD logging or not */

// Datafile class
File dataFile;


/* Function definitions */

bool Log_To_SD_card(void);
void setTimeStampString(void);


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
      #if SCREEN_HEIGHT == 32
    display.begin(&Adafruit128x32, I2C_ADDRESS, RST_PIN);
    #endif
    #if SCREEN_HEIGHT == 64
    display.begin(&Adafruit128x64, I2C_ADDRESS, RST_PIN);
    #endif
  #else // RST_PIN >= 0
    #if SCREEN_HEIGHT == 32
    display.begin(&Adafruit128x32, OLED_I2C_ADDR);
    #endif
    #if SCREEN_HEIGHT == 64
    display.begin(&Adafruit128x64, OLED_I2C_ADDR);
    #endif
  #endif // RST_PIN >= 0

  display.setFont(Adafruit5x7);
  //display.setFont(X11fixed7x14);

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
  
  display.setRow(0);
  display.setCol(0);
  display.set1X();
  
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
  dtostrf((f_ShuntCurrent_mA),7,2,CurrentString);
  dtostrf(f_BusVoltage_V,6,3,VoltString);
  
  //display volt
  Serial.print(VoltString);
  Serial.print(F(" V"));
  display.set2X();
  display.print(F("  "));
  display.print(VoltString);
  display.println(F(" V"));

  //display current
  Serial.print(CurrentString);
  Serial.println(F(" mA "));
  display.print(CurrentString);
  display.println(F(" mA"));

  display.set1X();
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

/**
* @brief Query date & time and set the [Date|Time]StampStrings & logFileName accordingly.
* @param void
*/
void setTimeStampString(void)
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

/**
* @brief Log the measurements with timestamp to SD card in CSV format.
* @param void
* @return bool FileOpenSuccess
*/
bool Log_To_SD_card(void)
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
  }
  
  return FileOpenSuccess;
}
