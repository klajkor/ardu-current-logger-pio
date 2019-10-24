# Arduino current logger with INA219 I2C module

Board: Arduino Nano

Extension modules used:
- INA219, I2C
- DS3231 RTC, I2C
- SSD1306 OLED dsiplay, I2C
- SD card, SPI

Libraries used:
- Adafruit INA219 by Adafruit - Copyright (c) 2012, Adafruit Industries
- SD by Adafruit Industries - Copyright (c) 2012, Adafruit Industries
- SSD1306Ascii by Bill Greiman - Copyright (c) 2019, Bill Greiman
- uRTCLib by Naguissa - Copyright (c) 2019, Naguissa

Arduino Nano pinout connections

I2C bus:
- SCK - pin A5
- SDA - pin A4

SD card on SPI bus:
- MOSI - pin D11
- MISO - pin D12
- CLK - pin D13
- CS - pin D4

Toolchain: VSCode + Platform.IO

Copyright (c) 2019, Robert Klajko
