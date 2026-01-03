# Arduino USB-to-Serial and HID driver firmware

The object of this project is to provide firmware for the USB-to-Serial chip (commonly either and ATMega16U2 or ATMega8U2) found on the Arduino UNO/Mega 2560 boards which can act as both a standard USB-to-Serial bridge, and as a HID Keyboard driver controllable over serial, while still allowing the main chip to be programmed form the Arduino IDE.

# Building

Before building, edit the Makefile to set the BOARD_NAME variable to your target Arduino board, currently either ARDUINO_UNO or ARDUINO_MEGA. Also, set LUFA_PATH to the absolute path of the LUFA library, which can be found [here](https://github.com/abcminiuser/lufa).
To build, use `make`. To flash a board, follow these steps:
 - Plug the board in via USB
 - Put the board into DFU mode. This is done by shorting together the reset and ground pins on the ICSP header (the two pins closest to the USB connector):
 
 ![Image of the two pins closest to the USB connector on an Arduino Uno](https://europe1.discourse-cdn.com/arduino/original/4X/f/d/1/fd155e3afae2c4400a63baad8fd1e7fcaa022cc2.png)
 - Make sure you've built the project with `make`, and run `sudo -E make flash`.

# Usage

I also wrote an API library for Arduino IDE, that can be found at [warriorjac9-no2fa/arduino-firmware-api](https://github.com/warriorjacq9-no2fa/arduino-firmware-api)
