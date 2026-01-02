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

I will make an Arduino library for this firmware soon, but in the meantime, here is a simple function that types a given string:

```c
#define CMD_KBD 0xFE

void type(const char *s)
{
    uint16_t buf[128];
    char c, i;
    while(c = *s++) {
        uint8_t modifier = 0;
        uint8_t scancode = 0;

        // Letters
        if (c >= 'a' && c <= 'z') {
            scancode = 0x04 + (c - 'a');
        } else if (c >= 'A' && c <= 'Z') {
            scancode = 0x04 + (c - 'A');
            modifier = HID_MOD_LSHIFT;
        }

        // Numbers
        else if (c >= '1' && c <= '9') {
            scancode = 0x1E + (c - '1');
        } else if (c == '0') {
            scancode = 0x27;
        }

        // Shifted numbers / symbols
        else {
            switch (c) {
                case '!': scancode = 0x1E; modifier = HID_MOD_LSHIFT; break;
                case '@': scancode = 0x1F; modifier = HID_MOD_LSHIFT; break;
                case '#': scancode = 0x20; modifier = HID_MOD_LSHIFT; break;
                case '$': scancode = 0x21; modifier = HID_MOD_LSHIFT; break;
                case '%': scancode = 0x22; modifier = HID_MOD_LSHIFT; break;
                case '^': scancode = 0x23; modifier = HID_MOD_LSHIFT; break;
                case '&': scancode = 0x24; modifier = HID_MOD_LSHIFT; break;
                case '*': scancode = 0x25; modifier = HID_MOD_LSHIFT; break;
                case '(': scancode = 0x26; modifier = HID_MOD_LSHIFT; break;
                case ')': scancode = 0x27; modifier = HID_MOD_LSHIFT; break;

                case '\n': scancode = 0x28; break;

                // Punctuation
                case ' ': scancode = 0x2C; break;
                case '-': scancode = 0x2D; break;
                case '_': scancode = 0x2D; modifier = HID_MOD_LSHIFT; break;
                case '=': scancode = 0x2E; break;
                case '+': scancode = 0x2E; modifier = HID_MOD_LSHIFT; break;
                case '[': scancode = 0x2F; break;
                case '{': scancode = 0x2F; modifier = HID_MOD_LSHIFT; break;
                case ']': scancode = 0x30; break;
                case '}': scancode = 0x30; modifier = HID_MOD_LSHIFT; break;
                case '\\': scancode = 0x31; break;
                case '|': scancode = 0x31; modifier = HID_MOD_LSHIFT; break;
                case ';': scancode = 0x33; break;
                case ':': scancode = 0x33; modifier = HID_MOD_LSHIFT; break;
                case '\'': scancode = 0x34; break;
                case '"': scancode = 0x34; modifier = HID_MOD_LSHIFT; break;
                case '`': scancode = 0x35; break;
                case '~': scancode = 0x35; modifier = HID_MOD_LSHIFT; break;
                case ',': scancode = 0x36; break;
                case '<': scancode = 0x36; modifier = HID_MOD_LSHIFT; break;
                case '.': scancode = 0x37; break;
                case '>': scancode = 0x37; modifier = HID_MOD_LSHIFT; break;
                case '/': scancode = 0x38; break;
                case '?': scancode = 0x38; modifier = HID_MOD_LSHIFT; break;

                default:
                    return 0; // Not a printable ASCII character
            }
        }

        buf[i] = ((uint16_t)modifier << 8) | scancode;
        i++;
    }
    Serial.write(CMD_KBD); // Switch to keyboard mode
    Serial.write(i * 2); // Write length in bytes (2 bytes per keypress)
    for(int j = 0; j < i; j++) {
        Serial.write(buf[j] >> 8); // First byte modifier
        Serial.write(buf[j] & 0xFF); // Second byte scancode
    }
}
```

This code does several things. First, it iterates through the input string and converts each character to a scancode and modifier code. The reference for these codes can be found in the [USB HID Usage Tables](https://usb.org/sites/default/files/hut1_6.pdf), page 89. Modifier format is defined in [USB HID Device Class Definition](https://www.usb.org/sites/default/files/documents/hid1_11.pdf), page 66. Then, it sends the keyboard mode byte, 0xFE, over serial, immediately followed by the amount of bytes to follow which are keypresses/modifier bytes. Then, it sends each key/mod combo with the modifier byte first and the scancode second.

The firmware will act as standard USB-to-Serial outside of keyboard mode.