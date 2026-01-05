/*
  Copyright 2026  Jack Flusche (jackflusche@gmail.com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaims all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.

  This software is under the GNU AGPLv3 license,
  see the LICENSE file at project root
*/

/** \file
 *
 *  Main source file for the firmware project. This file contains the main tasks of
 *  the project and is responsible for the initial application hardware configuration.
 */

#include "firmware.h"

/** Circular buffer to hold data from the host before it is sent to the device via the serial port. */
static RingBuffer_t USBtoUSART_Buffer;

/** Underlying data buffer for \ref USBtoUSART_Buffer, where the stored bytes are located. */
static uint8_t      USBtoUSART_Buffer_Data[32];

/** Circular buffer to hold data from the serial port before it is sent to the host. */
static RingBuffer_t USARTtoUSB_Buffer;

/** Underlying data buffer for \ref USARTtoUSB_Buffer, where the stored bytes are located. */
static uint8_t      USARTtoUSB_Buffer_Data[32];

/** LED decay timers, decremented every 1ms */
static volatile uint8_t	LEDTimer_TX = 0;
static volatile uint8_t LEDTimer_RX = 0;

/** Buffer to hold the previously generated Keyboard HID report, for comparison purposes inside the HID class driver. */
static uint8_t PrevKeyboardHIDReportBuffer[sizeof(USB_KeyboardReport_Data_t)];

/** Circular buffer to hold data for the HID interface until it is acted on */
static RingBuffer_t USARTtoKBD_Buffer;
static uint8_t		USARTtoKBD_Buffer_Data[128];

/** Tracks the SET_IDLE state of the HID interface */
static volatile bool 	HIDIsIdle = false;

/** LUFA CDC Class driver interface configuration and state information. This structure is
 *  passed to all CDC Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_Interface =
	{
		.Config =
			{
				.ControlInterfaceNumber         = INTERFACE_ID_CDC_CCI,
				.DataINEndpoint                 =
					{
						.Address                = CDC_TX_EPADDR,
						.Size                   = CDC_TXRX_EPSIZE,
						.Banks                  = 1,
					},
				.DataOUTEndpoint                =
					{
						.Address                = CDC_RX_EPADDR,
						.Size                   = CDC_TXRX_EPSIZE,
						.Banks                  = 1,
					},
				.NotificationEndpoint           =
					{
						.Address                = CDC_NOTIFICATION_EPADDR,
						.Size                   = CDC_NOTIFICATION_EPSIZE,
						.Banks                  = 1,
					},
			},
	};

/** LUFA HID Class driver interface configuration and state information. This structure is
 *  passed to all HID Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_HID_Device_t Keyboard_HID_Interface =
	{
		.Config =
			{
				.InterfaceNumber              = INTERFACE_ID_HID,
				.ReportINEndpoint             =
					{
						.Address              = KBD_EPADDR,
						.Size                 = KBD_EPSIZE,
						.Banks                = 1,
					},
				.PrevReportINBuffer           = PrevKeyboardHIDReportBuffer,
				.PrevReportINBufferSize       = sizeof(PrevKeyboardHIDReportBuffer),
			},
	};

void timer1_init(void) {
    TCCR1A = 0;
    TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10); // CTC mode, Prescaler 64
    OCR1A = 249;                 // 1 ms (16MHz / 64 / 1000 - 1)
    TIMSK1 = (1 << OCIE1A);       // Enable compare interrupt
}

ISR(TIMER1_COMPA_vect) {
    if (LEDTimer_RX > 0) {
        LEDTimer_RX--;
		LEDs_TurnOnLEDs(LEDS_LED2);
    } else LEDs_TurnOffLEDs(LEDS_LED2);
    if (LEDTimer_TX > 0) {
        LEDTimer_TX--;
		LEDs_TurnOnLEDs(LEDS_LED1);
    } else LEDs_TurnOffLEDs(LEDS_LED1);
}

/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */
int main(void)
{
	SetupHardware();

	RingBuffer_InitBuffer(&USBtoUSART_Buffer, USBtoUSART_Buffer_Data, sizeof(USBtoUSART_Buffer_Data));
	RingBuffer_InitBuffer(&USARTtoUSB_Buffer, USARTtoUSB_Buffer_Data, sizeof(USARTtoUSB_Buffer_Data));
	RingBuffer_InitBuffer(&USARTtoKBD_Buffer, USARTtoKBD_Buffer_Data, sizeof(USARTtoKBD_Buffer_Data));

	LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
	GlobalInterruptEnable();

	for (;;)
	{
		/* Only try to read in bytes from the CDC interface if the transmit buffer is not full */
		if (!(RingBuffer_IsFull(&USBtoUSART_Buffer)))
		{
			int16_t ReceivedByte = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);

			/* Store received byte into the USART transmit buffer */
			if (!(ReceivedByte < 0)) {
				LEDTimer_RX = 5;
				RingBuffer_Insert(&USBtoUSART_Buffer, ReceivedByte);
			}
		}

		uint16_t BufferCount = RingBuffer_GetCount(&USARTtoUSB_Buffer);
		if (BufferCount)
		{
			Endpoint_SelectEndpoint(VirtualSerial_CDC_Interface.Config.DataINEndpoint.Address);

			/* Check if a packet is already enqueued to the host - if so, we shouldn't try to send more data
			 * until it completes as there is a chance nothing is listening and a lengthy timeout could occur */
			if (Endpoint_IsINReady())
			{
				/* Never send more than one bank size less one byte to the host at a time, so that we don't block
				 * while a Zero Length Packet (ZLP) to terminate the transfer is sent if the host isn't listening */
				uint8_t BytesToSend = MIN(BufferCount, (CDC_TXRX_EPSIZE - 1));

				/* Read bytes from the USART receive buffer into the USB IN endpoint */
				while (BytesToSend--)
				{
					/* Try to send the next byte of data to the host, abort if there is an error without dequeuing */
					if (CDC_Device_SendByte(&VirtualSerial_CDC_Interface,
											RingBuffer_Peek(&USARTtoUSB_Buffer)) != ENDPOINT_READYWAIT_NoError)
					{
						break;
					}

					/* Dequeue the already sent byte from the buffer now we have confirmed that no transmission error occurred */
					RingBuffer_Remove(&USARTtoUSB_Buffer);
				}
			}
		}

		/* Load the next byte from the USART transmit buffer into the USART if transmit buffer space is available */
		if (Serial_IsSendReady() && !(RingBuffer_IsEmpty(&USBtoUSART_Buffer)))
		  	Serial_SendByte(RingBuffer_Remove(&USBtoUSART_Buffer));

		HID_Device_USBTask(&Keyboard_HID_Interface);
		CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
		USB_USBTask();
	}
}

/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware(void)
{
#if (ARCH == ARCH_AVR8)
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	/* Disable clock division */
	clock_prescale_set(clock_div_1);
#endif

	/* Hardware Initialization */
	Serial_Init(9600, false);
	timer1_init();
	LEDs_Init();
	USB_Init();

	/* Initialize USART1 */
	/* Keep the TX line held high (idle) while the USART is reconfigured */
	PORTD |= (1 << 3);

	/* Must turn off USART before reconfiguring it, otherwise incorrect operation may occur */
	UCSR1B = 0;
	UCSR1A = 0;
	UCSR1C = 0;

	// 9600 baud default
	UBRR1  = SERIAL_2X_UBBRVAL(9600);

	UCSR1C = (1 << UCSZ11) | (1 << UCSZ10);
	UCSR1A = (1 << U2X1);
	UCSR1B = (1 << RXCIE1) | (1 << TXEN1) | (1 << RXEN1);

	/* Release the TX line after the USART has been reconfigured */
	PORTD &= ~(1 << 3);


	/* Pull target /RESET line high */
	PORTD |= (1 << 7);
	DDRD  |= (1 << 7);
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
    bool ConfigSuccess = true;

    ConfigSuccess &= CDC_Device_ConfigureEndpoints(&VirtualSerial_CDC_Interface);
    bool HIDSuccess = HID_Device_ConfigureEndpoints(&Keyboard_HID_Interface);
    ConfigSuccess &= HIDSuccess;
    
    if (!HIDSuccess || !ConfigSuccess) {
        LEDs_SetAllLEDs(LEDS_LED1 | LEDS_LED3);
    }
    
    USB_Device_EnableSOFEvents();
}


/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
	// Handle HID-specific requests first
	if ((USB_ControlRequest.bmRequestType & CONTROL_REQTYPE_RECIPIENT) == REQREC_INTERFACE)
	{
		// Check if request is for HID interface
		if ((USB_ControlRequest.wIndex & 0xFF) == INTERFACE_ID_HID)
		{
			switch (USB_ControlRequest.bRequest)
			{
				case HID_REQ_SetIdle:
					HIDIsIdle = true;
					Endpoint_ClearSETUP();
					Endpoint_ClearStatusStage();
					return;
			}
		}
	}
	
	// Let class drivers handle their requests
	HID_Device_ProcessControlRequest(&Keyboard_HID_Interface);
	CDC_Device_ProcessControlRequest(&VirtualSerial_CDC_Interface);
}

/** Event handler for the USB device Start Of Frame event. */
void EVENT_USB_Device_StartOfFrame(void)
{
    HID_Device_MillisecondElapsed(&Keyboard_HID_Interface);
}

/** HID class driver callback function for the processing of HID reports from the host.
 *
 *  \param[in] HIDInterfaceInfo  Pointer to the HID class interface configuration structure being referenced
 *  \param[in] ReportID    Report ID of the received report from the host
 *  \param[in] ReportType  The type of report that the host has sent, either HID_REPORT_ITEM_Out or HID_REPORT_ITEM_Feature
 *  \param[in] ReportData  Pointer to a buffer where the received report has been stored
 *  \param[in] ReportSize  Size in bytes of the received HID report
 */
void CALLBACK_HID_Device_ProcessHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
                                          const uint8_t ReportID,
                                          const uint8_t ReportType,
                                          const void* ReportData,
                                          const uint16_t ReportSize)
{
	uint8_t  LEDMask   = LEDS_NO_LEDS;
	uint8_t* LEDReport = (uint8_t*)ReportData;

	if (*LEDReport & HID_KEYBOARD_LED_NUMLOCK)
	  LEDMask |= LEDS_LED3;

	if (*LEDReport & HID_KEYBOARD_LED_CAPSLOCK)
	  LEDMask |= LEDS_LED4;

	LEDs_ChangeLEDs(LEDS_LED3 | LEDS_LED4, LEDMask);
}

enum USART_State {
	STATE_IDLE,
	STATE_S_GETLEN,
	STATE_PROCESS,
};

enum HID_State {
	STATE_STRING,
	STATE_REPORT,
};

static enum USART_State ustate = STATE_IDLE;
static enum HID_State hstate = STATE_STRING;

/** The amount of keyboard commands remaining on serial */
static uint8_t 		KBDCommandCount = 0;
static bool 		SentInitial = false;

/** HID class driver callback function for the creation of HID reports to the host.
 *
 *  \param[in]     HIDInterfaceInfo  Pointer to the HID class interface configuration structure being referenced
 *  \param[in,out] ReportID    Report ID requested by the host if non-zero, otherwise callback should set to the generated report ID
 *  \param[in]     ReportType  Type of the report to create, either HID_REPORT_ITEM_In or HID_REPORT_ITEM_Feature
 *  \param[out]    ReportData  Pointer to a buffer where the created report should be stored
 *  \param[out]    ReportSize  Number of bytes written in the report (or zero if no report is to be sent)
 *
 *  \return Boolean \c true to force the sending of the report, \c false to let the library determine if it needs to be sent
 */
bool CALLBACK_HID_Device_CreateHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
                                         uint8_t* const ReportID,
                                         const uint8_t ReportType,
                                         void* ReportData,
                                         uint16_t* const ReportSize)
{
	USB_KeyboardReport_Data_t* KeyboardReport = (USB_KeyboardReport_Data_t*)ReportData;
	memset(KeyboardReport, 0, sizeof(*KeyboardReport));

	if (!HIDIsIdle) {
		*ReportSize = 0;
		return false;
	} else if (!SentInitial) {
		*ReportSize = sizeof(USB_KeyboardReport_Data_t);
		SentInitial = true;
		return true;
	}

	uint16_t BufferCount = RingBuffer_GetCount(&USARTtoKBD_Buffer);
	
	switch(hstate) {
		case STATE_STRING:
			if(BufferCount >= 2) {
				KeyboardReport->Modifier = RingBuffer_Peek(&USARTtoKBD_Buffer);
				RingBuffer_Remove(&USARTtoKBD_Buffer);
				KeyboardReport->KeyCode[0] = RingBuffer_Peek(&USARTtoKBD_Buffer);
				RingBuffer_Remove(&USARTtoKBD_Buffer);
			}
			break;
		case STATE_REPORT:
			if(BufferCount >= 7) {
				KeyboardReport->Modifier = RingBuffer_Peek(&USARTtoKBD_Buffer);
				RingBuffer_Remove(&USARTtoKBD_Buffer);
				for(int i = 0; i < 6; i++) {
					KeyboardReport->KeyCode[i] = RingBuffer_Peek(&USARTtoKBD_Buffer);
					RingBuffer_Remove(&USARTtoKBD_Buffer);
				}
			}
			break;
	}

	*ReportSize = sizeof(USB_KeyboardReport_Data_t);
	return true;
}

/** ISR to manage the reception of data from the serial port, placing received bytes into a circular buffer
 *  for later transmission to the host.
 */
ISR(USART1_RX_vect, ISR_BLOCK)
{
	uint8_t ReceivedByte = UDR1;

	switch(ustate){
		case STATE_IDLE:
			if (ReceivedByte == CMD_KBD_STRING) {
				ustate = STATE_S_GETLEN;
				hstate = STATE_STRING;
				break;
			} else if (ReceivedByte == CMD_KBD_REPORT) {
				KBDCommandCount = 7;
				ustate = STATE_PROCESS;
				hstate = STATE_REPORT;
				break;
			}
			if ((USB_DeviceState == DEVICE_STATE_Configured) && !(RingBuffer_IsFull(&USARTtoUSB_Buffer))) {
				// Flash TX
				LEDTimer_TX = 5;
				RingBuffer_Insert(&USARTtoUSB_Buffer, ReceivedByte);
			}
			break;
		case STATE_S_GETLEN:
			KBDCommandCount = MIN(ReceivedByte, sizeof(USARTtoKBD_Buffer_Data));
			ustate = STATE_PROCESS;
			break;
		case STATE_PROCESS:
			if(!RingBuffer_IsFull(&USARTtoKBD_Buffer)) {
				LEDTimer_TX = 5;
				RingBuffer_Insert(&USARTtoKBD_Buffer, ReceivedByte);

				if(KBDCommandCount > 0 && --KBDCommandCount == 0) ustate = STATE_IDLE;
			}
			break;
	}
}

/** Event handler for the CDC Class driver Line Encoding Changed event.
 *
 *  \param[in] CDCInterfaceInfo  Pointer to the CDC class interface configuration structure being referenced
 */
void EVENT_CDC_Device_LineEncodingChanged(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo)
{
	uint8_t ConfigMask = 0;

	switch (CDCInterfaceInfo->State.LineEncoding.ParityType)
	{
		case CDC_PARITY_Odd:
			ConfigMask = ((1 << UPM11) | (1 << UPM10));
			break;
		case CDC_PARITY_Even:
			ConfigMask = (1 << UPM11);
			break;
	}

	if (CDCInterfaceInfo->State.LineEncoding.CharFormat == CDC_LINEENCODING_TwoStopBits)
	  ConfigMask |= (1 << USBS1);

	switch (CDCInterfaceInfo->State.LineEncoding.DataBits)
	{
		case 6:
			ConfigMask |= (1 << UCSZ10);
			break;
		case 7:
			ConfigMask |= (1 << UCSZ11);
			break;
		case 8:
			ConfigMask |= ((1 << UCSZ11) | (1 << UCSZ10));
			break;
	}

	/* Keep the TX line held high (idle) while the USART is reconfigured */
	PORTD |= (1 << 3);

	/* Must turn off USART before reconfiguring it, otherwise incorrect operation may occur */
	UCSR1B = 0;
	UCSR1A = 0;
	UCSR1C = 0;

	/* Special case 57600 baud for compatibility with the ATmega328 bootloader. */	
	UBRR1  = (CDCInterfaceInfo->State.LineEncoding.BaudRateBPS == 57600)
			 ? SERIAL_UBBRVAL(CDCInterfaceInfo->State.LineEncoding.BaudRateBPS)
			 : SERIAL_2X_UBBRVAL(CDCInterfaceInfo->State.LineEncoding.BaudRateBPS);	

	UCSR1C = ConfigMask;
	UCSR1A = (CDCInterfaceInfo->State.LineEncoding.BaudRateBPS == 57600) ? 0 : (1 << U2X1);
	UCSR1B = ((1 << RXCIE1) | (1 << TXEN1) | (1 << RXEN1));

	/* Release the TX line after the USART has been reconfigured */
	PORTD &= ~(1 << 3);
}

void EVENT_CDC_Device_ControLineStateChanged(
    USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo)
{
    bool CurrentDTRState =
        (CDCInterfaceInfo->State.ControlLineStates.HostToDevice & CDC_CONTROL_LINE_OUT_DTR);

    if (CurrentDTRState) {
        PORTD &= ~(1 << 7);   // DTR asserted = RESET low
		LEDTimer_TX = 5;
	} else {
        PORTD |= (1 << 7);    // DTR deasserted = RESET high
		LEDTimer_RX = 5;
	}
}