/* 
 * File:   hidmedia.c
 * Author: mark
 *
 * Created on May 1, 2013, 9:20 PM
 *
 * This code is derived from Microchip USB HID Keyboard example code,
 * customised and refactored to work as the MWJ Media Keyboard.
 *
 * Copyright (C) 2013 Mark Webb-Johnson
 * Portions probably copyright Microchip Technology Inc.
 *
 * THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 * TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 * IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 */

#ifndef KEYBOARD_C
#define KEYBOARD_C

/** INCLUDES *******************************************************/
#include "usb_config.h"
#include "./USB/usb.h"
#include "HardwareProfile.h"
#include "./USB/usb_function_hid.h"

/** CONFIGURATION **************************************************/

// PIC18F4550/PIC18F2550 configuration for the MWJ Media Controller
#pragma config PLLDIV   = 5         // 20Mhz external oscillator
#pragma config CPUDIV   = OSC1_PLL2
#pragma config USBDIV   = 2         // Clock source from 96MHz PLL/2
#pragma config FOSC     = HSPLL_HS
#pragma config FCMEN    = OFF
#pragma config IESO     = OFF
#pragma config PWRT     = OFF
#pragma config BOR      = ON
#pragma config BORV     = 3
#pragma config VREGEN   = ON
#pragma config WDT      = OFF
#pragma config WDTPS    = 32768
#pragma config MCLRE    = ON
#pragma config LPT1OSC  = OFF
#pragma config PBADEN   = OFF
// #pragma config CCP2MX   = ON
#pragma config STVREN   = ON
#pragma config LVP      = OFF
// #pragma config ICPRT    = OFF
#pragma config XINST    = OFF
#pragma config CP0      = OFF
#pragma config CP1      = OFF
// #pragma config CP2      = OFF
// #pragma config CP3      = OFF
#pragma config CPB      = OFF
// #pragma config CPD      = OFF
#pragma config WRT0     = OFF
#pragma config WRT1     = OFF
// #pragma config WRT2     = OFF
// #pragma config WRT3     = OFF
#pragma config WRTB     = OFF
#pragma config WRTC     = OFF
// #pragma config WRTD     = OFF
#pragma config EBTR0    = OFF
#pragma config EBTR1    = OFF
// #pragma config EBTR2    = OFF
// #pragma config EBTR3    = OFF
#pragma config EBTRB    = OFF

/** VARIABLES ******************************************************/
//The ReceivedDataBuffer[] and ToSendDataBuffer[] arrays are used as
//USB packet buffers in this firmware.  Therefore, they must be located in
//a USB module accessible portion of microcontroller RAM.
#pragma udata USB_VARIABLES=0x500
#define IN_DATA_BUFFER_ADDRESS_TAG
#define OUT_DATA_BUFFER_ADDRESS_TAG

unsigned char hid_report_in[HID_INT_IN_EP_SIZE] IN_DATA_BUFFER_ADDRESS_TAG;
volatile unsigned char hid_report_out[HID_INT_OUT_EP_SIZE] OUT_DATA_BUFFER_ADDRESS_TAG;

#pragma udata
USB_HANDLE lastINTransmission;
USB_HANDLE lastOUTTransmission;

unsigned long keypresses_counter;
BYTE keypresses,keypresses_spending,keypresses_lpending,keypresses_store;
typedef struct {
  unsigned dpadDown:1;          // 0x01
  unsigned dpadLeft:1;          // 0x02
  unsigned dpadEnter:1;         // 0x04
  unsigned dpadRight:1;         // 0x08
  unsigned topRight:1;          // 0x10
  unsigned dpadUp:1;            // 0x20
  unsigned topLeft:1;           // 0x40
  unsigned mode:1;              // 0x80
} keypresses_t;
#define keypressBits (*((keypresses_t*)&keypresses))
#define keypressShortPendingBits (*((keypresses_t*)&keypresses_spending))
#define keypressLongPendingBits (*((keypresses_t*)&keypresses_lpending))

BYTE mode;
BYTE idler;

/** PRIVATE PROTOTYPES *********************************************/
BOOL modeIsPressed(void);
BOOL dpadUpIsPressed(void);
BOOL dpadDownIsPressed(void);
BOOL dpadLeftIsPressed(void);
BOOL dpadRightIsPressed(void);
BOOL dpadEnterIsPressed(void);
BOOL topLeftIsPressed(void);
BOOL topRightIsPressed(void);
BOOL modeLongPressed(void);
BOOL dpadUpLongPressed(void);
BOOL dpadDownLongPressed(void);
BOOL dpadLeftLongPressed(void);
BOOL dpadRightLongPressed(void);
BOOL dpadEnterLongPressed(void);
BOOL topLeftLongPressed(void);
BOOL topRightLongPressed(void);

static void InitializeSystem(void);
void ProcessIO(void);
void UserInit(void);
void YourHighPriorityISRCode();
void YourLowPriorityISRCode();
void USBCBSendResume(void);
void hid_txkey(BYTE);
void hid_txkey2(BYTE, BYTE);
void hid_txconsumer(BYTE);

void KeyboardMode0(void);
void KeyboardMode1(void);

void KeyboardDebouncer(void);

void USBHIDCBSetReportComplete(void);

/** VECTOR REMAPPING ***********************************************/
#if defined(__18CXX)
//On PIC18 devices, addresses 0x00, 0x08, and 0x18 are used for
//the reset, high priority interrupt, and low priority interrupt
//vectors.  However, the current Microchip USB bootloader
//examples are intended to occupy addresses 0x00-0x7FF or
//0x00-0xFFF depending on which bootloader is used.  Therefore,
//the bootloader code remaps these vectors to new locations
//as indicated below.  This remapping is only necessary if you
//wish to program the hex file generated from this project with
//the USB bootloader.  If no bootloader is used, edit the
//usb_config.h file and comment out the following defines:
//#define PROGRAMMABLE_WITH_USB_HID_BOOTLOADER
//#define PROGRAMMABLE_WITH_USB_LEGACY_CUSTOM_CLASS_BOOTLOADER

#if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)
#define REMAPPED_RESET_VECTOR_ADDRESS			0x1000
#define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS	0x1008
#define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS	0x1018
#elif defined(PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER)
#define REMAPPED_RESET_VECTOR_ADDRESS			0x800
#define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS	0x808
#define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS	0x818
#else
#define REMAPPED_RESET_VECTOR_ADDRESS			0x00
#define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS	0x08
#define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS	0x18
#endif

#if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)||defined(PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER)
extern void _startup(void); // See c018i.c in your C18 compiler dir
#pragma code REMAPPED_RESET_VECTOR = REMAPPED_RESET_VECTOR_ADDRESS

void _reset(void)
{
  _asm goto _startup _endasm
}
#endif
#pragma code REMAPPED_HIGH_INTERRUPT_VECTOR = REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS

void Remapped_High_ISR(void)
{
  _asm goto YourHighPriorityISRCode _endasm
}
#pragma code REMAPPED_LOW_INTERRUPT_VECTOR = REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS

void Remapped_Low_ISR(void)
{
  _asm goto YourLowPriorityISRCode _endasm
}

#if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)||defined(PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER)
//Note: If this project is built while one of the bootloaders has
//been defined, but then the output hex file is not programmed with
//the bootloader, addresses 0x08 and 0x18 would end up programmed with 0xFFFF.
//As a result, if an actual interrupt was enabled and occured, the PC would jump
//to 0x08 (or 0x18) and would begin executing "0xFFFF" (unprogrammed space).  This
//executes as nop instructions, but the PC would eventually reach the REMAPPED_RESET_VECTOR_ADDRESS
//(0x1000 or 0x800, depending upon bootloader), and would execute the "goto _startup".  This
//would effective reset the application.

//To fix this situation, we should always deliberately place a
//"goto REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS" at address 0x08, and a
//"goto REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS" at address 0x18.  When the output
//hex file of this project is programmed with the bootloader, these sections do not
//get bootloaded (as they overlap the bootloader space).  If the output hex file is not
//programmed using the bootloader, then the below goto instructions do get programmed,
//and the hex file still works like normal.  The below section is only required to fix this
//scenario.
#pragma code HIGH_INTERRUPT_VECTOR = 0x08

void High_ISR(void)
{
  _asm goto REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS _endasm
}
#pragma code LOW_INTERRUPT_VECTOR = 0x18

void Low_ISR(void)
{
  _asm goto REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS _endasm
}
#endif	//end of "#if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)||defined(PROGRAMMABLE_WITH_USB_LEGACY_CUSTOM_CLASS_BOOTLOADER)"

#pragma code


//These are your actual interrupt handling routines.
#pragma interrupt YourHighPriorityISRCode

void YourHighPriorityISRCode()
{
  //Check which interrupt flag caused the interrupt.
  //Service the interrupt
  //Clear the interrupt flag
  //Etc.
#if defined(USB_INTERRUPT)
  USBDeviceTasks();
#endif

} //This return will be a "retfie fast", since this is in a #pragma interrupt section
#pragma interruptlow YourLowPriorityISRCode

void YourLowPriorityISRCode()
{
  //Check which interrupt flag caused the interrupt.
  //Service the interrupt
  //Clear the interrupt flag
  //Etc.

} //This return will be a "retfie", since this is in a #pragma interruptlow section
#endif


/** DECLARATIONS ***************************************************/
#if defined(__18CXX)
#pragma code
#endif

/********************************************************************
 * Function:        void main(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Main program entry point.
 *
 * Note:            None
 *******************************************************************/
#if defined(__18CXX)
void main(void)
#else
int main(void)
#endif
{
  InitializeSystem();

#if defined(USB_INTERRUPT)
  USBDeviceAttach();
#endif

  while (1) {
#if defined(USB_POLLING)
    // Check bus status and service USB interrupts.
    USBDeviceTasks(); // Interrupt or polling method.  If using polling, must call
    // this function periodically.  This function will take care
    // of processing and responding to SETUP transactions
    // (such as during the enumeration process when you first
    // plug in).  USB hosts require that USB devices should accept
    // and process SETUP packets in a timely fashion.  Therefore,
    // when using polling, this function should be called
    // regularly (such as once every 1.8ms or faster** [see
    // inline code comments in usb_device.c for explanation when
    // "or faster" applies])  In most cases, the USBDeviceTasks()
    // function does not take very long to execute (ex: <100
    // instruction cycles) before it returns.
#endif

    // Application-specific tasks.
    // Application related code may be added here, or in the ProcessIO() function.
    ProcessIO();
  }//end while
}//end main

/********************************************************************
 * Function:        static void InitializeSystem(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        InitializeSystem is a centralize initialization
 *                  routine. All required USB initialization routines
 *                  are called from here.
 *
 *                  User application initialization routine should
 *                  also be called from here.
 *
 * Note:            None
 *******************************************************************/
static void InitializeSystem(void)
{
  ADCON1 |= 0x0F; // Default all pins to digital

  //	The USB specifications require that USB peripheral devices must never source
  //	current onto the Vbus pin.  Additionally, USB peripherals should not source
  //	current on D+ or D- when the host/hub is not actively powering the Vbus line.
  //	When designing a self powered (as opposed to bus powered) USB peripheral
  //	device, the firmware should make sure not to turn on the USB module and D+
  //	or D- pull up resistor unless Vbus is actively powered.  Therefore, the
  //	firmware needs some means to detect when Vbus is being powered by the host.
  //	A 5V tolerant I/O pin can be connected to Vbus (through a resistor), and
  // 	can be used to detect when Vbus is high (host actively powering), or low
  //	(host is shut down or otherwise not supplying power).  The USB firmware
  // 	can then periodically poll this I/O pin to know when it is okay to turn on
  //	the USB module/D+/D- pull up resistor.  When designing a purely bus powered
  //	peripheral device, it is not possible to source current on D+ or D- when the
  //	host is not actively providing power on Vbus. Therefore, implementing this
  //	bus sense feature is optional.  This firmware can be made to use this bus
  //	sense feature by making sure "USE_USB_BUS_SENSE_IO" has been defined in the
  //	HardwareProfile.h file.
#if defined(USE_USB_BUS_SENSE_IO)
  tris_usb_bus_sense = INPUT_PIN; // See HardwareProfile.h
#endif

  //	If the host PC sends a GetStatus (device) request, the firmware must respond
  //	and let the host know if the USB peripheral device is currently bus powered
  //	or self powered.  See chapter 9 in the official USB specifications for details
  //	regarding this request.  If the peripheral device is capable of being both
  //	self and bus powered, it should not return a hard coded value for this request.
  //	Instead, firmware should check if it is currently self or bus powered, and
  //	respond accordingly.  If the hardware has been configured like demonstrated
  //	on the PICDEM FS USB Demo Board, an I/O pin can be polled to determine the
  //	currently selected power source.  On the PICDEM FS USB Demo Board, "RA2"
  //	is used for	this purpose.  If using this feature, make sure "USE_SELF_POWER_SENSE_IO"
  //	has been defined in HardwareProfile - (platform).h, and that an appropriate I/O pin
  //  has been mapped	to it.
#if defined(USE_SELF_POWER_SENSE_IO)
  tris_self_power = INPUT_PIN; // See HardwareProfile.h
#endif

  UserInit();

  USBDeviceInit(); //usb_device.c.  Initializes USB module SFRs and firmware
  //variables to known states.
}//end InitializeSystem

/******************************************************************************
 * Function:        void UserInit(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This routine should take care of all of the demo code
 *                  initialization that is required.
 *
 * Note:
 *
 *****************************************************************************/
void UserInit(void)
{
  //Initialize all of the LED pins
  TRISA |= 0b00011111; // Ensure RA0..RA4 are INPUTs
  TRISA &= 0b11011111; // Ensure RA5 is OUTPUT
  TRISC |= 0b00000111; // Ensrue RC0..RC2 are INPUTs

  keypresses = 0;
  keypresses_spending = 0;
  keypresses_lpending = 0;
  keypresses_store = 0;
  keypresses_counter = 0;
  mode = 0;
  idler = 0;

  mInitAllLEDs();

  //Initialize all of the push buttons
  //mInitAllSwitches();
  //old_sw2 = sw2;
  //old_sw3 = sw3;

  //initialize the variable holding the handle for the last
  // transmission

  lastINTransmission = 0;
  lastOUTTransmission = 0;

}//end UserInit

/********************************************************************
 * Function:        void ProcessIO(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is a place holder for other user
 *                  routines. It is a mixture of both USB and
 *                  non-USB tasks.
 *
 * Note:            None
 *******************************************************************/
void ProcessIO(void)
{
  KeyboardDebouncer();

  // User Application USB tasks
  if ((USBDeviceState < CONFIGURED_STATE) || (USBSuspendControl == 1)) return;

  switch (mode)
  {
  case 0:
    KeyboardMode0();
    break;
  case 1:
    KeyboardMode1();
    break;
  }

  if (!HIDTxHandleBusy(lastINTransmission)) {
    if ((idler & 0b11111110) == 0)
      hid_txconsumer(0x00); // No media key pressed
    else
      hid_txkey(0x00); // No key pressed
    idler++;
  }

}//end ProcessIO

void hid_txkey(BYTE key)
{
  if (key == 0)
    mLED_1_Off()
  else
    mLED_1_On();

  hid_report_in[0] = 1; //Report ID=1
  hid_report_in[1] = 0; //ModiferKey
  hid_report_in[2] = 0; //Reserved
  hid_report_in[3] = key; //HID keycode
  hid_report_in[4] = 0;
  hid_report_in[5] = 0;
  hid_report_in[6] = 0;
  hid_report_in[7] = 0;
  hid_report_in[8] = 0;
  lastINTransmission = HIDTxPacket(HID_EP, (BYTE*) hid_report_in, 0x09);
}

void hid_txkey2(BYTE key1, BYTE key2)
{
  if (key1 == 0)
    mLED_1_Off()
  else
    mLED_1_On();

  hid_report_in[0] = 1; //Report ID=1
  hid_report_in[1] = 0; //ModiferKey
  hid_report_in[2] = 0; //Reserved
  hid_report_in[3] = key1; //HID keycode
  hid_report_in[4] = key2; //HID keycode
  hid_report_in[5] = 0;
  hid_report_in[6] = 0;
  hid_report_in[7] = 0;
  hid_report_in[8] = 0;
  lastINTransmission = HIDTxPacket(HID_EP, (BYTE*) hid_report_in, 0x09);
}

void hid_txconsumer(BYTE key)
{
  if (key == 0)
    mLED_1_Off()
  else
    mLED_1_On();
  
  hid_report_in[0] = 2; //Report ID=2
  hid_report_in[1] = key; // HID keycode
  lastINTransmission = HIDTxPacket(HID_EP, (BYTE*) hid_report_in, 0x02);
}

void KeyboardMode0(void)
{
  //Check if the IN endpoint is not busy, and if it isn't check if we want to send
  //keystroke data to the host.
  // (MWJ) bit format: 7 reserved, 6 vol down, 5 vol up, 4 mute, 3 play/pause, 2 stop, 1 prev track, 0 next track
  // (MWJ) For example, 0x40 for volume down and 0x20 for volume up.
  if (!HIDTxHandleBusy(lastINTransmission)) {

    if (modeIsPressed()||modeLongPressed()) {
      hid_txkey2(0x65,0x1E); // SEARCH+1
      mode = 1;
      }
    else if (dpadLeftIsPressed()) {
      hid_txkey2(0x65,0x13); // SEARCH+p = Previous Song
      }
    else if (dpadRightIsPressed()) {
      hid_txkey2(0x65,0x11); // SEARCH+n = Next Song
      }
    else if (dpadUpIsPressed()) {
      hid_txkey2(0x65,0x04); // SEARCH+a = Music App #1
      }
    else if (dpadDownIsPressed()) {
      hid_txkey2(0x65,0x05); // SEARCH+b = Music App #2
      }
    else if (dpadEnterIsPressed()) {
      hid_txkey2(0x65,0x16); // SEARCH+s = Play/Pause
      }
    else if (dpadEnterLongPressed()) {
      hid_txkey2(0x65,0x19); // SEARCH+v = Voice Control
      }
    else if (topLeftIsPressed()) {
      hid_txconsumer(0x40); // Volume Down
      }
    else if (topRightIsPressed()) {
      hid_txconsumer(0x20); // Volume Up
      }
    else if (topLeftLongPressed()|topRightLongPressed()) {
      hid_txconsumer(0x10); // Mute
      }
    }


  //Check if any data was sent from the PC to the keyboard device.  Report descriptor allows
  //host to send 1 byte of data.  Bits 0-4 are LED states, bits 5-7 are unused pad bits.
  //The host can potentially send this OUT report data through the HID OUT endpoint (EP1 OUT),
  //or, alternatively, the host may try to send LED state information by sending a
  //SET_REPORT control transfer on EP0.  See the USBHIDCBSetReportHandler() function.
  if (!HIDRxHandleBusy(lastOUTTransmission)) {
    lastOUTTransmission = HIDRxPacket(HID_EP, (BYTE*) & hid_report_out, 1);
    }

  return;
}

void KeyboardMode1(void)
{
  //Check if the IN endpoint is not busy, and if it isn't check if we want to send
  //keystroke data to the host.
  if (!HIDTxHandleBusy(lastINTransmission)) {

    if (modeIsPressed()||modeLongPressed()) {
      hid_txkey2(0x65,0x27); // SEARCH+0
      mode = 0;
      }
    else if (dpadLeftIsPressed()) {
      hid_txkey(0x50); // Left Arrow
      }
    else if (dpadRightIsPressed()) {
      hid_txkey(0x4F); // Right Arrow
      }
    else if (dpadUpIsPressed()) {
      hid_txkey(0x52); // Up Arrow
      }
    else if (dpadDownIsPressed()) {
      hid_txkey(0x51); // Down Arrow
      }
    else if (dpadEnterIsPressed()) {
      hid_txkey(0x28); // ENTER
      }
    else if (dpadEnterLongPressed()) {
      hid_txkey2(0x65,0x19); // SEARCH+v = Voice Control
      }
    else if (topLeftIsPressed()) {
      hid_txkey(0x29); // ESC
      }
    else if (topLeftLongPressed()||topRightLongPressed()) {
      hid_txkey(0x4A); // HOME
      }
    else if (topRightIsPressed()) {
      hid_txkey(0x3A); // MENU
      }
    }


  //Check if any data was sent from the PC to the keyboard device.  Report descriptor allows
  //host to send 1 byte of data.  Bits 0-4 are LED states, bits 5-7 are unused pad bits.
  //The host can potentially send this OUT report data through the HID OUT endpoint (EP1 OUT),
  //or, alternatively, the host may try to send LED state information by sending a
  //SET_REPORT control transfer on EP0.  See the USBHIDCBSetReportHandler() function.
  if (!HIDRxHandleBusy(lastOUTTransmission)) {
    lastOUTTransmission = HIDRxPacket(HID_EP, (BYTE*) & hid_report_out, 1);
  }

  return;
}

// Debounce logic
// If a key is pressed
//   At level 1, store it and wait for release
//   At level 2, treat it as longpress, set it and wait for release
// If a key is released
//   If store != 0, that is our result

#define DEBOUNCE_LEVEL1 1000L
#define DEBOUNCE_LEVEL2 75000L
#define DEBOUNCE_LEVEL3 600000L

void KeyboardDebouncer(void)
{
  keypresses = (PORTC << 5) + (PORTA & 0b00011111);

  if (keypresses != 0xff) {
    // One or more keys are pressed

    if (keypresses_counter >= DEBOUNCE_LEVEL3) {
      return; // Special case - wait for key release
      }
    else if (keypresses_counter > DEBOUNCE_LEVEL2) {
      // It has been held for a long press
      keypresses_lpending = keypresses ^ 0xff;
      keypresses_store = 0;
      keypresses_counter = DEBOUNCE_LEVEL3;
      }
    else if (keypresses_counter > DEBOUNCE_LEVEL1) {
      // It has been held for a short press
      keypresses_store = keypresses ^ 0xff;
      keypresses_counter++;
      }
    else {
      keypresses_counter++;
      }
    }
  else {
    if (keypresses_store != 0) {
      keypresses_spending = keypresses_store;
      keypresses_store = 0;
      }
    keypresses_counter = 0;
  }
}

BOOL modeIsPressed(void)
{
  if (keypressShortPendingBits.mode) {
    keypressShortPendingBits.mode = 0;
    return TRUE;
  }
  return FALSE;
}

BOOL dpadUpIsPressed(void)
{
  if (keypressShortPendingBits.dpadUp) {
    keypressShortPendingBits.dpadUp = 0;
    return TRUE;
  }
  return FALSE;
}

BOOL dpadDownIsPressed(void)
{
  if (keypressShortPendingBits.dpadDown) {
    keypressShortPendingBits.dpadDown = 0;
    return TRUE;
  }
  return FALSE;
}

BOOL dpadLeftIsPressed(void)
{
  if (keypressShortPendingBits.dpadLeft) {
    keypressShortPendingBits.dpadLeft = 0;
    return TRUE;
  }
  return FALSE;
}

BOOL dpadRightIsPressed(void)
{
  if (keypressShortPendingBits.dpadRight) {
    keypressShortPendingBits.dpadRight = 0;
    return TRUE;
  }
  return FALSE;
}

BOOL dpadEnterIsPressed(void)
{
  if (keypressShortPendingBits.dpadEnter) {
    keypressShortPendingBits.dpadEnter = 0;
    return TRUE;
  }
  return FALSE;
}

BOOL topLeftIsPressed(void)
{
  if (keypressShortPendingBits.topLeft) {
    keypressShortPendingBits.topLeft = 0;
    return TRUE;
  }
  return FALSE;
}

BOOL topRightIsPressed(void)
{
  if (keypressShortPendingBits.topRight) {
    keypressShortPendingBits.topRight = 0;
    return TRUE;
  }
  return FALSE;
}

BOOL modeLongPressed(void)
{
  if (keypressLongPendingBits.mode) {
    keypressLongPendingBits.mode = 0;
    return TRUE;
  }
  return FALSE;
}

BOOL dpadUpLongPressed(void)
{
  if (keypressLongPendingBits.dpadUp) {
    keypressLongPendingBits.dpadUp = 0;
    return TRUE;
  }
  return FALSE;
}

BOOL dpadDownLongPressed(void)
{
  if (keypressLongPendingBits.dpadDown) {
    keypressLongPendingBits.dpadDown = 0;
    return TRUE;
  }
  return FALSE;
}

BOOL dpadLeftLongPressed(void)
{
  if (keypressLongPendingBits.dpadLeft) {
    keypressLongPendingBits.dpadLeft = 0;
    return TRUE;
  }
  return FALSE;
}

BOOL dpadRightLongPressed(void)
{
  if (keypressLongPendingBits.dpadRight) {
    keypressLongPendingBits.dpadRight = 0;
    return TRUE;
  }
  return FALSE;
}

BOOL dpadEnterLongPressed(void)
{
  if (keypressLongPendingBits.dpadEnter) {
    keypressLongPendingBits.dpadEnter = 0;
    return TRUE;
  }
  return FALSE;
}

BOOL topLeftLongPressed(void)
{
  if (keypressLongPendingBits.topLeft) {
    keypressLongPendingBits.topLeft = 0;
    return TRUE;
  }
  return FALSE;
}

BOOL topRightLongPressed(void)
{
  if (keypressLongPendingBits.topRight) {
    keypressLongPendingBits.topRight = 0;
    return TRUE;
  }
  return FALSE;
}

// ******************************************************************************************************
// ************** USB Callback Functions ****************************************************************
// ******************************************************************************************************
// The USB firmware stack will call the callback functions USBCBxxx() in response to certain USB related
// events.  For example, if the host PC is powering down, it will stop sending out Start of Frame (SOF)
// packets to your device.  In response to this, all USB devices are supposed to decrease their power
// consumption from the USB Vbus to <2.5mA each.  The USB module detects this condition (which according
// to the USB specifications is 3+ms of no bus activity/SOF packets) and then calls the USBCBSuspend()
// function.  You should modify these callback functions to take appropriate actions for each of these
// conditions.  For example, in the USBCBSuspend(), you may wish to add code that will decrease power
// consumption from Vbus to <2.5mA (such as by clock switching, turning off LEDs, putting the
// microcontroller to sleep, etc.).  Then, in the USBCBWakeFromSuspend() function, you may then wish to
// add code that undoes the power saving things done in the USBCBSuspend() function.

// The USBCBSendResume() function is special, in that the USB stack will not automatically call this
// function.  This function is meant to be called from the application firmware instead.  See the
// additional comments near the function.

// Note *: The "usb_20.pdf" specs indicate 500uA or 2.5mA, depending upon device classification. However,
// the USB-IF has officially issued an ECN (engineering change notice) changing this to 2.5mA for all
// devices.  Make sure to re-download the latest specifications to get all of the newest ECNs.

/******************************************************************************
 * Function:        void USBCBSuspend(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Call back that is invoked when a USB suspend is detected
 *
 * Note:            None
 *****************************************************************************/
void USBCBSuspend(void)
{
  //Example power saving code.  Insert appropriate code here for the desired
  //application behavior.  If the microcontroller will be put to sleep, a
  //process similar to that shown below may be used:

  //ConfigureIOPinsForLowPower();
  //SaveStateOfAllInterruptEnableBits();
  //DisableAllInterruptEnableBits();
  //EnableOnlyTheInterruptsWhichWillBeUsedToWakeTheMicro();	//should enable at least USBActivityIF as a wake source
  //Sleep();
  //RestoreStateOfAllPreviouslySavedInterruptEnableBits();	//Preferrably, this should be done in the USBCBWakeFromSuspend() function instead.
  //RestoreIOPinsToNormal();									//Preferrably, this should be done in the USBCBWakeFromSuspend() function instead.

  //Alternatively, the microcontorller may use clock switching to reduce current consumption.

  OSCCON = 0x13; //Sleep on sleep, 125kHz selected as microcontroller clock source

  //IMPORTANT NOTE: Do not clear the USBActivityIF (ACTVIF) bit here.  This bit is
  //cleared inside the usb_device.c file.  Clearing USBActivityIF here will cause
  //things to not work as intended.
}

/******************************************************************************
 * Function:        void USBCBWakeFromSuspend(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The host may put USB peripheral devices in low power
 *					suspend mode (by "sending" 3+ms of idle).  Once in suspend
 *					mode, the host may wake the device back up by sending non-
 *					idle state signalling.
 *
 *					This call back is invoked when a wakeup from USB suspend
 *					is detected.
 *
 * Note:            None
 *****************************************************************************/
void USBCBWakeFromSuspend(void)
{
  // If clock switching or other power savings measures were taken when
  // executing the USBCBSuspend() function, now would be a good time to
  // switch back to normal full power run mode conditions.  The host allows
  // 10+ milliseconds of wakeup time, after which the device must be
  // fully back to normal, and capable of receiving and processing USB
  // packets.  In order to do this, the USB module must receive proper
  // clocking (IE: 48MHz clock must be available to SIE for full speed USB
  // operation).
  // Make sure the selected oscillator settings are consistant with USB operation
  // before returning from this function.

  OSCCON = 0x60; //Primary clock source selected.
  //Adding a software start up delay will ensure
  //that the primary oscillator and PLL are running before executing any other
  //code.  If the PLL isn't being used, (ex: primary osc = 48MHz externally applied EC)
  //then this code adds a small unnecessary delay, but it is harmless to execute anyway.
  {
    unsigned int pll_startup_counter = 800; //Long delay at 31kHz, but ~0.8ms at 48MHz
    while (pll_startup_counter--); //Clock will switch over while executing this delay loop
  }
}

/********************************************************************
 * Function:        void USBCB_SOF_Handler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USB host sends out a SOF packet to full-speed
 *                  devices every 1 ms. This interrupt may be useful
 *                  for isochronous pipes. End designers should
 *                  implement callback routine as necessary.
 *
 * Note:            None
 *******************************************************************/
void USBCB_SOF_Handler(void)
{
  // No need to clear UIRbits.SOFIF to 0 here.
  // Callback caller is already doing that.
}

/*******************************************************************
 * Function:        void USBCBErrorHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The purpose of this callback is mainly for
 *                  debugging during development. Check UEIR to see
 *                  which error causes the interrupt.
 *
 * Note:            None
 *******************************************************************/
void USBCBErrorHandler(void)
{
  // No need to clear UEIR to 0 here.
  // Callback caller is already doing that.

  // Typically, user firmware does not need to do anything special
  // if a USB error occurs.  For example, if the host sends an OUT
  // packet to your device, but the packet gets corrupted (ex:
  // because of a bad connection, or the user unplugs the
  // USB cable during the transmission) this will typically set
  // one or more USB error interrupt flags.  Nothing specific
  // needs to be done however, since the SIE will automatically
  // send a "NAK" packet to the host.  In response to this, the
  // host will normally retry to send the packet again, and no
  // data loss occurs.  The system will typically recover
  // automatically, without the need for application firmware
  // intervention.

  // Nevertheless, this callback function is provided, such as
  // for debugging purposes.
}

/*******************************************************************
 * Function:        void USBCBCheckOtherReq(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        When SETUP packets arrive from the host, some
 * 					firmware must process the request and respond
 *					appropriately to fulfill the request.  Some of
 *					the SETUP packets will be for standard
 *					USB "chapter 9" (as in, fulfilling chapter 9 of
 *					the official USB specifications) requests, while
 *					others may be specific to the USB device class
 *					that is being implemented.  For example, a HID
 *					class device needs to be able to respond to
 *					"GET REPORT" type of requests.  This
 *					is not a standard USB chapter 9 request, and
 *					therefore not handled by usb_device.c.  Instead
 *					this request should be handled by class specific
 *					firmware, such as that contained in usb_function_hid.c.
 *
 * Note:            None
 *******************************************************************/
void USBCBCheckOtherReq(void)
{
  USBCheckHIDRequest();
}//end

/*******************************************************************
 * Function:        void USBCBStdSetDscHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USBCBStdSetDscHandler() callback function is
 *					called when a SETUP, bRequest: SET_DESCRIPTOR request
 *					arrives.  Typically SET_DESCRIPTOR requests are
 *					not used in most applications, and it is
 *					optional to support this type of request.
 *
 * Note:            None
 *******************************************************************/
void USBCBStdSetDscHandler(void)
{
  // Must claim session ownership if supporting this request
}//end

/*******************************************************************
 * Function:        void USBCBInitEP(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called when the device becomes
 *                  initialized, which occurs after the host sends a
 * 					SET_CONFIGURATION (wValue not = 0) request.  This
 *					callback function should initialize the endpoints
 *					for the device's usage according to the current
 *					configuration.
 *
 * Note:            None
 *******************************************************************/
void USBCBInitEP(void)
{
  //enable the HID endpoint
  USBEnableEndpoint(HID_EP, USB_IN_ENABLED | USB_OUT_ENABLED | USB_HANDSHAKE_ENABLED | USB_DISALLOW_SETUP);
  //Arm OUT endpoint so we can receive caps lock, num lock, etc. info from host
  lastOUTTransmission = HIDRxPacket(HID_EP, (BYTE*) & hid_report_out, 1);
}

/********************************************************************
 * Function:        void USBCBSendResume(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USB specifications allow some types of USB
 * 					peripheral devices to wake up a host PC (such
 *					as if it is in a low power suspend to RAM state).
 *					This can be a very useful feature in some
 *					USB applications, such as an Infrared remote
 *					control	receiver.  If a user presses the "power"
 *					button on a remote control, it is nice that the
 *					IR receiver can detect this signalling, and then
 *					send a USB "command" to the PC to wake up.
 *
 *					The USBCBSendResume() "callback" function is used
 *					to send this special USB signalling which wakes
 *					up the PC.  This function may be called by
 *					application firmware to wake up the PC.  This
 *					function will only be able to wake up the host if
 *                  all of the below are true:
 *
 *					1.  The USB driver used on the host PC supports
 *						the remote wakeup capability.
 *					2.  The USB configuration descriptor indicates
 *						the device is remote wakeup capable in the
 *						bmAttributes field.
 *					3.  The USB host PC is currently sleeping,
 *						and has previously sent your device a SET
 *						FEATURE setup packet which "armed" the
 *						remote wakeup capability.
 *
 *                  If the host has not armed the device to perform remote wakeup,
 *                  then this function will return without actually performing a
 *                  remote wakeup sequence.  This is the required behavior,
 *                  as a USB device that has not been armed to perform remote
 *                  wakeup must not drive remote wakeup signalling onto the bus;
 *                  doing so will cause USB compliance testing failure.
 *
 *					This callback should send a RESUME signal that
 *                  has the period of 1-15ms.
 *
 * Note:            This function does nothing and returns quickly, if the USB
 *                  bus and host are not in a suspended condition, or are
 *                  otherwise not in a remote wakeup ready state.  Therefore, it
 *                  is safe to optionally call this function regularly, ex:
 *                  anytime application stimulus occurs, as the function will
 *                  have no effect, until the bus really is in a state ready
 *                  to accept remote wakeup.
 *
 *                  When this function executes, it may perform clock switching,
 *                  depending upon the application specific code in
 *                  USBCBWakeFromSuspend().  This is needed, since the USB
 *                  bus will no longer be suspended by the time this function
 *                  returns.  Therefore, the USB module will need to be ready
 *                  to receive traffic from the host.
 *
 *                  The modifiable section in this routine may be changed
 *                  to meet the application needs. Current implementation
 *                  temporary blocks other functions from executing for a
 *                  period of ~3-15 ms depending on the core frequency.
 *
 *                  According to USB 2.0 specification section 7.1.7.7,
 *                  "The remote wakeup device must hold the resume signaling
 *                  for at least 1 ms but for no more than 15 ms."
 *                  The idea here is to use a delay counter loop, using a
 *                  common value that would work over a wide range of core
 *                  frequencies.
 *                  That value selected is 1800. See table below:
 *                  ==========================================================
 *                  Core Freq(MHz)      MIP         RESUME Signal Period (ms)
 *                  ==========================================================
 *                      48              12          1.05
 *                       4              1           12.6
 *                  ==========================================================
 *                  * These timing could be incorrect when using code
 *                    optimization or extended instruction mode,
 *                    or when having other interrupts enabled.
 *                    Make sure to verify using the MPLAB SIM's Stopwatch
 *                    and verify the actual signal on an oscilloscope.
 *******************************************************************/
void USBCBSendResume(void)
{
  static WORD delay_count;

  //First verify that the host has armed us to perform remote wakeup.
  //It does this by sending a SET_FEATURE request to enable remote wakeup,
  //usually just before the host goes to standby mode (note: it will only
  //send this SET_FEATURE request if the configuration descriptor declares
  //the device as remote wakeup capable, AND, if the feature is enabled
  //on the host (ex: on Windows based hosts, in the device manager
  //properties page for the USB device, power management tab, the
  //"Allow this device to bring the computer out of standby." checkbox
  //should be checked).
  if (USBGetRemoteWakeupStatus() == TRUE) {
    //Verify that the USB bus is in fact suspended, before we send
    //remote wakeup signalling.
    if (USBIsBusSuspended() == TRUE) {
      USBMaskInterrupts();

      //Clock switch to settings consistent with normal USB operation.
      USBCBWakeFromSuspend();
      USBSuspendControl = 0;
      USBBusIsSuspended = FALSE; //So we don't execute this code again,
      //until a new suspend condition is detected.

      //Section 7.1.7.7 of the USB 2.0 specifications indicates a USB
      //device must continuously see 5ms+ of idle on the bus, before it sends
      //remote wakeup signalling.  One way to be certain that this parameter
      //gets met, is to add a 2ms+ blocking delay here (2ms plus at
      //least 3ms from bus idle to USBIsBusSuspended() == TRUE, yeilds
      //5ms+ total delay since start of idle).
      delay_count = 3600U;
      do {
        delay_count--;
      } while (delay_count);

      //Now drive the resume K-state signalling onto the USB bus.
      USBResumeControl = 1; // Start RESUME signaling
      delay_count = 1800U; // Set RESUME line for 1-13 ms
      do {
        delay_count--;
      } while (delay_count);
      USBResumeControl = 0; //Finished driving resume signalling

      USBUnmaskInterrupts();
    }
  }
}

/*******************************************************************
 * Function:        BOOL USER_USB_CALLBACK_EVENT_HANDLER(
 *                        int event, void *pdata, WORD size)
 *
 * PreCondition:    None
 *
 * Input:           int event - the type of event
 *                  void *pdata - pointer to the event data
 *                  WORD size - size of the event data
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called from the USB stack to
 *                  notify a user application that a USB event
 *                  occured.  This callback is in interrupt context
 *                  when the USB_INTERRUPT option is selected.
 *
 * Note:            None
 *******************************************************************/
BOOL USER_USB_CALLBACK_EVENT_HANDLER(int event, void *pdata, WORD size)
{
  switch (event) {
  case EVENT_TRANSFER:
    //Add application specific callback task or callback function here if desired.
    break;
  case EVENT_SOF:
    USBCB_SOF_Handler();
    break;
  case EVENT_SUSPEND:
    USBCBSuspend();
    break;
  case EVENT_RESUME:
    USBCBWakeFromSuspend();
    break;
  case EVENT_CONFIGURED:
    USBCBInitEP();
    break;
  case EVENT_SET_DESCRIPTOR:
    USBCBStdSetDscHandler();
    break;
  case EVENT_EP0_REQUEST:
    USBCBCheckOtherReq();
    break;
  case EVENT_BUS_ERROR:
    USBCBErrorHandler();
    break;
  case EVENT_TRANSFER_TERMINATED:
    //Add application specific callback task or callback function here if desired.
    //The EVENT_TRANSFER_TERMINATED event occurs when the host performs a CLEAR
    //FEATURE (endpoint halt) request on an application endpoint which was
    //previously armed (UOWN was = 1).  Here would be a good place to:
    //1.  Determine which endpoint the transaction that just got terminated was
    //      on, by checking the handle value in the *pdata.
    //2.  Re-arm the endpoint if desired (typically would be the case for OUT
    //      endpoints).
    break;
  default:
    break;
  }
  return TRUE;
}


// *****************************************************************************
// ************** USB Class Specific Callback Function(s) **********************
// *****************************************************************************

/********************************************************************
 * Function:        void USBHIDCBSetReportHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        USBHIDCBSetReportHandler() is used to respond to
 *					the HID device class specific SET_REPORT control
 *					transfer request (starts with SETUP packet on EP0 OUT).
 * Note:
 *******************************************************************/
void USBHIDCBSetReportHandler(void)
{
  //Prepare to receive the keyboard LED state data through a SET_REPORT
  //control transfer on endpoint 0.  The host should only send 1 byte,
  //since this is all that the report descriptor allows it to send.
  USBEP0Receive((BYTE*) & CtrlTrfData, USB_EP0_BUFF_SIZE, USBHIDCBSetReportComplete);
}

//Secondary callback function that gets called when the above
//control transfer completes for the USBHIDCBSetReportHandler()

void USBHIDCBSetReportComplete(void)
{
  //1 byte of LED state data should now be in the CtrlTrfData buffer.
}
/** EOF hidmedia.c **********************************************/

#endif
