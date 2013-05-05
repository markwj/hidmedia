/************************************************************************
	HardwareProfile.h

	WFF USB Generic HID Demonstration 3
    usbGenericHidCommunication reference firmware 3_0_0_0
    Copyright (C) 2011 Simon Inns

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

	Email: simon.inns@gmail.com

************************************************************************/

#ifndef HARDWAREPROFILE_H
#define HARDWAREPROFILE_H

// USB stack hardware selection options ----------------------------------------------------------------

// (This section is the set of definitions required by the MCHPFSUSB framework.)

// Uncomment the following define if you wish to use the self-power sense feature 
// and define the port, pin and tris for the power sense pin below:
// #define USE_SELF_POWER_SENSE_IO
//#define tris_self_power     TRISAbits.TRISA2
#if defined(USE_SELF_POWER_SENSE_IO)
	#define self_power          PORTAbits.RA2
#else
	#define self_power          1
#endif

// Uncomment the following define if you wish to use the bus-power sense feature 
// and define the port, pin and tris for the power sense pin below:
//#define USE_USB_BUS_SENSE_IO
//#define tris_usb_bus_sense  TRISAbits.TRISA1
#if defined(USE_USB_BUS_SENSE_IO)
	#define USB_BUS_SENSE       PORTAbits.RA1
#else
	#define USB_BUS_SENSE       1
#endif

// Uncomment the following line to make the output HEX of this project work with the MCHPUSB Bootloader    
//#define PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER

// Uncomment the following line to make the output HEX of this project work with the HID Bootloader
#define PROGRAMMABLE_WITH_USB_HID_BOOTLOADER		

// Application specific hardware definitions ------------------------------------------------------------

// Oscillator frequency (48Mhz with a 20Mhz external oscillator)
#define CLOCK_FREQ 48000000

// Device Vendor Indentifier (VID) (0x04D8 is Microchip's VID)
#define USB_VID	0x04D8

// Device Product Indentifier (PID) (0x0042)
#define USB_PID	0x0042

// Manufacturer string descriptor
//#define MSDLENGTH	3
//#define MSD		'M','W','J'

// Product String descriptor
//#define PSDLENGTH	20
//#define PSD		'M','W','J',' ','M','e','d','i','a',' ','C','o','n','t','r','o','l','l','e','r'

// Device serial number string descriptor
//#define DSNLENGTH	7
//#define DSN		'M','W','J','_','3','.','0'

// Common useful definitions
#define INPUT_PIN 1
#define OUTPUT_PIN 0
#define FLAG_FALSE 0
#define FLAG_TRUE 1

// PIC to hardware pin mapping and control macros

    /** LED ************************************************************/
    #define mInitAllLEDs()

    #define mLED_1              LATAbits.LATA5
    #define mLED_2
    #define mLED_3
    #define mLED_4

    #define mGetLED_1()         mLED_1
    #define mGetLED_2()         1
    #define mGetLED_3()         1
    #define mGetLED_4()         1

    #define mLED_1_On()         mLED_1 = 1;
    #define mLED_2_On()
    #define mLED_3_On()
    #define mLED_4_On()

    #define mLED_1_Off()        mLED_1 = 0;
    #define mLED_2_Off()
    #define mLED_3_Off()
    #define mLED_4_Off()

    #define mLED_1_Toggle()     mLED_1 = !mLED_1;
    #define mLED_2_Toggle()
    #define mLED_3_Toggle()
    #define mLED_4_Toggle()

    /** SWITCH *********************************************************/
    //#define mInitSwitch2()      TRISAbits.TRISA0=1;
    //#define mInitSwitch3()      mInitSwitch2();
    //#define mInitAllSwitches()  mInitSwitch2();
    //#define sw2                 PORTAbits.RA1
    //#define sw3                 PORTAbits.RA2

#endif
