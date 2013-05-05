/************************************************************************
	debug.h

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

#ifndef DEBUG_H
#define DEBUG_H

// Microchip Application Library includes
#include "./USB/usb.h"
#include "./USB/usb_function_hid.h"

// Set the size of the debug information buffer in characters
#define DEBUGBUFFERSIZE 128

// Function prototypes
void debugInitialise(void);
void debugOut(char*);
void copyDebugToSendBuffer(BYTE* sendDataBuffer);

#endif