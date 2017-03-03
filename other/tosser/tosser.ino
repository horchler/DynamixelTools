/*
 *	tosser.ino
 *
 *  Requires DynamixelQ Library and ROBOTIS_OPENCM IDE to compile:
 *      https://github.com/horchler/DynamixelQ
 *      http://support.robotis.com/en/software/robotis_opencm/robotis_opencm.htm
 *	
 *	Tosser to interact with Matlab DXL class. Read all available bytes from USB serial
 *	port and write them directly to Dynamixel serial bus. Read all available bytes from
 *	Dynamixel serial bus and write them directly to the USB serial port. Additionally,
 *	special packets permit communication with and control of the tosser. See Tosser.h.
 *	
 *	Actuators are assumed to be MX Series actuators by default. To handle AX Series
 *	actuators (or mixed types), change the DEFAULT_BAUD_RATE_VALUE property in the Matlab
 *	DXL class to BaudRateValue.BAUD_1000000 (or less) or call the begin method of the
 *	Matlab DXL class and specify the desired baud rate value.
 *	
 *	Author: Andrew D. Horchler, horchler @ gmail . com
 *	Created: 7-5-15, modified: 4-6-16
 */

#include "DynamixelQ.h"
#include "Tosser.h"

void setup() {
  Tosser.begin();
}

void loop() {
  // Empty
}
