/**********************************************************************

Config.h
COPYRIGHT (c) 2013-2016 Gregg E. Berman

Part of DCC++ BASE STATION for the Arduino

**********************************************************************/

/////////////////////////////////////////////////////////////////////////////////////
//
// DEFINE MOTOR_SHIELD_TYPE ACCORDING TO THE FOLLOWING TABLE:
//
//  0 = ARDUINO MOTOR SHIELD          (MAX 18V/2A PER CHANNEL)
//  1 = POLOLU MC33926 MOTOR SHIELD   (MAX 28V/3A PER CHANNEL)

#define MOTOR_SHIELD_TYPE   0

/////////////////////////////////////////////////////////////////////////////////////
//
// DEFINE NUMBER OF MAIN TRACK REGISTER

#define MAX_MAIN_REGISTERS 12

/////////////////////////////////////////////////////////////////////////////////////
//
// DEFINE COMMUNICATIONS INTERFACE
//
//  0 = Built-in Serial Port
//  1 = Internal Uno R4 Wifi

#define COMM_INTERFACE  1

/////////////////////////////////////////////////////////////////////////////////////
//
// DEFINE STATIC IP ADDRESS *OR* COMMENT OUT TO USE DHCP
//

//#define IP_ADDRESS { 192, 168, 1, 16 }

/////////////////////////////////////////////////////////////////////////////////////
//
// DEFINE PORT TO USE FOR ETHERNET COMMUNICATIONS INTERFACE
//

#define ETHERNET_PORT 2560

/////////////////////////////////////////////////////////////////////////////////////
//
// DEFINE MAC ADDRESS ARRAY FOR ETHERNET COMMUNICATIONS INTERFACE
//

//#define MAC_ADDRESS {  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xEF }

/////////////////////////////////////////////////////////////////////////////////////
//
// DEFINE THE SSID OF THE WIRELESS NETWORK TO CONNECT
//

#define SECRET_SSID ""

/////////////////////////////////////////////////////////////////////////////////////
//
// DEFINE THE PASSWORD OF THE WIRELESS NETWORK TO CONNECT
//

#define SECRET_PASS ""

/////////////////////////////////////////////////////////////////////////////////////
//
// DEFINE THE HOSTNAME OF THE DCC COMMAND STATION
//

#define WIFI_HOSTNAME "DCC-command-station"

/////////////////////////////////////////////////////////////////////////////////////
//
// DEFINE THE TIMEZONE OFFSET
//

#define TIMEZONE_OFFSET 1