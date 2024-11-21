/**********************************************************************

DCCpp_UnoR4.h
COPYRIGHT (c) 2024 Matthieu Overney

Inspired by DCC++ BASE STATION for the Arduino

**********************************************************************/

#include "Config.h"

#ifndef DCCpp_UnoR4_h
#define DCCpp_UnoR4_h

/////////////////////////////////////////////////////////////////////////////////////
// RELEASE VERSION
/////////////////////////////////////////////////////////////////////////////////////

#define VERSION "0.8.0"

/////////////////////////////////////////////////////////////////////////////////////
// This code only works for ARDUINO UNO R4 WiFi
/////////////////////////////////////////////////////////////////////////////////////

#if defined  ARDUINO_UNOWIFIR4

  #define ARDUINO_TYPE    "R4"

  #define DCC_SIGNAL_PIN_MAIN 12          // Arduino UNO R4 - uses GTIOC6B
  #define DCC_SIGNAL_PIN_PROG 13          // Arduino UNO R4 - uses GTIOC2B

#else

  #error CANNOT COMPILE - This version of DCC++ only works with ARDUINO UNO R4 WiFi

#endif

/////////////////////////////////////////////////////////////////////////////////////
// SELECT MOTOR SHIELD
/////////////////////////////////////////////////////////////////////////////////////

#if MOTOR_SHIELD_TYPE == 0

  #define MOTOR_SHIELD_NAME "ARDUINO MOTOR SHIELD"

  #define SIGNAL_ENABLE_PIN_MAIN 3
  #define SIGNAL_ENABLE_PIN_PROG 11

  #define CURRENT_MONITOR_PIN_MAIN A0
  #define CURRENT_MONITOR_PIN_PROG A1

  #define DIRECTION_MOTOR_CHANNEL_PIN_A 12
  #define DIRECTION_MOTOR_CHANNEL_PIN_B 13

#elif MOTOR_SHIELD_TYPE == 1

  #define MOTOR_SHIELD_NAME "POLOLU MC33926 MOTOR SHIELD"

  #define SIGNAL_ENABLE_PIN_MAIN 9
  #define SIGNAL_ENABLE_PIN_PROG 11

  #define CURRENT_MONITOR_PIN_MAIN A0
  #define CURRENT_MONITOR_PIN_PROG A1

  #define DIRECTION_MOTOR_CHANNEL_PIN_A 7
  #define DIRECTION_MOTOR_CHANNEL_PIN_B 8

#else

  #error CANNOT COMPILE - PLEASE SELECT A PROPER MOTOR SHIELD TYPE

#endif

/////////////////////////////////////////////////////////////////////////////////////
// SELECT COMMUNICATION INTERACE
/////////////////////////////////////////////////////////////////////////////////////

#if COMM_INTERFACE == 0

  #define COMM_TYPE 0
  #define INTERFACE Serial

#elif (COMM_INTERFACE == 1)

  #define COMM_TYPE 1
  #define INTERFACE eServer
  
#else

  #error CANNOT COMPILE - Please select a proper value for COMM_INTERFACE in CONFIG.H file

#endif

/////////////////////////////////////////////////////////////////////////////////////
// SET WHETHER TO SHOW PACKETS - DIAGNOSTIC MODE ONLY
/////////////////////////////////////////////////////////////////////////////////////

// If SHOW_PACKETS is set to 1, then for select main operations track commands that modify an internal DCC packet register,
// if printFlag for that command is also set to 1, DCC++ BASE STATION will additionally return the 
// DCC packet contents of the modified register in the following format:

//    <* REG: B1 B2 ... Bn CSUM / REPEAT>
//
//    REG: the number of the main operations track packet register that was modified
//    B1: the first hexidecimal byte of the DCC packet
//    B2: the second hexidecimal byte of the DCC packet
//    Bn: the nth hexidecimal byte of the DCC packet
//    CSUM: a checksum byte that is required to be the final byte in any DCC packet
//    REPEAT: the number of times the DCC packet was re-transmitted to the tracks after its iniital transmission
 
#define SHOW_PACKETS  0       // set to zero to disable printing of every packet for select main operations track commands

/////////////////////////////////////////////////////////////////////////////////////

#endif


