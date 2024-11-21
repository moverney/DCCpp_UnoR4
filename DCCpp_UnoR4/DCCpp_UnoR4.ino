/**********************************************************************

/!\ ALPHA Release /!\
/!\ Work in progress /!\

DCC++ Arduino Uno R4 Wifi BASE STATION 
COPYRIGHT (c) 2024 Matthieu Overney

  This program is is based on the great DCC++ base station written by Gregg E. Berman https://github.com/DccPlusPlus
  It is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see http://www.gnu.org/licenses

**********************************************************************/
/**********************************************************************
      
This C++ program written for the Arduino Uno R4 Wifi only. It has not been tested on other R4 family boards.

It allows a standard Arduino Uno R4 WiFi an Arduino Motor Shield (as well as others) 
to be used as a fully-functioning digital command and control (DCC) base station
for controlling model train layouts that conform to current National Model
Railroad Association (NMRA) DCC standards.
It has only be tested with an Arduino Motor Shield and other shield may require some code adaptations.

This version of DCC++ BASE STATION for Uno R4 supports:

  * 2-byte and 4-byte locomotive addressing
  * Simultaneous control of multiple locomotives
  * 128-step speed throttling
  * Cab functions F0-F28
  * Activate/de-activate accessory functions using 512 addresses, each with 4 sub-addresses
      - includes optional functionailty to monitor and store of the direction of any connected turnouts

This version of DCC++ DOES NOT SUPPORT programming cabs for the moment.
All other functionalities of DCC++ (sensors, accesories, ...) have not been tested

REFERENCES:

  DCC++:                       https://github.com/DccPlusPlus
  Arduino:                     http://www.arduino.cc/
  GNU General Public License:  http://opensource.org/licenses/GPL-3.0

An Arduino Motor Shield (or similar), powered by a standard 15V DC power supply and attached
on top of the Arduino Uno R4 Wifi, is used to transform the 0-5V DCC logic signals
produced by the Uno's Timer interrupts into proper 0-15V bi-polar DCC signals.

Main line output on pin 12 is connected to the Motor Shield's DIRECTION A input (pin 12).
Programming line on pin 13 is connected to the Motor Shield's DIRECTION B input (pin 13).
No jumper is required.

Other Motor Shields may require different sets of jumper or configurations (see Config.h and DCCpp_Uno.h for details).

Uno R4 onboard WiFi module is used for network connection. SSID and Password may be configured in Config.h
No addtional WiFi shield is supported.

When configured as such, the CHANNEL A and CHANNEL B outputs of the Motor Shield may be
connected directly to the tracks.  This software assumes CHANNEL A is connected
to the Main Operations Track, and CHANNEL B is connected to the Programming Track (currently not working).

DCC++ BASE STATION in split into multiple modules, each with its own header file:

  DCCpp_UnoR4:        declares required global objects and contains initial Arduino setup()
                    and Arduino loop() functions, as well as interrput code for OC0B and OC1B.
                    Also includes declarations of optional array of Turn-Outs and optional array of Sensors 

  SerialCommand:    contains methods to read and interpret text commands from the serial line,
                    process those instructions, and, if necessary call appropriate Packet RegisterList methods
                    to update either the Main Track or Programming Track Packet Registers

  PacketRegister:   contains methods to load, store, and update Packet Registers with DCC instructions

  CurrentMonitor:   contains methods to separately monitor and report the current drawn from CHANNEL A and
                    CHANNEL B of the Arduino Motor Shield's, and shut down power if a short-circuit overload
                    is detected

  Accessories:      contains methods to operate and store the status of any optionally-defined turnouts controlled
                    by a DCC stationary accessory decoder.

  Sensor:           contains methods to monitor and report on the status of optionally-defined infrared
                    sensors embedded in the Main Track and connected to various pins on the Arudino Uno

  Outputs:          contains methods to configure one or more Arduino pins as an output for your own custom use

  EEStore:          contains methods to store, update, and load various DCC settings and status
                    (e.g. the states of all defined turnouts) in the EEPROM for recall after power-up

DCC++ BASE STATION is configured through the Config.h file that contains all user-definable parameters                    

**********************************************************************/

// BEGIN BY INCLUDING THE HEADER FILES FOR EACH MODULE
 
#include "DCCpp_UnoR4.h"
#include "PacketRegister.h"
#include "CurrentMonitor.h"
#include "Sensor.h"
#include "SerialCommand.h"
#include "Accessories.h"
#include "EEStore.h"
#include "Config.h"
#include "Comm.h"
#include "IRQManager.h"
// Include the RTC library
#include "RTC.h"


//Include the NTP library
#include <NTPClient.h>

void showConfiguration();

#if COMM_TYPE == 1
  WiFiServer INTERFACE(ETHERNET_PORT);         // Create and instance of a WiFiServer
#endif

// WiFi parameters
int status = WL_IDLE_STATUS;     // the WiFi radio's status
WiFiUDP Udp; // A UDP instance to let us send and receive packets over UDP
NTPClient timeClient(Udp);


// NEXT DECLARE GLOBAL OBJECTS TO PROCESS AND STORE DCC PACKETS AND MONITOR TRACK CURRENTS.
// NOTE REGISTER LISTS MUST BE DECLARED WITH "VOLATILE" QUALIFIER TO ENSURE THEY ARE PROPERLY UPDATED BY INTERRUPT ROUTINES

volatile RegisterList mainRegs(MAX_MAIN_REGISTERS);    // create list of registers for MAX_MAIN_REGISTER Main Track Packets
volatile RegisterList progRegs(2);                     // create a shorter list of only two registers for Program Track Packets

CurrentMonitor mainMonitor(CURRENT_MONITOR_PIN_MAIN,"<p2>");  // create monitor for current on Main Track
CurrentMonitor progMonitor(CURRENT_MONITOR_PIN_PROG,"<p3>");  // create monitor for current on Program Track

GenericIrqCfg_t cfgMain;
GenericIrqCfg_t cfgProg;

// Configuration for UNO R4 WiFi
#define DCC_ZERO_BIT_TOTAL_DURATION_GPT16 9599 // 9600 / 48000000 = 0.0002 seconds = 200 microseconds
#define DCC_ZERO_BIT_PULSE_DURATION_GPT16 4799

#define DCC_ONE_BIT_TOTAL_DURATION_GPT16 5567 // 5568 / 48000000 = 0.000116 seconds = 116 microseconds
#define DCC_ONE_BIT_PULSE_DURATION_GPT16 2783

///////////////////////////////////////////////////////////////////////////////
// MAIN ARDUINO LOOP
///////////////////////////////////////////////////////////////////////////////

void loop(){
  
  SerialCommand::process();              // check for, and process, and new serial commands
  
  if(CurrentMonitor::checkTime()){      // if sufficient time has elapsed since last update, check current draw on Main and Program Tracks 
    mainMonitor.check();
    progMonitor.check();
  }

  Sensor::check();    // check sensors for activate/de-activate
  
} // loop



///////////////////////////////////////////////////////////////////////////////
// INITIAL SETUP
///////////////////////////////////////////////////////////////////////////////

void setup(){  

  Serial.begin(115200);            // configure serial interface
  Serial.flush();
  
  setupWifi(); // Connect to the Wifi network
  RetrieveNTPTime(); // Retrieve NTP time and init RTC (for future use not required currently)

  #if COMM_TYPE == 1
    INTERFACE.begin();
  #endif
  
  #ifdef SDCARD_CS
    pinMode(SDCARD_CS,OUTPUT);
    digitalWrite(SDCARD_CS,HIGH);     // Deselect the SD card
  #endif

  EEStore::init();                                          // initialize and load Turnout and Sensor definitions stored in EEPROM
 
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);

// This portion of code is commented because with the Arduino Uno R4 WiFi the condition if(!digitalRead(A5)) always evaluate to true even if the PIN is not grounded
/*
  pinMode(A5,INPUT);                                       // if pin A5 is grounded upon start-up, print system configuration and halt
  digitalWrite(A5,HIGH);
  if(!digitalRead(A5)) {
    showConfiguration();
  }
*/

  Serial.print("<iDCC++ BASE STATION FOR ARDUINO ");      // Print Status to Serial Line regardless of COMM_TYPE setting so use can open Serial Monitor and check configurtion 
  Serial.print(ARDUINO_TYPE);
  Serial.print(" / ");
  Serial.print(MOTOR_SHIELD_NAME);
  Serial.print(": V-");
  Serial.print(VERSION);
  Serial.print(" / ");
  Serial.print(__DATE__);
  Serial.print(" ");
  Serial.print(__TIME__);
  Serial.print(">");

  SerialCommand::init(&mainRegs, &progRegs, &mainMonitor);   // create structure to read and parse commands from serial line

  Serial.print("<N");
  Serial.print(COMM_TYPE);
  Serial.print(": ");

  #if COMM_TYPE == 0
    Serial.print("SERIAL>");
  #elif COMM_TYPE == 1
    Serial.print(WiFi.localIP());
    Serial.print(":");
    Serial.print(ETHERNET_PORT);
    Serial.print(">");
  #endif
  
  setupDCCSignalTimers();

} // setup

void setupDCCSignalTimers() {
  
  // 10.2.5 Module Stop Control Register D (MSTPCRD) p.177
  // Activate 16-bit GPT timer module
  R_MSTP->MSTPCRD_b.MSTPD6 = 0;   // Value 0 cancels the stopped state
  
  // CONFIGURE TIMER GPT6 TO OUTPUT 50% DUTY CYCLE DCC SIGNALS ON GTIOC6B PIN FOR MAIN LINE
  setupMainLineTimer(); // Setup timer for main line
  setupProgLineTimer(); // Setup timer for programming line
  
  // 22.2.12 General PWM Timer Control Register (GTCR) p. 417  
  R_GPT2->GTCR |= 1; // GPT2 timer count starts
  R_GPT6->GTCR |= 1; // GPT6 timer count starts
}

void setupMainLineTimer() {
  
  // Direction Pin for Motor Shield Channel A - MAIN OPERATIONS TRACK
  // Controlled by Arduino 16-bit General Purpose Timer 6 
  // Values for 16-bit GTCCRB register calibrated for 1:1 prescale at 48 MHz clock frequency
  // Resulting waveforms are 200 microseconds for a ZERO bit and 116 microseconds for a ONE bit with exactly 50% duty cycle

  pinMode(DIRECTION_MOTOR_CHANNEL_PIN_A,INPUT);      // ensure this pin is not active! Direction will be controlled by DCC SIGNAL instead (below)
  digitalWrite(DIRECTION_MOTOR_CHANNEL_PIN_A,LOW);

  pinMode(DCC_SIGNAL_PIN_MAIN, OUTPUT);      // THIS ARDUINO OUPUT PIN MUST BE PHYSICALLY CONNECTED TO THE PIN FOR DIRECTION-A OF MOTOR CHANNEL-A

  //=================================================================================
  // Setup GPT6 timer for main line
  // 22.2.12 General PWM Timer Control Register (GTCR) p. 417
  R_GPT6->GTCR = 0;              // Clear register value. Counting stops, don't forget to restart if at the end of the initialization.
  R_GPT6->GTCR |= (0b000 << 16); // Saw-wave PWM mode, MD[2:0] = 0b000
  R_GPT6->GTCR |= (0b000 << 24); // Pre-scaler 1, TPCS[2:0] = 0b000 
  
  // 22.2.13 General PWM Timer Count Direction and Duty Setting Register (GTUDDTYC) p. 418
  R_GPT6->GTUDDTYC = 0x1; // Count up

  // 22.2.20 General PWM Timer Cycle Setting Register (GTPR) p. 431
  R_GPT6->GTPR = DCC_ONE_BIT_TOTAL_DURATION_GPT16; // Counter maximum value

  // 22.2.18 General PWM Timer Counter (GTCNT) p. 430
  R_GPT6->GTCNT = 0; // Counter start value

  // 22.2.14 General PWM Timer I/O Control Register (GTIOR) p. 420
  // PWM output configuration
  R_GPT6->GTIOR  = 0x00000000;      // Clear register value
  R_GPT6->GTIOR |= (0b00110 << 16); // GTIOB[4:0] = 00110b - Output initial LOW; HIGH output on compare match B; LOW output at cycle end (cf. Table 22.5 p. 418)
  R_GPT6->GTIOR |= 0x01000000;      // Output activation on pin GTIOC6B 

  // 22.2.19 General PWM Timer Compare Capture Register n (GTCCRn) (n = A to F) p. 430
  R_GPT6->GTCCR[1] = DCC_ONE_BIT_PULSE_DURATION_GPT16; // Compare match value GTCCRB (table index = 1)

  // 19.2.5 Port mn Pin Function Select Register (PmnPFS/PmnPFS_HA/PmnPFS_BY) (m = 0 to 9; n = 00 to 15) p. 361
  R_PFS->PORT[4].PIN[10].PmnPFS_b.PSEL = 0b00011; // Port 410 on pin D12 for PWM mode
  R_PFS->PORT[4].PIN[10].PmnPFS_b.PMR = 1; // Port is used as input/output for a peripheral function

  // Interrupt setup
  // Interrupt address
  cfgMain.irq = FSP_INVALID_VECTOR;
  // Interrupt priority
  cfgMain.ipl = 8;
  // Interrupt is triggered by the timer overflow (GTCNT == GTPR) (see elc_defines.h)
  cfgMain.event = ELC_EVENT_GPT6_COUNTER_OVERFLOW;  
  // Interrupt function registration
  IRQManager::getInstance().addGenericInterrupt(cfgMain, GPT6_OVF_callback);
  
  pinMode(SIGNAL_ENABLE_PIN_MAIN,OUTPUT);   // master enable for motor channel A
  mainRegs.loadPacket(1,RegisterList::idlePacket,2,0);    // load idle packet into register 1    
}

void GPT6_OVF_callback() {
  R_ICU->IELSR[cfgMain.irq] &= ~(R_ICU_IELSR_IR_Msk); /* Interrupt flag reset */ 
  DCCSignal(&mainRegs, R_GPT6, 1);
}

void setupProgLineTimer() { 

  // Directon Pin for Motor Shield Channel B - PROGRAMMING TRACK
  // Controlled by Arduino 16-bit General Purpose Timer 2 
  // Values for 16-bit GTCCRB register calibrated for 1:1 prescale at 48 MHz clock frequency
  // Resulting waveforms are 200 microseconds for a ZERO bit and 116 microseconds for a ONE bit with exactly 50% duty cycle

  pinMode(DIRECTION_MOTOR_CHANNEL_PIN_B,INPUT);      // ensure this pin is not active! Direction will be controlled by DCC SIGNAL instead (below)
  digitalWrite(DIRECTION_MOTOR_CHANNEL_PIN_B,LOW);

  pinMode(DCC_SIGNAL_PIN_PROG,OUTPUT);      // THIS ARDUINO OUTPUT PIN MUST BE PHYSICALLY CONNECTED TO THE PIN FOR DIRECTION-B OF MOTOR CHANNEL-B

  //=================================================================================
  //Setup GPT2 timer for programmation line
  // 22.2.12 General PWM Timer Control Register (GTCR) p. 417
  R_GPT2->GTCR = 0;              // Clear register value. Counting stops, don't forget to restart if at the end of the initialization.
  R_GPT2->GTCR |= (0b000 << 16); // Saw-wave PWM mode, MD[2:0] = 0b000
  R_GPT2->GTCR |= (0b000 << 24); // Pre-scaler 1, TPCS[2:0] = 0b000 

  // 22.2.13 General PWM Timer Count Direction and Duty Setting Register (GTUDDTYC) p. 418
  R_GPT2->GTUDDTYC = 0x1; // Count up

  // 22.2.20 General PWM Timer Cycle Setting Register (GTPR) p. 431
  R_GPT2->GTPR = DCC_ONE_BIT_TOTAL_DURATION_GPT16; // Counter maximum value

  // 22.2.18 General PWM Timer Counter (GTCNT) p. 430
  R_GPT2->GTCNT = 0; // Counter start value

  // 22.2.14 General PWM Timer I/O Control Register (GTIOR) p. 420
  // Configuration de l'entrée sortie pour PWM
  R_GPT2->GTIOR  = 0x00000000;      // Clear register value
  R_GPT2->GTIOR |= (0b00110 << 16); // GTIOB[4:0] = 00110b - Output initial LOW; HIGH output on compare match B; LOW output at cycle end (cf. Table 22.5 p. 418)
  R_GPT2->GTIOR |= 0x01000000;      // Output activation on pin GTIOC2B 

  // 22.2.19 General PWM Timer Compare Capture Register n (GTCCRn) (n = A to F) p. 430
  R_GPT2->GTCCR[1] = DCC_ONE_BIT_PULSE_DURATION_GPT16; // Compare match value GTCCRB (table index = 1)

  // 19.2.5 Port mn Pin Function Select Register (PmnPFS/PmnPFS_HA/PmnPFS_BY) (m = 0 to 9; n = 00 to 15) p. 361
  R_PFS->PORT[1].PIN[2].PmnPFS_b.PSEL = 0b00011; // Port 102 on pin D13 for PWM mode
  R_PFS->PORT[1].PIN[2].PmnPFS_b.PMR = 1; // Port is used as input/output for a peripheral function

  // Interrupt setup
  // Interrupt address
  cfgProg.irq = FSP_INVALID_VECTOR;
  // Interrupt priority
  cfgProg.ipl = 8;
  // Interrupt is triggered by the timer overflow (GTCNT == GTPR) (see elc_defines.h)
  cfgProg.event = ELC_EVENT_GPT2_COUNTER_OVERFLOW;  
  // Interrupt function registration
  IRQManager::getInstance().addGenericInterrupt(cfgProg, GPT2_OVF_callback);
  
  pinMode(SIGNAL_ENABLE_PIN_PROG,OUTPUT);   // master enable for motor channel B
  progRegs.loadPacket(1,RegisterList::idlePacket,2,0);    // load idle packet into register 1   
}

void GPT2_OVF_callback() {
  R_ICU->IELSR[cfgProg.irq] &= ~(R_ICU_IELSR_IR_Msk); /* Remise du flag d'interruption à 0 pour l'événement */ 
  DCCSignal(&progRegs, R_GPT2, 1);
}

///////////////////////////////////////////////////////////////////////////////
// DEFINE THE INTERRUPT LOGIC THAT GENERATES THE DCC SIGNAL
///////////////////////////////////////////////////////////////////////////////

// The macro coded in DCC++ has been rewritten to a function. There should not be any performance issues on a Uno R4 which is more powerfull than the Uno R3.

void DCCSignal(volatile RegisterList *regs, R_GPT0_Type *timer, int compareIndex) {  
  if(regs->currentBit==regs->currentReg->activePacket->nBits){    /* IF no more bits in this DCC Packet */  
    regs->currentBit=0;                                       /*   reset current bit pointer and determine which Register and Packet to process next--- */ 
    if(regs->nRepeat>0 && regs->currentReg==regs->reg){               /*   IF current Register is first Register AND should be repeated */  
      regs->nRepeat--;                                        /*     decrement repeat count; result is this same Packet will be repeated */  
    } else if(regs->nextReg!=NULL){                           /*   ELSE IF another Register has been updated */  
      regs->currentReg=regs->nextReg;                             /*     update currentReg to nextReg */  
      regs->nextReg=NULL;                                     /*     reset nextReg to NULL */  
      regs->tempPacket=regs->currentReg->activePacket;            /*     flip active and update Packets */        
      regs->currentReg->activePacket=regs->currentReg->updatePacket;  
      regs->currentReg->updatePacket=regs->tempPacket;  
    } else{                                               /*   ELSE simply move to next Register */  
      if(regs->currentReg==regs->maxLoadedReg)                    /*     BUT IF this is last Register loaded */  
        regs->currentReg=regs->reg;                               /*       first reset currentReg to base Register, THEN */  
      regs->currentReg++;                                     /*     increment current Register (note this logic causes Register[0] to be skipped when simply cycling through all Registers) */  
    }                                                     /*   END-ELSE */  
  }                                                       /* END-IF: currentReg, activePacket, and currentBit should now be properly set to point to next DCC bit */  

  if(regs->currentReg->activePacket->buf[regs->currentBit/8] & regs->bitMask[regs->currentBit%8]) {  /* IF bit is a ONE */      
    timer->GTPR = DCC_ONE_BIT_TOTAL_DURATION_GPT16;      /*   set GPT timer to full cycle duration of DCC ONE bit */  
    timer->GTCCR[compareIndex] = DCC_ONE_BIT_PULSE_DURATION_GPT16;  /*   set GPT timer to half cycle duration of DCC ONE bit */   
  }  
  else {   /* ELSE it is a ZERO */  
    timer->GTPR = DCC_ZERO_BIT_TOTAL_DURATION_GPT16;      /*   set GPT timer to full cycle duration of DCC ONE bit */  
    timer->GTCCR[compareIndex] = DCC_ZERO_BIT_PULSE_DURATION_GPT16;  /*   set GPT timer to half cycle duration of DCC ONE bit */  
  }   /* END-ELSE */   
                                                                                         
  regs->currentBit++;   /* point to next bit in current Packet */ 
}

///////////////////////////////////////////////////////////////////////////////
// PRINT CONFIGURATION INFO TO SERIAL PORT REGARDLESS OF INTERFACE TYPE
// - ACTIVATED ON STARTUP IF SHOW_CONFIG_PIN IS TIED HIGH 

void showConfiguration(){

  Serial.print("\n*** DCC++ CONFIGURATION ***\n");

  Serial.print("\nVERSION:      ");
  Serial.print(VERSION);
  Serial.print("\nCOMPILED:     ");
  Serial.print(__DATE__);
  Serial.print(" ");
  Serial.print(__TIME__);

  Serial.print("\nARDUINO:      ");
  Serial.print(ARDUINO_TYPE);

  Serial.print("\n\nMOTOR SHIELD: ");
  Serial.print(MOTOR_SHIELD_NAME);
  
  Serial.print("\n\nDCC SIG MAIN: ");
  Serial.print(DCC_SIGNAL_PIN_MAIN);
  Serial.print("\n   DIRECTION: ");
  Serial.print(DIRECTION_MOTOR_CHANNEL_PIN_A);
  Serial.print("\n      ENABLE: ");
  Serial.print(SIGNAL_ENABLE_PIN_MAIN);
  Serial.print("\n     CURRENT: ");
  Serial.print(CURRENT_MONITOR_PIN_MAIN);

  Serial.print("\n\nDCC SIG PROG: ");
  Serial.print(DCC_SIGNAL_PIN_PROG);
  Serial.print("\n   DIRECTION: ");
  Serial.print(DIRECTION_MOTOR_CHANNEL_PIN_B);
  Serial.print("\n      ENABLE: ");
  Serial.print(SIGNAL_ENABLE_PIN_PROG);
  Serial.print("\n     CURRENT: ");
  Serial.print(CURRENT_MONITOR_PIN_PROG);

  Serial.print("\n\nNUM TURNOUTS: ");
  Serial.print(EEStore::eeStore->data.nTurnouts);
  Serial.print("\n     SENSORS: ");
  Serial.print(EEStore::eeStore->data.nSensors);
  Serial.print("\n     OUTPUTS: ");
  Serial.print(EEStore::eeStore->data.nOutputs);
  
  Serial.print("\n\nINTERFACE:    ");
  #if COMM_TYPE == 0
    Serial.print("SERIAL");
  #elif COMM_TYPE == 1
    Serial.print("\nPORT:         ");
    Serial.print(ETHERNET_PORT);
    Serial.print("\nIP ADDRESS:   ");
 
    Serial.print(WiFi.localIP());

    #ifdef IP_ADDRESS
      Serial.print(" (STATIC)");
    #else
      Serial.print(" (DHCP)");
    #endif
  
  #endif
  Serial.print("\n\nPROGRAM HALTED - PLEASE RESTART ARDUINO");

  while(true);
}

///////////////////////////////////////////////////////////////////////////////
//
// WIFI SETUP AND NTP RETRIEVE

void setupWifi() {
    // check for the WiFi module:
    if (WiFi.status() == WL_NO_MODULE) {
      Serial.println("Communication with WiFi module failed!");
      // don't continue
      while (true);
    }

    String fv = WiFi.firmwareVersion();
    if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
      Serial.println("Please upgrade the firmware");
    }

    // attempt to connect to WiFi network:
    while (status != WL_CONNECTED) {
      Serial.print("Attempting to connect to WPA SSID: ");
      // Connect to WPA/WPA2 network:
      #ifdef IP_ADDRESS
        WiFi.config(IP_ADDRESS);
      #endif
      status = WiFi.begin(SECRET_SSID, SECRET_PASS);      
    }

    // you're connected now, so print out the data:
    Serial.print("You're connected to the network");
    printCurrentNet();
    printWifiData();
    
  }

  void RetrieveNTPTime() {
    RTC.begin();
    Serial.println("\nStarting connection to time server...");
    timeClient.begin();
    timeClient.update();

    // Get the current date and time from an NTP server and convert
    // it to UTC +2 by passing the time zone offset in hours.
    // You may change the time zone offset to your local one.
    auto timeZoneOffsetHours = TIMEZONE_OFFSET;
    auto unixTime = timeClient.getEpochTime() + (timeZoneOffsetHours * 3600);
    Serial.print("Unix time = ");
    Serial.println(unixTime);
    RTCTime timeToSet = RTCTime(unixTime);
    RTC.setTime(timeToSet);

    // Retrieve the date and time from the RTC and print them
    RTCTime currentTime;
    RTC.getTime(currentTime); 
    Serial.println("The RTC was just set to: " + String(currentTime));
  }

  void printWifiData() {
    // print your board's IP address:
    IPAddress ip = WiFi.localIP();
    Serial.print("IP Address: ");
    
    Serial.println(ip);

    // print your MAC address:
    byte mac[6];
    WiFi.macAddress(mac);
    Serial.print("MAC address: ");
    printMacAddress(mac);
  }

  void printCurrentNet() {
    // print the SSID of the network you're attached to:
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());

    // print the MAC address of the router you're attached to:
    byte bssid[6];
    WiFi.BSSID(bssid);
    Serial.print("BSSID: ");
    printMacAddress(bssid);

    // print the received signal strength:
    long rssi = WiFi.RSSI();
    Serial.print("signal strength (RSSI):");
    Serial.println(rssi);

    // print the encryption type:
    byte encryption = WiFi.encryptionType();
    Serial.print("Encryption Type:");
    Serial.println(encryption, HEX);
    Serial.println();
  }

  void printMacAddress(byte mac[]) {
    for (int i = 0; i < 6; i++) {
      if (i > 0) {
        Serial.print(":");
      }
      if (mac[i] < 16) {
        Serial.print("0");
      }
      Serial.print(mac[i], HEX);
    }
    Serial.println();
  }