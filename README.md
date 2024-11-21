# What is DCC++ Uno R4

The goal of this project is to port DCC++ BaseStation from Gregg E. Berman https://github.com/DccPlusPlus/BaseStation to the Arduino Uno R4 Wifi.

Due to the Uno R4 Wifi capability, the base station will only require 2 components:
1. An Arduino Uno R4 WiFi board 
1. An Arduino Motor Shield.

# Disclaimer
> <b>:warning: This software comes with no warranty</b>

This is work in progress. Currently the develop branch contains an alpha version of the base station. It has not been deeply tested.

With following setup Arduino Uno R4 WiFi + Arduino motor shield, operating cabs through Serial or WiFi connection works.

>:warning: Programming line does not work! 
It seems that analog pins from Uno R4 behaves differently than the one from Uno R3.

# References

[Renesas RA4M1 processor family documentation](https://www.renesas.com/en/document/mah/renesas-ra4m1-group-users-manual-hardware?r=1054146)

[Renesas Arduino libraries](https://github.com/arduino/ArduinoCore-renesas)

[Renesas RA4M1 header file](https://github.com/arduino/ArduinoCore-renesas/blob/main/variants/UNOWIFIR4/includes/ra/fsp/src/bsp/cmsis/Device/RENESAS/Include/R7FA4M1AB.h)

[Arduino Uno R4 pinout](https://docs.arduino.cc/resources/pinouts/ABX00087-full-pinout.pdf)
