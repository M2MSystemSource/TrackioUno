#ifndef TRACKIO_PINS
  #define TRACKIO_PINS

  #include <Arduino.h>
  #include <SoftwareSerial.h>

  #ifdef DEIMOS_SOCKET1_MEGA2560
    #define GSM_PWREN  36 // IO00
    #define GSM_PWRKEY 33  // IO01
    #define GPS_EN     44 // IO02
    #define GSM_STATUS 37 // IO03

    #define SerialSim Serial // comunicación con SIMCOM
    #define SerialMon Serial1 // debug, terminal
  #endif

  #ifdef DEIMOS_SOCKET2_MEGA2560
    #define GSM_PWREN  47 // IO08
    #define GSM_PWRKEY 42  // IO09
    #define GPS_EN     45 // IO10
    #define GSM_STATUS 4 // IO11

    #define SerialSim Serial1 // comunicación con SIMCOM
    #define SerialMon Serial // debug, terminal
  #endif

  #ifdef HALLEY_SOCKET2_MEGA2560
    #define GSM_PWREN  47 // IO08
    #define GSM_PWRKEY 42  // IO09
    #define GPS_EN     45 // IO10
    #define GSM_STATUS 4 // IO11

    #define SerialSim Serial1 // comunicación con SIMCOM
    #define SerialMon Serial // comunicación con SIMCOM
  #endif

  #ifdef HALLEY_SOCKET2_UNO
    #define GSM_PWREN  8
    #define GSM_PWRKEY 9
    #define GPS_EN     10
    #define GSM_STATUS 11

    #define SerialSim Serial     // comunicación con SIMCOM
    extern SoftwareSerial SoftSerial;
    #define SerialMon SoftSerial // comunicación con SIMCOM
  #endif

  #ifdef DEIMOS_SOCKET1_ZEROSAMD21
    #define GSM_PWREN  2
    #define GSM_PWRKEY 5
    #define GPS_EN     6
    #define GSM_STATUS 7

    #define SerialSim Serial1    // comunicación con SIMCOM
    #define SerialMon Serial // comunicación con SIMCOM
    // #define SerialMon SerialUSB // comunicación con SIMCOM
  #endif

  #ifdef DEIMOS_SOCKET1_UNO
    #define GSM_PWREN  4
    #define GSM_PWRKEY 5
    #define GPS_EN     6
    #define GSM_STATUS 7

    #define SerialSim Serial     // comunicación con SIMCOM
    extern SoftwareSerial SoftSerial;
    #define SerialMon SoftSerial // comunicación con SIMCOM
  #endif

  #ifdef PINS_MEGA2560
    #define IO00 36
    #define IO01 33
    #define IO02 44
    #define IO03 37
    #define IO04 34
    #define IO05 43
    #define IO06 46
    #define IO07 48
    #define IO08 47
    #define IO09 42
    #define IO10 45
    #define IO11 4
    #define IO12 32
    #define IO13 26
    #define IO14 31
    #define IO15 27
    #define IO16 39
    #define IO17 25
    #define IO18 24
    #define IO19 30
    #define IO20 28
    #define IO21 23
    #define IO22 29
    #define IO23 22
    #define IO24 35
    #define IO25 40
    #define IO26 41
    #define IO27 38
    #define IO28 28
    #define IO29 9
    #define IO30 8
    #define IO31 7
    #define LED01 13
  #endif

  #ifdef PINS_ZEROSAMD21D
    #define IO00 2
    #define IO01 5
    #define IO02 6
    #define IO03 7
    #define IO04 8
    #define IO05 30
    #define IO06 24
    #define IO07 22
    #define IO08 -1
    #define IO09 -1
    #define IO10 -1
    #define IO11 -1
    #define IO12 -1
    #define IO13 -1
    #define IO14 -1
    #define IO15 -1
    #define IO16 -1
    #define IO17 -1
    #define IOA0 A0
    #define LED01 9
  #endif

  #ifdef PINS_UNO
    #define NMI     2
    #define _INT0   3
    #define IO00    4
    #define IO01    5
    #define IO02    6
    #define IO03    7
    #define IO04    8
    #define IO05    A0
    #define IO06    A1
    #define IO07    A2
    #define LED01   9
    #define PWM0    9
    #define RXD     0
    #define TXD     1
    #define SDA     A4
    #define SCL     A5
    #define MOSI    11
    #define MISO    12
    #define SCK     13
    #define CS0     10
    #define _1WIRE  A3		// SJ4 must be closed & SJ1 must be open
    #define AD0     A6
    #define CAPT0   A7
    #define RSTOUT  A3		// SJ1 must be closed & SJ4 must be open
  #endif

#endif
