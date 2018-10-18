/*
 * @brief Trackio Class para gestión de SIM868 (GPRS/GPS)
 *
 * Copyright 2018 M2M System Source S.L
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * @file Trackio.cpp
 * @author Jordi Enguídanos <jenguidanos@m2msystemsource.com>
 */

#include <Arduino.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>

#include "Trackio.h"

#define OK         (char *) "OK"
#define ERROR      (char *) "ERROR"
#define READY      (char *) "READY"
#define CONNECT_OK (char *) "CONNECT OK"

struct Conf cfg;

/**
 * @brief Almacena la lectura del Serial del modem. Se utiliza principalmente
 * en Trackio::sendCommand()
 */
char buffer[120];

/**
 * @brief Indica si se mostrarán mensajes de log (solo afecta al método
 * Trackio::sendComman())
 */
bool __DEBUG = true;

/**
 * @brief Cada vez que un comando AT falla se suma 1. Al llegar a X se hará
 * un reinicio del microcontrolador a través de Watchdog
 *
 * El valor de cuantos fallos se admite está en el método Trackio::sendCommand()
 */
uint8_t modemSerialsFails = 0;

/**
 * @brief Lleva un conteo de cuantas veces falla una conexión TCP.
 *
 * A partir de determinado número se reiniciará el micro utilizando watchdog.
 * El valor de cuantos fallos se admite está en el método Trackio::openTcp()
 */
uint8_t openTcpFails = 0;

/**
 * @brief Número de segundos que se irá a dormir (gneralmente gpsInterval)
 *
 * El método Trackio::sleepNow(int secs) establece los segundos. Este valor
 * se pasa a sleep_seconds_remaining la cual decrementa el número de segundos
 * dentro del macro ISR()
 */
volatile int sleep_seconds_remaining;

// interrupt raised by the watchdog firing
// when the watchdog fires, this function will be executed
// remember that interrupts are disabled in ISR functions
// now we must check if the board is sleeping or if this is a genuine
// interrupt (hanging)
ISR(WDT_vect)
{
  // Check if we are in sleep mode or it is a genuine WDR.
  if (sleep_seconds_remaining > 0) {
    // not hang out, just waiting
    // decrease the number of remaining avail loops
    sleep_seconds_remaining = sleep_seconds_remaining - 1;
    wdt_reset();
  } else {
    // must be rebooted
    // configure
    MCUSR = 0;                          // reset flags

                                        // Put timer in reset-only mode:
    WDTCSR |= 0b00011000;               // Enter config mode.
    WDTCSR =  0b00001000 | 0b000000;    // clr WDIE (interrupt enable...7th from left)
                                        // set WDE (reset enable...4th from left), and set delay interval
                                        // reset system in 16 ms...
                                        // unless wdt_disable() in loop() is reached first

    // reboot
    while(1);
  }
}

bool Trackio::begin() {
  Trackio::configure();

  SerialMon.begin(9600);
  SerialSim.begin(9600);
  delay(100);

  SerialMon.print(F("- Trackio "));
  SerialMon.print(F(VERSION));
  SerialMon.println(F(" START -"));

  Trackio::loadConf();

  digitalWrite(LED01, LOW);

  if (!Trackio::powerOn()) {
    return false;
  }

  Trackio::transmissionClockCounter = 0;
  Trackio::printInfo();
  return true;
}

void Trackio::configure () {
  // onboard led
  pinMode(LED01, OUTPUT);
  digitalWrite(LED01, HIGH);

  // control del simcom
  pinMode(GSM_PWRKEY, OUTPUT);
  pinMode(GSM_PWREN, OUTPUT);
  pinMode(GPS_EN, OUTPUT);
  pinMode(GSM_STATUS, INPUT);
  pinMode(IO04, OUTPUT);
  pinMode(IO05, INPUT);
  pinMode(IO06, OUTPUT); // actuador externo (rele, led...)
  pinMode(IO07, OUTPUT); // actuador externo (rele, led...)
  pinMode(MISO, OUTPUT); // actuador externo (rele, led...)
  pinMode(MOSI, OUTPUT); // actuador externo (rele, led...)

  Trackio::_delay(1000);

  SerialMon.println(F("Configure OK"));
}

void Trackio::loadConf () {
  eeprom_read_block((void*)&cfg, (void*)0, sizeof(cfg));

  if (cfg.eeprom != RH_configVersion) {
    SerialMon.println(F("Caragando configuración por primera vez..."));

    // RECUERDA! Si realizas un cambio aquí, modifica la variable configVersion,
    // al inicio de este archivo, para asegurar que los cambios se aplican
    cfg.battMode = RH_battMode;
    cfg.deepSleep = RH_deepSleep;
    cfg.eeprom = RH_eeprom;
    cfg.gpsInterval = RH_gpsInterval; // multiplicado por cfg.transmissionClock
    cfg.opmode = RH_opmode;
    cfg.primaryOpMode = RH_primaryOpMode;
    cfg.sleep = RH_sleep;
    cfg.transmissionClock = RH_transmissionClock;
    cfg.requiredVbat = RH_requiredVbat;
    cfg.requiredVin = RH_requiredVin;
    cfg.requiredVsys5v = RH_requiredVsys5v;

    eeprom_write_block((const void*)&cfg, (void*)0, sizeof(cfg));
  }

  SerialMon.println(F("Configuración cargada"));
}

void Trackio::saveConf () {
  eeprom_write_block((const void*)&cfg, (void*)0, sizeof(cfg));
}

bool Trackio::powerOn () {
  uint8_t gsm_status;

  Trackio::powerOff();
  Trackio::_delay(1000);
  digitalWrite(GSM_PWREN, HIGH);

  digitalWrite(LED01, HIGH);

  for (char i=0; i<5; i++) {
    gsm_status = digitalRead(GSM_STATUS);
    if (gsm_status== HIGH){
      SerialMon.println(F("GSM HIGH!!"));
      break;
    } else {
      digitalWrite(LED01, !digitalRead(LED01));
      SerialMon.println(F("GSM LOW!"));
      digitalWrite(GSM_PWRKEY, HIGH);
      Trackio::_delay(1500);
      digitalWrite(GSM_PWRKEY, LOW);
      Trackio::_delay(1500);
    }
  }

  digitalWrite(LED01, LOW);

  if (!gsm_status) {
    // No se ha podido encender el modem. Revisar que GSM_PWREN, GSM_STATUS y
    // GSM_PWRKEY son los pines correctos.
    return false;
  }

  if (!Trackio::powerOnGps()) {
    // se quiere usar el GPS pero el poweron ha fallado, revisar pin GPS_EN
    // y conexión del módulo esclavo
    return false;
  }
  return true;
}

void Trackio::powerOff () {
  SerialMon.println("POWEROFF!");
  digitalWrite(LED01, LOW);

  digitalWrite(GSM_PWREN, LOW);
  Trackio::_delay(100);
  digitalWrite(GPS_EN, LOW);
  Trackio::_delay(100);
}

// #############################################################################
void Trackio::printInfo () {
  Trackio::getImei();
  Trackio::printIccid();
  Trackio::printBattery();
  Trackio::getBattery();
}

void Trackio::getImei () {
  char x[8] = "AT+CGSN";
  // guardamos el imei en el buffer
  Trackio::sendCommand(x);

  char * splitImei;

  splitImei = strtok(buffer, "\n");
  splitImei = strtok(NULL, "\n");
  strncpy(imei, splitImei, 15);

  SerialMon.print(F("IMEI: ")); SerialMon.println(imei);
}

void Trackio::printIccid () {
  Trackio::sendCommand((char *) "AT+CCID");
}

void Trackio::printBattery () {
  char x[7] = "AT+CBC";
  Trackio::sendCommand(x);
}

void Trackio::getBattery () {
  if (cfg.battMode == 1) {
    Trackio::getSimcomBattery();
  } else {
    Trackio::getAnalogBattery();
  }

  Trackio::checkLowBattery();
}

void Trackio::getSimcomBattery() {
  char x[7] = "AT+CBC";
  Trackio::sendCommand(x);

  char * split;
  char * split2;
  char cbc[50];

  split = strtok(buffer, "\n"); // empty linea
  split = strtok(NULL, "\n"); // result
  strcpy(cbc, split);
  split2 = strtok(cbc, ",");
  split2 = strtok(NULL, ","); // porcentage de batería
  split2 = strtok(NULL, ","); // batería

  Trackio::vbat = atoi(split2);
  Trackio::vin = 0;
  Trackio::vsys_5v = 0;
}

void Trackio::checkLowBattery () {
  char low = 0;

  if (cfg.requiredVbat >0 && Trackio::vbat < cfg.requiredVbat) low = 1;
  if (cfg.requiredVin > 0 && Trackio::vin < cfg.requiredVin) low = 1;
  if (cfg.requiredVsys5v > 0 && Trackio::vsys_5v < cfg.requiredVsys5v) low = 1;

  if (low == 1) {
    // reset serial fails, evita reinicio antes de deep sleep
    modemSerialsFails = 0;
    SerialMon.println("Entramos en LOW MODE");
    // establecemos el modo de bajo consumo y apagamos el simcom
    cfg.opmode = OP_LOW;
    Trackio::powerOff();
  } else {
    // si venimos de bajo consuno establecemos el modo OP_STARTUP para
    // iniciar la secuencia de arranque, que incluye encender el Simcom
    // (y salir de bajo consumo)
    if (cfg.opmode == OP_LOW) {
      cfg.opmode = OP_STARTUP;
    }
  }
}

void Trackio::getAnalogBattery() {
  uint16_t lectura_mV=0, result=0;
  float aux_f=0;

  // --
  lectura_mV = Trackio::readAnalogBatt(VSYS_PIN);
  aux_f = (float) (lectura_mV / aux_bat);
  result = (uint16_t) aux_f;
  Trackio::vsys_5v = result;
  SerialMon.print("VSYS_5v: "); SerialMon.print(result);

  // --
  lectura_mV = Trackio::readAnalogBatt(VBAT_PIN);
  aux_f = (float) (lectura_mV / aux_bat);
  result = (uint16_t) aux_f - 100L;
  Trackio::vbat = result;
  SerialMon.print(" - VBAT: "); SerialMon.print(result);

  // --
  lectura_mV = Trackio::readAnalogBatt(VIN_PIN);
  aux_f = (float) (lectura_mV / aux_ext_bat);
  result = (uint16_t) aux_f;
  Trackio::vin = result;
  SerialMon.print(" - VIN: "); SerialMon.println(result);
  SerialMon.println(F(""));
}

uint16_t Trackio::readAnalogBatt(byte adc_pin) {
  int readingADC;
  float reading_mV;
  uint16_t result;

  readingADC = analogRead(adc_pin);
  readingADC = (int) readingADC;
  reading_mV = (float) readingADC * (float) mV_step_used;
  reading_mV = (float) reading_mV *1000;
  result = (uint16_t) reading_mV;
  return(result);
}

void Trackio::printPin () {
  char x[9] = "AT+CPIN?";
  Trackio::sendCommand(x);
}

// #############################################################################
bool Trackio::checkStatus () {
  if (!Trackio::checkModem()) {
    SerialMon.println(F("FAIL checkModem"));
    return false;
  }

  Trackio::getSignalStrength();

  if (!Trackio::checkCreg()) {
    SerialMon.println(F("FAIL checkCreg"));
    return false;
  }

  return true;
}

void Trackio::getSignalStrength () {
  char x[10] = "AT+CSQ";

  if (Trackio::sendCommand(x, OK)) {
    char * split;
    split = strtok(buffer, " ");
    split = strtok(NULL, ",");
    strcpy(Trackio::gsm, split);
  }
}

bool Trackio::checkCreg () {
  if (!Trackio::atOk) return false;
  Trackio::cregOk = false;

  // Verificamos el creg 10 veces
	for (int i=0; i<10; i++) {
		char x2[9] = "AT+CREG?";
	  Trackio::sendCommand(x2);

		if (strstr(buffer, ",5") || strstr(buffer, ",1")) {
	    Trackio::cregOk = true;
			return true;
	  }

		Trackio::_delay(1000);
	}

	return false;
}

bool Trackio::checkModem () {
	Trackio::atOk = false;

	if (Trackio::sendCommand((char *) "AT", OK)) {
		Trackio::atOk = true;

		Trackio::_delay(100);

		char x2[5] = "ATE0";
		Trackio::sendCommand(x2);
		Trackio::_delay(100);
	}

	return Trackio::atOk;
}

// #############################################################################
bool Trackio::openTcp () {
  Trackio::tcpOk = false; // reset

  // Obtener el estado del servicio GPRS
  if (!Trackio::gprsIsOpen()) {
    return false;
  }

  // abrimos puerto
  char cmd[100];
  sprintf(cmd, "AT+CIPSTART=\"TCP\",\"%s\",\"%s\"", RH_SERVER, RH_PORT);
  Trackio::sendCommand(cmd);
  Trackio::_delay(2000);

  // procesamos respuesta
  char * response;
  response = strtok(buffer, "\n"); // 1º empty line
  response = strtok(NULL, "\n"); // 2º response

  // si ya está abierto Sim868 devolverá OK, sino READY CONNECT
  if (strstr(response, OK) || strstr(response, READY)) {
    openTcpFails = 0;
    SerialMon.println(F("TCP OPEN OK!"));
    Trackio::tcpOk = true;
    return true;
  }

  SerialMon.println(F("ERROR TCP OPEN!"));
  openTcpFails++;
  if (openTcpFails == 3) {
    SerialMon.println("El TCP ha fallado en multiples ocasiones - Hard Reset!");
    while (1) {} // llamamos al watchdog.
  }

  return false;
}

void Trackio::closeTcp (int mode) {
  char cmd[15];
  sprintf(cmd, "AT+CIPCLOSE=%i", mode);
  Trackio::sendCommand(cmd);
  Trackio::tcpOk = false;
  digitalWrite(LED01, LOW);
}

bool Trackio::tcpIsOpen () {
  char cmd[20] = "AT+CIPSTATUS";
  return Trackio::sendCommand(cmd, CONNECT_OK, 10);
}

bool Trackio::sayHello () {
  char hello[50];

  // debemos transmitir al servidor el modo operacional, OP_TCP/AUTO
  int opmode = 1; // por defecto TCP
  if (cfg.opmode == OP_AUTO) opmode = 2; // si no AUTO

  // x debería ser prescindible, pero si inserto Trackio::imei
  // directamente en el sprintf se produce un segmentation fault error
  // que mis altas capacidades en el lenguaje C no logran entender
  char x[20];
  strcpy(x, Trackio::imei);
  sprintf(hello, "%s|%d", x, opmode);

  if (Trackio::tcpOk && Trackio::transmit(hello)) {
    SerialMon.print(F("buffer: ")); SerialMon.println(buffer);
    cfg.opmode = cfg.primaryOpMode;
    digitalWrite(LED01, HIGH);
    return true;
  }

  cfg.opmode = OP_STARTUP;
  return false;
}

bool Trackio::transmit (char * msg) {
  if (!Trackio::tcpOk) return false;

  char cipsend[20];
  sprintf(cipsend, "AT+CIPSEND=%i", strlen(msg));

  // enviamos los dos comandos, CIPSEND y el mensaje
  Trackio::sendCommand(cipsend);
  Trackio::sendCommand(msg);

  if (strstr(buffer, "SEND OK")) {
    SerialMon.println(F("SEND OK!!"));
    delay(100); // relax!
    // Comprobamos si el servidor nos ha devuelto una respuesta
    char * response = getTransmitResponse(buffer);
    if (isCommand(response)) {
      SerialMon.print(F("response: ")); SerialMon.println(response);
      char * cmd = Trackio::extractCommand(response);
      if (Trackio::processCommand(cmd)) {
        SerialMon.println(F("Command PROCESSED: ")); SerialMon.println(cmd);
      }
    } else {
      SerialMon.println(F("Response is not a commnand"));
    }

    if (cfg.opmode == OP_TCP) {
      SerialMon.println(F("Listening for TCP commands..."));
    }

    return true;
  }

  Trackio::tcpOk = false;
  SerialMon.println(F("ERROR SEND DATA!!"));
  return false;
}

bool Trackio::transmitGps () {
  if (cfg.gpsInterval < 1) return false; // deshabilitado

  char gpsdata[120];
  Trackio::resetGpsData();
  Trackio::getGps();

  if (cfg.opmode == OP_TCP) {
    // en el modo bidireccional no enviamos el imei, el socket del servidor
    // ya tiene almacenado este dato por lo que no es necesario enviarlo de
    // forma recurrente
    if (Trackio::gps.fix == 0) {
      sprintf(gpsdata, "0,%s", Trackio::gsm);
    } else {
      sprintf(gpsdata, "1,%s,%s,%s,%s,%s,%s,%s,%s,%s",
        Trackio::gps.time,
        Trackio::gps.lat,
        Trackio::gps.lon,
        Trackio::gps.alt,
        Trackio::gps.sog,
        Trackio::gps.cog,
        Trackio::gps.hdop,
        Trackio::gps.sats,
        Trackio::gsm
      );
    }
  } else {
    // En el modo OP_AUTO, que se cierra el TCP, el
    if (Trackio::gps.fix == 0) {
      sprintf(gpsdata, "%s|0,%s", Trackio::imei, Trackio::gsm);
    } else {
      sprintf(gpsdata, "%s|1,%s,%s,%s,%s,%s,%s,%s,%s,%s",
        Trackio::imei,
        Trackio::gps.time,
        Trackio::gps.lat,
        Trackio::gps.lon,
        Trackio::gps.alt,
        Trackio::gps.sog,
        Trackio::gps.cog,
        Trackio::gps.hdop,
        Trackio::gps.sats,
        Trackio::gsm
      );
    }
  }

  // add battery
  char batt[15];
  sprintf(batt, "|%u,%u,%u", Trackio::vbat, Trackio::vin, Trackio::vsys_5v);
  strcat(gpsdata, batt);

  Trackio::resetGpsData();

  return Trackio::transmit(gpsdata);
}

char * Trackio::getTransmitResponse (char * buffer) {
  char * response;

  response = strtok(buffer, "\n"); // empty line
  response = strtok(NULL, "\n"); // "SEND OK"
  response = strtok(NULL, "\n"); // response

  return response;
}

// #############################################################################

bool Trackio::tcpHasCommand () {
  uint8_t counter = 0;
  strcpy(buffer, "");
  bool gettingCommand = false;
  bool hasCommand = false;

  if (SerialSim.available()) {
    while (SerialSim.available() > 0) {
      char x = SerialSim.read();

      if (35 == (int) x && !hasCommand) { // init command with #
        gettingCommand = true;
        SerialMon.println("START command:");
      } else if (36 == (int) x) { // end command with $
        gettingCommand = false;
        hasCommand = true;
        SerialMon.println("");
        SerialMon.println("END command:");
        Trackio::cmd[counter] = '\0';
      } else if (gettingCommand) {
        SerialMon.print(x);
        Trackio::cmd[counter] = x;
        counter++;
      }

      Trackio::_delay(1);
    }

    SerialMon.print("Command: "); SerialMon.println(Trackio::cmd);

    if (hasCommand) {
      return true;
    }
  }

  return false;


  return false;
  __DEBUG = false;
  strcpy(buffer, "");

  Trackio::sendCommand((char *) "AT+CIPRXGET=2,50");
  char * split;
  split = strtok(buffer, "\n");
  split = strtok(NULL, "\n");
  split = strtok(NULL, "\n");
  char cmd[50];
  strcpy(cmd, split);
  // primer y último caracter como # y $
  if ((int) cmd[0] == 35 || (int) cmd[strlen(cmd) -1] == 36) {
    strcpy(Trackio::cmd, Trackio::extractCommand(cmd));
    return true;
  }

  __DEBUG = true;
  return false;
}

bool Trackio::processCommand (char * cmd) {
  Command command = Trackio::splitCommand(cmd);

  // damos por hecho que es válido (luego se verá)
  bool isValidCmd = true;

  if      (strcmp(command.property, "IO6") == 0) return Trackio::cmd_setIO(IO06, command.value);
  else if (strcmp(command.property, "IO7") == 0) return Trackio::cmd_setIO(IO07, command.value);
  else if (strcmp(command.property, "MOSI") == 0) return Trackio::cmd_setIO(MOSI, command.value);
  else if (strcmp(command.property, "MISO") == 0) return Trackio::cmd_setIO(MISO, command.value);
  else if (strcmp(command.property, "CFG") == 0)  return Trackio::cmd_setConf(command.value);
  else {
    // finalmente no es válido...
    isValidCmd = false;
    SerialMon.print(F("UNKNOW COMMAND: ")); SerialMon.println(command.property);
  }

  return isValidCmd;
}

Command Trackio::splitCommand (char * cmd) {
  SerialMon.print(F("splitCommand: ")); SerialMon.print(cmd);
  Command command;

  char * split;

  split = strtok(cmd, "|");
  if (split != NULL) strcpy(command.property, split);

  split = strtok(NULL , "|");
  if (split != NULL) strcpy(command.value, split);

  SerialMon.print(F(" | prop: ")); SerialMon.print(command.property);
  SerialMon.print(F(" | value: ")); SerialMon.println(command.value);

  return command;
}

bool Trackio::isCommand (char * command) {
  char * split;
  uint8_t count = 0;
  char cmd[50];
  strcpy(cmd, command);

  // primer y último caracter como # y $
  if ((int) cmd[0] != 35 || (int) cmd[strlen(cmd) -1] != 36) {
    return false;
  }

  split = strtok(cmd, "|");
  if (split != NULL) count++;

  split = strtok(NULL, "|");
  if (split != NULL) count++;

  if (count == 2) {
    return true;
  }

  return false;
}

char * Trackio::extractCommand (char * cmd) {
  static char __cmd[50];
  SerialMon.print(F("x")); SerialMon.print(cmd); SerialMon.println(F("x"));

  uint8_t len = strlen(cmd) - 1;
  int i;

  for (i = 1; i < len; i++) {

    if ((int) cmd[i] != 35 && (int) cmd[i] != 36) {
      SerialMon.print(F("Extract Command i: ")); SerialMon.println(cmd[i]);
      __cmd[i] = '\0';
      __cmd[i - 1] = cmd[i];
    }
  }

  SerialMon.print(F("Extract Command: ")); SerialMon.println(__cmd);
  return __cmd;
}

bool Trackio::cmd_setIO (int IO, char * status) {
  if (IO < 0 || IO > 100) return false;

  if      (strcmp(status, "ON")  == 0) digitalWrite(IO, HIGH);
  else if (strcmp(status, "OFF") == 0) digitalWrite(IO, LOW);
  else if (strcmp(status, "OUT") == 0) pinMode(IO, OUTPUT);
  else if (strcmp(status, "IN")  == 0) pinMode(IO, INPUT);

  return true;
}

bool Trackio::cmd_setConf (char * command) {
  SerialMon.print(F("setConf: ")); SerialMon.println(command);
  const uint8_t maxConfs = 3;
  char confs[maxConfs][20] = {};
  uint8_t count = 0;

  char * cmds;
  cmds = strtok(command, ";");
  while (cmds != NULL) {
    strcpy(confs[count], cmds);
    cmds = strtok(NULL, ";");
    count++;
  }

  for (uint8_t i = 0; i < maxConfs; i++) {
    if (confs[i]) {
      Trackio::applyConf(confs[i]);
    }
  }

  Trackio::saveConf();
  return true;
}

bool Trackio::applyConf (char * conf) {
  SerialMon.print(F("applyConf: ")); SerialMon.println(conf);
  char * cmd;
  char x[50];
  char prop[10];
  char value[5];

  strcpy(x, conf);

  cmd = strtok(x, ":");
  if (cmd == NULL) return false;

  strcpy(prop, cmd);
  cmd = strtok(NULL, ":");
  strcpy(value, cmd);

  if      (strcmp(prop, "dslp") == 0) cfg.deepSleep = atoi(value);
  else if (strcmp(prop, "gpsi") == 0) cfg.gpsInterval = atoi(value);
  else if (strcmp(prop, "op") == 0)   cfg.opmode = atoi(value);
  else if (strcmp(prop, "pop") == 0)  cfg.primaryOpMode = atoi(value);
  else if (strcmp(prop, "slp") == 0)  cfg.sleep = atoi(value);

  return true;
}

// #############################################################################

bool Trackio::enableGprs () {
  // Obtener el estado del servicio GPRS
  if (!Trackio::gprsIsOpen()) {
    return false;
  }

  char apnCmd[150];
  sprintf(
    apnCmd,
    "AT+CSTT=\"%s\",\"%s\",\"%s\"",
    RH_APN, RH_APN_USER, RH_APN_PASS
  );

  Trackio::sendCommand(apnCmd, 1000);
  Trackio::_delay(1000);

  Trackio::sendCommand((char *) "AT+CIICR", 1000);
  Trackio::_delay(1000);

  Trackio::sendCommand((char *) "AT+CIFSR", 100);
  Trackio::_delay(1000);

  return true;
}

bool Trackio::gprsIsOpen () {
  if (!Trackio::cregOk) return false;

	char x2[10] = "AT+CGATT?";
  Trackio::sendCommand(x2);

	if (strstr(buffer, "OK")) {
		SerialMon.println(F("GPRS SERVICE OK!!"));
		return true;
	}

	SerialMon.println(F("ERROR GPS SERVICE!!"));
	return false;
}

// #############################################################################

bool Trackio::powerOnGps () {
  char x1[13] = "AT+CGNSPWR=1";
  if (!Trackio::sendCommand(x1, (char *) "OK")) {
    return false; }
  Trackio::_delay(1500);

  // 2
  digitalWrite(GPS_EN, HIGH);
  Trackio::_delay(1000);

  // 3
  char x2[13] = "AT+CGPSRST=0";
  Trackio::sendCommand(x2);

  return true;
}

void Trackio::getGps () {
  if (cfg.gpsInterval < 1) return;

  Trackio::getBattery();
  Trackio::getSignalStrength();

  char cmd[11] = "AT+CGNSINF";
  Trackio::sendCommand(cmd);

  char * token = strtok(buffer, "\n");
  token = strtok(NULL, "\n");
  char * token2 = strtok(token, " ");
  token2 = strtok(NULL, " ");

  Trackio::parseGps(token2);
}

void Trackio::resetGpsData () {
  Trackio::gps.fix = 0;
}

bool Trackio::gpsHasFix (char * gps) {
  Trackio::gpsFix = false;

  for (uint8_t i = 0; i <= 20; i++) {
    if (i == 2) {
      // ascii char 48 = 0
      // ascii char 49 = 1
      if ((int) gps[i] == 49) {
        Trackio::gpsFix = true;
        parseGps(gps);
        return true;
      }
    }
  }

  return false;
}

void Trackio::parseSimcomTime (char * time) {
  char * split;
  split = strtok(time, ".");
  strcpy(Trackio::gps.time, split);
}

void Trackio::parseGps (char * gps) {
  char * split = strtok(gps, ",");

  split = strtok(NULL, ","); Trackio::gps.fix = atoi(split);
  split = strtok(NULL, ","); strcpy(Trackio::gps.time, split);
  split = strtok(NULL, ","); strcpy(Trackio::gps.lat, split);
  split = strtok(NULL, ","); strcpy(Trackio::gps.lon, split);
  split = strtok(NULL, ","); strcpy(Trackio::gps.alt, split);
  split = strtok(NULL, ","); strcpy(Trackio::gps.sog, split);
  split = strtok(NULL, ","); strcpy(Trackio::gps.cog, split);
  split = strtok(NULL, ",");
  split = strtok(NULL, ","); strcpy(Trackio::gps.hdop, split);
  split = strtok(NULL, ",");
  split = strtok(NULL, ",");
  split = strtok(NULL, ","); strcpy(Trackio::gps.sats, split);
  split = strtok(NULL, ",");
  split = strtok(NULL, ",");
}

// #############################################################################
void Trackio::sleepNow () {
  Trackio::sleepNow(cfg.gpsInterval);
}

void Trackio::sleepNow (float seconds) {
  float cycles = seconds / 8;
  SerialMon.println(F("-------------- SLEEP NOW --------------"));

  sleep_seconds_remaining = cycles; // defines how many cycles should sleep

  // Set sleep to full power down.  Only external interrupts or
  // the watchdog timer can wake the CPU!
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);

  // Turn off the ADC while asleep.
  power_adc_disable();

  if (cfg.deepSleep) Trackio::powerOff();

  while (sleep_seconds_remaining > 0) { // while some cycles left, sleep!
    SerialMon.print(F(" REMAINING:")); SerialMon.print(sleep_seconds_remaining);
    // Enable sleep and enter sleep mode.
    sleep_mode();

    // CPU is now asleep and program execution completely halts!
    // Once awake, execution will resume at this point if the
    // watchdog is configured for resume rather than restart


    // When awake, disable sleep mode
    sleep_disable();
  }

  SerialMon.println(F(""));
  SerialMon.println(F("-------------- WAKEUP --------------"));

  // put everything on again
  power_all_enable();
}

void Trackio::configureSleep () {
  // SerialMon.println("Configure Watchdog / Sleep");

  cli();                           // disable interrupts for changing the registers

  MCUSR = 0;                       // reset status register flags

                                   // Put timer in interrupt-only mode:
  WDTCSR |= 0b00011000;            // Set WDCE (5th from left) and WDE (4th from left) to enter config mode,
                                   // using bitwise OR assignment (leaves other bits unchanged).
  WDTCSR =  0b01000000 | 0b100001; // set WDIE: interrupt enabled
                                   // clr WDE: reset disabled
                                   // and set delay interval (right side of bar) to X seconds
  /*
   * 16 ms:     0b000000
   * 500 ms:    0b000101
   * 1 second:  0b000110
   * 2 seconds: 0b000111
   * 4 seconds: 0b100000
   * 8 seconds: 0b100001
   */
  sei();                           // re-enable interrupts
}

// #############################################################################

void Trackio::_delay (int time) {
  wdt_reset();
  delay(time);
  wdt_reset();
}

// #############################################################################

bool Trackio::sendCommand (char * cmd, char * validate) {
	return Trackio::sendCommand(cmd, validate, 0);
}

bool Trackio::sendCommand (char * cmd, int time) {
	return Trackio::sendCommand(cmd, NULL, time);
}

bool Trackio::sendCommand (char * cmd) {
	return Trackio::sendCommand(cmd, NULL, 0);
}

bool Trackio::sendCommand (char *cmd, char * validate, int time) {
	int count = 0;
  int indexPosition = 0;

  while (SerialSim.available()) SerialSim.read();

	// vaciamos el buffer previo
  strcpy(buffer, "");

  if (__DEBUG) {
    SerialMon.println(F(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"));
    SerialMon.print(F("SerialSim > ")); SerialMon.println(cmd);
  }

	// enviamos el comando al modem
  SerialSim.println(cmd);
  SerialSim.flush();

	Trackio::_delay(time);

	// Esperamos respuesta comprobando el buffer cada 400 millis
  Serial.println("Waiting modem response...");
  while (!SerialSim.available()) {
    SerialMon.print(F("."));
    Trackio::_delay(100);
    count++;
    if (count > 100) {
      SerialMon.println(F("sendSerialAt failed"));
      modemSerialsFails++;
      if (modemSerialsFails > 10) {
        SerialMon.println("Muchos fallos, reiniciando en 8 secs, o pulsa botón reset...");
        delay(10000);
      }
      return false;
    }
  }

  modemSerialsFails = 0; // reset contador

	// acomodamos la respuesta en buffer
  while(SerialSim.available() > 0) {
    buffer[indexPosition] = SerialSim.read();
    indexPosition++;
    Trackio::_delay(5);
  }

  if (__DEBUG) {
    SerialMon.println(buffer);
	  SerialMon.println(F("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<"));
  }

	// comprobamos si hay datos que validar
	if (validate != NULL && strlen(validate) >= 0) {
		if (strstr(buffer, validate)) {
			return true;
		}
		return false;
	}

  return true;
}
