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
#include <FlashStorage.h>
#include <Adafruit_SleepyDog.h>
#include "Trackio.h"

#include <Readline.h>
ReadLine readLine;

#define OK         (char *) "OK"
#define ERROR      (char *) "ERROR"
#define READY      (char *) "READY"
#define CONNECT_OK (char *) "CONNECT OK"

// OFFSETS para el cálculo en las lecturas analógicas de baterías
const float mV_step_5V=0.0048828125;
const float mV_step_3V3=0.00322265625;
const float mV_step_1V1=0.00107421875;
float mV_step_used;

float reading_mV;
float aux_reading;

const float VBAT_aux=0.3197278911564626;
const float VIN_aux=0.0448901623686724;
const float VSYS_aux=0.3197278911564626;

struct Conf cfg;

FlashStorage(my_flash_store, Conf);

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


bool Trackio::begin() {
  Trackio::configureIOs();

  SerialMon.begin(38400); while (!SerialMon) {}
  SerialSim.begin(38400); while (!SerialSim) {}

  _(F("- Trackio ")); _(F(VERSION)); __(F(" START -"));

  Trackio::blink();
  Trackio::loadConf();

  if (!Trackio::powerOn()) {
    return false;
  }

  Trackio::transmissionClockCounter = 0;
  Trackio::printInfo();
  return true;
}

void Trackio::configureIOs () {
  // onboard led
  pinMode(LED, OUTPUT);

  // control del simcom
  pinMode(GSM_PWRKEY, OUTPUT);
  pinMode(GSM_PWREN, OUTPUT);
  pinMode(GPS_EN, OUTPUT);
  pinMode(GSM_STATUS, INPUT);
  pinMode(IO6, OUTPUT); // actuador externo (rele, led...)
  pinMode(MUX_SW, OUTPUT);
  digitalWrite(MUX_SW, HIGH);

  mV_step_used = mV_step_3V3;
}

void Trackio::loadConf () {

  Conf owner;
  owner = my_flash_store.read();

  if (owner.eeprom != RH_eeprom) {
    __("Cargamos flash por primera vez");
    // RECUERDA! No modifiques nada aquí, revisa `lib/trackio/static-conf.h`
    cfg.battMode = RH_battMode;
    cfg.deepSleep = RH_deepSleep;
    cfg.eeprom = RH_eeprom;
    cfg.gpsInterval = RH_gpsInterval;
    cfg.opmode = RH_opmode;
    cfg.primaryOpMode = RH_primaryOpMode;
    cfg.sleep = RH_sleep;
    cfg.transmissionClock = RH_transmissionClock;
    cfg.requiredVbat = RH_requiredVbat;
    cfg.requiredVin = RH_requiredVin;
    cfg.requiredVsys5v = RH_requiredVsys5v;

    Trackio::saveConf();
  } else {
    __("La configuración ya existe.");
    cfg = owner;
  }

  if (cfg.GPIO6 == HIGH) {
    digitalWrite(IO6, HIGH);
  } else {
    digitalWrite(IO6, LOW);
  }

  __(F("Configuración cargada..."));
}

void Trackio::saveConf () {
  my_flash_store.write(cfg);
}

bool Trackio::powerOn () {
  uint8_t gsm_status;

  Trackio::powerOff();
  Trackio::_delay(1000);
  digitalWrite(GSM_PWREN, HIGH);

  for (char i=0; i<5; i++) {
    gsm_status = digitalRead(GSM_STATUS);
    if (gsm_status== HIGH){
      __(F("GSM HIGH!!"));
      break;
    } else {
      __(F("GSM LOW!"));
      digitalWrite(GSM_PWRKEY, HIGH);
      Trackio::_delay(1500);
      digitalWrite(GSM_PWRKEY, LOW);
      Trackio::_delay(1500);
    }
  }

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
  __("POWEROFF!");
  digitalWrite(LED, LOW);

  digitalWrite(GSM_PWREN, LOW);
  Trackio::_delay(100);
  digitalWrite(GPS_EN, LOW);
  Trackio::_delay(100);
}

// #############################################################################
void Trackio::printInfo () {
  Trackio::getImei();
  Trackio::printIccid();
  Trackio::printPin();
  Trackio::sendCommand((char *) "AT+GSV");
  Trackio::getBattery();
}

void Trackio::getImei () {
  if (Trackio::sendAt((char *) "AT+CGSN", 2)) {
    if (strlen(buffer) == 15) {
      strcpy(Trackio::imei, buffer);
      _(F("   -> IMEI: ")); __(Trackio::imei);
      return;
    }
  }

  _(F("   -> IMEI: ")); __("FAIL");
}

void Trackio::printIccid () {
  Trackio::sendAt((char *) "AT+CCID");
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
    __("Entramos en LOW MODE");
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
  uint16_t lectura_mV;

  __(F("--------------------------------------------------------------"));
  digitalWrite(MUX_SW,LOW);
  delay(10);
  lectura_mV = (float) Trackio::readAnalogBatt(A3);
  Trackio::vbat = (float) (lectura_mV / VBAT_aux);
  _(F("VBAT: ")); __(Trackio::vbat);

  lectura_mV = (float) Trackio::readAnalogBatt(A1);
  Trackio::vsys_5v = (float) (lectura_mV / VSYS_aux);
  _(F("VSYS: ")); __(Trackio::vsys_5v);

  digitalWrite(MUX_SW,HIGH);
  delay(100);
  lectura_mV = (float) Trackio::readAnalogBatt(A2);
  Trackio::vin = (float) (lectura_mV / VIN_aux);
  _(F("VIN: ")); __(Trackio::vin);
  __("");
}

uint16_t Trackio::readAnalogBatt(byte adc_pin) {
  int readingADC;
  float reading_mV;
  uint16_t result;

  readingADC = analogRead(adc_pin);
  readingADC = (int) readingADC;
  reading_mV = (float) readingADC * (float) mV_step_used;
  reading_mV = (float) reading_mV * 1000;
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
    __(F("FAIL checkModem"));
    return false;
  }

  Trackio::getSignalStrength();

  if (!Trackio::checkCreg()) {
    __(F("FAIL checkCreg"));
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
    __(F("TCP OPEN OK!"));
    Trackio::tcpOk = true;
    return true;
  }

  __(F("ERROR TCP OPEN!"));
  openTcpFails++;
  if (openTcpFails == 3) {
    __("El TCP ha fallado en multiples ocasiones - Hard Reset!");
    while (1) {} // llamamos al watchdog.
  }

  return false;
}

void Trackio::closeTcp (int mode) {
  char cmd[15];
  sprintf(cmd, "AT+CIPCLOSE=%i", mode);
  Trackio::sendCommand(cmd);
  Trackio::tcpOk = false;
  Trackio::listeningTcp = false;
  digitalWrite(LED, LOW);
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
    _(F("buffer: ")); __(buffer);
    cfg.opmode = cfg.primaryOpMode;
    digitalWrite(LED, HIGH);
    return true;
  }

  cfg.opmode = OP_STARTUP;
  return false;
}

bool Trackio::transmit (char * msg) {
  // comando para el mondo
  char cipsend[20];
  sprintf(cipsend, "AT+CIPSEND=%i", strlen(msg));

  // enviamos los dos comandos, CIPSEND y el mensaje
  Trackio::sendCommand(cipsend);
  Trackio::sendCommand(msg);

  if (strstr(buffer, "SEND OK")) {
    __(F("SEND OK!!"));
    delay(100); // relax!
    // Comprobamos si el servidor nos ha devuelto una respuesta
    char * response = getTransmitResponse(buffer);
    if (isCommand(response)) {
      _(F("response: ")); __(response);
      char * cmd = Trackio::extractCommand(response);
      if (Trackio::processCommand(cmd)) {
        __(F("Command PROCESSED: ")); __(cmd);
      }
    } else {
      __(F("Response is not a commnand"));
    }

    if (cfg.opmode == OP_TCP) {
      __(F("Listening for TCP commands..."));
      Trackio::listeningTcp = true;
    }

    return true;
  }

  Trackio::listeningTcp = false;
  Trackio::tcpOk = false;
  __(F("ERROR SEND DATA!!"));
  return false;
}

bool Trackio::transmitGps () {
  if (cfg.gpsInterval < 1) return false; // deshabilitado

  char gpsdata[150];
  Trackio::resetGpsData();
  Trackio::getGps();

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

  // add battery
  char batt[25];
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
  bool gettingCommand = false;
  bool hasCommand = false;
  Trackio::emptyBuffer();

  if (SerialSim.available()) {
    while (SerialSim.available() > 0) {
      char x = SerialSim.read();

      if (35 == (int) x && !hasCommand) { // init command with #
        gettingCommand = true;
        __("START command:");
      } else if (36 == (int) x) { // end command with $
        gettingCommand = false;
        hasCommand = true;
        __("");
        __("END command:");
        Trackio::cmd[counter] = '\0';
      } else if (gettingCommand) {
        _(x);
        Trackio::cmd[counter] = x;
        counter++;
      }

      Trackio::_delay(1);
    }

    _("Command: "); __(Trackio::cmd);

    if (hasCommand) {
      return true;
    }
  }

  return false;
}

bool Trackio::processCommand (char * cmd) {
  Command command = Trackio::splitCommand(cmd);

  // damos por hecho que es válido (luego se verá)
  bool isValidCmd = true;

  if (strcmp(command.property, "IO6") == 0) {
    cfg.GPIO6 = (strcmp(command.value, "ON") == 0) ? HIGH : LOW;
    _(F("GUARDAMOS: ")); __(command.value);
    _(F("GUARDAMOS: ")); __(cfg.GPIO6);
    Trackio::saveConf();
    return Trackio::cmd_setIO(IO6, command.value);
  }
  else if (strcmp(command.property, "IO7") == 0) return Trackio::cmd_setIO(IO7, command.value);
  else if (strcmp(command.property, "MOSI") == 0) return Trackio::cmd_setIO(MOSI, command.value);
  else if (strcmp(command.property, "MISO") == 0) return Trackio::cmd_setIO(MISO, command.value);
  else if (strcmp(command.property, "CFG") == 0)  return Trackio::cmd_setConf(command.value);
  else {
    // finalmente no es válido...
    isValidCmd = false;
    _(F("UNKNOW COMMAND: ")); __(command.property);
  }

  return isValidCmd;
}

Command Trackio::splitCommand (char * cmd) {
  _(F("splitCommand: ")); _(cmd);
  Command command;

  char * split;

  split = strtok(cmd, "|");
  if (split != NULL) strcpy(command.property, split);

  split = strtok(NULL , "|");
  if (split != NULL) strcpy(command.value, split);

  _(F(" | prop: ")); _(command.property);
  _(F(" | value: ")); __(command.value);

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
  _(F("x")); _(cmd); __(F("x"));

  uint8_t len = strlen(cmd) - 1;
  int i;

  for (i = 1; i < len; i++) {

    if ((int) cmd[i] != 35 && (int) cmd[i] != 36) {
      _(F("Extract Command i: ")); __(cmd[i]);
      __cmd[i] = '\0';
      __cmd[i - 1] = cmd[i];
    }
  }

  _(F("Extract Command: ")); __(__cmd);
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
  _(F("setConf: ")); __(command);
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
  _(F("applyConf: ")); __(conf);
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
    __(F("GPRS SERVICE OK!!"));
    return true;
  }

  __(F("ERROR GPS SERVICE!!"));
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

  Trackio::sendCommand((char *) "AT+CLBS=1,1");

  Trackio::getBattery();
  Trackio::getSignalStrength();

  char cmd[11] = "AT+CGNSINF";
  Trackio::sendCommand(cmd);

  char * token = strtok(buffer, "\n");
  token = strtok(NULL, "\n");
  char * token2 = strtok(token, " ");
  token2 = strtok(NULL, " ");

  Trackio::_delay(1000);
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
  Trackio::_delay(1000);

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

void Trackio::_delay (int time) {
  Watchdog.reset();
  delay(time);
  Watchdog.reset();
}

// #############################################################################

void Trackio::blink (uint8_t times, uint8_t ms) {
  digitalWrite(LED, LOW);

  while (times--) {
    digitalWrite(LED, HIGH);
    Trackio::_delay(ms);
    digitalWrite(LED, LOW);
    Trackio::_delay(ms);
  }
}

void Trackio::blink () {
  Trackio::blink(3, 200);
}

void Trackio::emptyBuffer () {
  memset(buffer, 0, sizeof buffer);
}

// #############################################################################

void Trackio::sleepNow(uint8_t times) {
  __(F("------- SLEEP NOW -------"));
  while (times--) {
    _(F("REMAINING: ")); __(times);
    Watchdog.sleep(8000);
  }
  __(F("------- WAKEUP -------"));
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
  Trackio::emptyBuffer();

  if (__DEBUG) {
    __(F(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"));
    _(F("SerialSim > ")); __(cmd);
  }

  // enviamos el comando al modem
  SerialSim.println(cmd);
  SerialSim.flush();

  Trackio::_delay(time);

  // Esperamos respuesta comprobando el buffer cada 400 millis
  while (!SerialSim.available()) {
    Trackio::_delay(100);
    count++;
    if (count > 100) {
      __(F("sendSerialAt failed"));
      modemSerialsFails++;
      if (modemSerialsFails > 10) {
        __("Muchos fallos, reiniciando en 8 secs, o pulsa botón reset...");
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
    __(buffer);
    __(F("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<"));
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

// #############################################################################

bool Trackio::sendAt (char * cmd) {
  return sendAt(cmd, 2, NULL);
}

bool Trackio::sendAt (char * cmd, char * validate) {
  return sendAt(cmd, 2, NULL);
}

bool Trackio::sendAt (char * cmd, int returnLine) {
  return sendAt(cmd, returnLine, NULL);
}

bool Trackio::sendAt (char * cmd, int returnLine, char * validate) {
  int loopCounter = 0;
  int lineCount = 0;
  bool hasLine = false;
  char * serialBuffer;

  // empty buffer
  while (SerialSim.available()) SerialSim.read();
  Trackio::emptyBuffer();

  // debug
  __("");
  __(F(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"));
  _("sendAt > "); __(cmd);

  // send command to the modem
  SerialSim.println(cmd);
  SerialSim.flush(); // ensure command is sent

  while (loopCounter < 40) {
    // _(".");
    serialBuffer = readLine.feed(&SerialSim);
    if (serialBuffer != NULL) {
      lineCount++;
      _("  -> "); __(serialBuffer);

      if (lineCount == returnLine) {
        strcpy(buffer, serialBuffer);
        hasLine = true;
        // break;
      }
    } else if (!serialBuffer && loopCounter >= 40) {
      break;
    }

    loopCounter++;
    delay(50);
  }

  if (hasLine && validate != NULL) {
    _("validate: "); __(validate);
    if (strstr(buffer, validate)) return true;
    return false;
  }

  return hasLine;
}
