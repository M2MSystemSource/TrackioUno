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

int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
int accelCounter = 0;
int accelThreshold = 1000;
int triggerVal = 200;

struct Conf cfg;

FlashStorage(my_flash_store, Conf);

/**
 * @brief Almacena la lectura del Serial del modem. Se utiliza principalmente
 * en Trackio::sendCommand()
 */
char buffer[UART_BUFFER_SIZE];

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

  __(""); __("#############################################");
  _(F("           - Trackio ")); _(F(VERSION)); __(F(" START -"));
  __("#############################################"); __("");

  Trackio::blink();
  Trackio::loadConf();
  #if RH_accel == 1
  Trackio::setupAccel();
  Wire.begin();
  #endif

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

  mV_step_used = mV_step_3V3;
}

void Trackio::loadConf () {

  Conf owner;
  owner = my_flash_store.read();

  if (owner.eeprom != RH_eeprom) {
    __(F("Cargamos flash por primera vez"));
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
    cfg.accel = RH_accel;

    Trackio::saveConf();
  } else {
    __(F("La configuración ya existe."));
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

  // 1 -> pulso powerKey
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

  // 2
  if (!gsm_status) {
    // No se ha podido encender el modem. Revisar que GSM_PWREN, GSM_STATUS y
    // GSM_PWRKEY son los pines correctos.
    return false;
  }

  // 3
  if (!Trackio::powerOnGps()) {
    // se quiere usar el GPS pero el poweron ha fallado, revisar pin GPS_EN
    // y conexión del módulo esclavo
    return false;
  }

  // 4
  return true;
}

void Trackio::powerOff () {
  __(F("POWEROFF!"));
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
      ___(F("  == IMEI: "), Trackio::imei);
      return;
    }
  }

  ___(F("   == IMEI: "), F("FAIL"));
}

void Trackio::printIccid () {
  Trackio::sendAt((char *) "AT+CCID");
}

void Trackio::getBattery () {
  Trackio::vbat = 0;
  Trackio::vin = 0;
  Trackio::vsys_5v = 0;

  if (cfg.battMode == 1) {
    Trackio::getSimcomBattery();
  } else {
    Trackio::getAnalogBattery();
  }

  Trackio::checkLowBattery();
}

void Trackio::getSimcomBattery() {
  if (Trackio::sendAt((char *) "AT+CBC", 2)) {
    // buffer = +CBC: 0,21,3571
    char * split;
    split = strtok(buffer, ",");
    split = strtok(NULL, ","); // porcentage de batería
    split = strtok(NULL, ","); // batería

    Trackio::vbat = atoi(split);
  }
}

void Trackio::checkLowBattery () {
  char low = 0;

  if (cfg.requiredVbat >0 && Trackio::vbat < cfg.requiredVbat) low = 1;
  if (cfg.requiredVin > 0 && Trackio::vin < cfg.requiredVin) low = 1;
  if (cfg.requiredVsys5v > 0 && Trackio::vsys_5v < cfg.requiredVsys5v) low = 1;

  if (low == 1) {
    // reset serial fails, evita reinicio antes de deep sleep
    modemSerialsFails = 0;
    __(F("Entramos en LOW MODE"));
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

  __(F(""));
  __(F(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"));
  __(F("getAnalogBattery >"));

  digitalWrite(MUX_SW, LOW); delay(10);
  lectura_mV = (float) Trackio::readAnalogBatt(VBAT_PIN);
  Trackio::vbat = (float) (lectura_mV / VBAT_aux);
  _(F("  == VBAT: ")); __(Trackio::vbat);

  lectura_mV = (float) Trackio::readAnalogBatt(VSYS_PIN);
  Trackio::vsys_5v = (float) (lectura_mV / VSYS_aux);
  _(F("  == VSYS: ")); __(Trackio::vsys_5v);

  digitalWrite(MUX_SW, HIGH); delay(10);
  lectura_mV = (float) Trackio::readAnalogBatt(VIN_PIN);
  Trackio::vin = (float) (lectura_mV / VIN_aux);
  _(F("  == VIN: ")); __(Trackio::vin);
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
  Trackio::sendAt((char *) "AT+CPIN?", 1);
}

// #############################################################################
bool Trackio::checkStatus () {
  if (!Trackio::checkModem()) {
    __(F("  == FAIL checkModem"));
    return false;
  }

  Trackio::getSignalStrength();

  if (!Trackio::checkCreg()) {
    __(F("  == FAIL checkCreg"));
    return false;
  }

  return true;
}

void Trackio::getSignalStrength () {
  char x[10] = "AT+CSQ";

  if (Trackio::sendAt(x, 1)) {
    char * split;
    split = strtok(buffer, " ");
    split = strtok(NULL, ",");
    strcpy(Trackio::gsm, split);
    ___(F("  == signal: "), Trackio::gsm);
  }
}

bool Trackio::checkCreg () {
  if (!Trackio::atOk) return false;
  Trackio::cregOk = false;

  // Verificamos el creg 20 veces
  for (int i=0; i<20; i++) {
    if (!Trackio::sendAt((char *) "AT+CREG?", 1)) continue;

    if (strstr(buffer, ",5") || strstr(buffer, ",1")) {
      Trackio::cregOk = true;
      return true;
    }

    Trackio::_delay(1000);
  }

  return false;
}

void Trackio::sendSMS () {
  __(F("SEND SMS"));
  sendCommand((char *) "AT+CMGF=1");
  Trackio::_delay(1000);
  sendCommand((char *) "AT+CSCS=\"GSM\"");
  Trackio::_delay(1000);
  sendCommand((char *) "AT+CMGS=\"+34691612793\"");
  Trackio::_delay(1000);
  sendCommand((char *) "Mola Mazo!");
  Trackio::_delay(1000);
}

bool Trackio::checkModem () {
  Trackio::atOk = false;

  if (Trackio::sendAt((char *) "AT", 2, OK)) {
    Trackio::atOk = true;
    Trackio::sendAt((char *) "ATE0");
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

  // enviamos comando para abrir TCP
  char cmd[100];
  sprintf(cmd, "AT+CIPSTART=\"TCP\",\"%s\",\"%s\"", RH_SERVER, RH_PORT);
  if (!Trackio::sendAt(cmd, 2)) return false;
  ___(F("  == CIPSTART: "), buffer);
  Trackio::_delay(1000);

  // si ya está abierto Sim868 devolverá OK, sino READY CONNECT
  if (strstr(buffer, OK) || strstr(buffer, READY)) {
    openTcpFails = 0;
    __(F("  == TCP OK"));
    Trackio::tcpOk = true;

    // al iniciar el TCP reseteamos el contador de alives para empezar de nuevo
    Trackio::transmissionClockCounter = 0;

    return true;
  }

  __(F("  == ERROR TCP"));
  openTcpFails++;
  if (openTcpFails == 3) {
    __(F("El TCP ha fallado en multiples ocasiones - Hard Reset!"));
    Trackio::hardReset();
  }

  return false;
}

void Trackio::closeTcp (int mode) {
  char cmd[15];
  sprintf(cmd, "AT+CIPCLOSE=%i", mode);
  Trackio::sendAt(cmd, 1);
  Trackio::tcpOk = false;
  Trackio::listeningTcp = false;
  digitalWrite(LED, LOW);
}

bool Trackio::tcpIsOpen () {
  return Trackio::sendAt((char *) "AT+CIPSTATUS", 2, CONNECT_OK);
}

bool Trackio::sayHello () {
  char hello[50];

  // debemos transmitir al servidor el modo operacional, OP_TCP/AUTO
  int opmode = 1; // por defecto TCP
  if (cfg.primaryOpMode == OP_AUTO) opmode = 2; // si no AUTO

  // x debería ser prescindible, pero si inserto Trackio::imei
  // directamente en el sprintf se produce un segmentation fault error
  // que mis altas capacidades en el lenguaje C no logran entender
  char imei[20];
  strcpy(imei, Trackio::imei);
  if (opmode == 1) {
    sprintf(hello, "%s|%s", imei, VERSION);
  } else {
    sprintf(hello, "%s|%d", imei, opmode);
  }

  if (Trackio::tcpOk && Trackio::transmit(hello)) {
    ___(F("buffer: "), buffer);
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
  if (strstr(buffer, ERROR)) {
    __(F("  == CIPSEND FAIL - abort"));
    return false;
  }

  Trackio::sendCommand(msg);

  // verificamos respuesta
  if (strstr(buffer, "SEND OK")) {
    // Comprobamos si el servidor nos ha devuelto una respuesta
    char * response = getTransmitResponse(buffer);
    ___(F("  == response: "), response);
    if (isCommand(response)) {
      char * cmd = Trackio::extractCommand(response);
      if (Trackio::processCommand(cmd)) {
        ___(F("  == Command PROCESSED: "), cmd);
      }
    } else {
      __(F("  == Response is not a commnand"));
    }

    if (cfg.opmode == OP_TCP) {
      __(F(""));
      __(F("Listening for TCP commands..."));
      Trackio::listeningTcp = true;
    }

    return true;
  }

  Trackio::listeningTcp = false;
  Trackio::tcpOk = false;
  __(F("  == ERROR SEND"));
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
        __(F("START command:"));
      } else if (36 == (int) x) { // end command with $
        gettingCommand = false;
        hasCommand = true;
        __(F(""));
        __(F("END command: "));
        Trackio::cmd[counter] = '\0';
      } else if (gettingCommand) {
        _(x);
        Trackio::cmd[counter] = x;
        counter++;
      }

      Trackio::_delay(1);
    }

    if (hasCommand) {
      ___(F("Command: "), Trackio::cmd);
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
    ___(F("GUARDAMOS: "), command.value);
    ___(F("GUARDAMOS: "), cfg.GPIO6);
    Trackio::saveConf();
    return Trackio::cmd_setIO(IO6, command.value);
  }
  else if (strcmp(command.property, "IO0") == 0) return Trackio::cmd_setIO(IO0, command.value);
  else if (strcmp(command.property, "IO1") == 0) return Trackio::cmd_setIO(IO1, command.value);
  else if (strcmp(command.property, "IO2") == 0) return Trackio::cmd_setIO(IO2, command.value);
  else if (strcmp(command.property, "IO3") == 0) return Trackio::cmd_setIO(IO3, command.value);
  else if (strcmp(command.property, "IO4") == 0) return Trackio::cmd_setIO(IO4, command.value);
  // else if (strcmp(command.property, "IO5") == 0) return Trackio::cmd_setIO(IO5, command.value); // no disponible en Arduino Zero
  else if (strcmp(command.property, "IO6") == 0) return Trackio::cmd_setIO(IO6, command.value);
  else if (strcmp(command.property, "IO7") == 0) return Trackio::cmd_setIO(IO7, command.value);
  else if (strcmp(command.property, "SDA") == 0) return Trackio::cmd_setIO(SDA, command.value);
  else if (strcmp(command.property, "SCL") == 0) return Trackio::cmd_setIO(SCL, command.value);
  else if (strcmp(command.property, "SCK") == 0) return Trackio::cmd_setIO(SCK, command.value);
  else if (strcmp(command.property, "CS0") == 0) return Trackio::cmd_setIO(CS0, command.value);
  else if (strcmp(command.property, "NMI") == 0) return Trackio::cmd_setIO(NMI, command.value);
  else if (strcmp(command.property, "INT0") == 0) return Trackio::cmd_setIO(INT0, command.value);
  else if (strcmp(command.property, "INT1") == 0) return Trackio::cmd_setIO(INT1, command.value);
  else if (strcmp(command.property, "MOSI") == 0) return Trackio::cmd_setIO(MOSI, command.value);
  else if (strcmp(command.property, "MISO") == 0) return Trackio::cmd_setIO(MISO, command.value);
  else if (strcmp(command.property, "MUX_SW") == 0) return Trackio::cmd_setIO(MUX_SW, command.value);
  else if (strcmp(command.property, "CFG") == 0)  return Trackio::cmd_setConf(command.value);
  else if (strcmp(command.property, "SAVE") == 0)  {Trackio::saveConf(); return true;}
  else if (strcmp(command.property, "RST") == 0)  Trackio::hardReset();
  else {
    // finalmente no es válido...
    isValidCmd = false;
    ___(F("UNKNOW COMMAND: "), command.property);
  }

  return isValidCmd;
}

Command Trackio::splitCommand (char * cmd) {
  ___(F("splitCommand: "), cmd);
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
      ___(F("Extract Command i: "), cmd[i]);
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
  ___(F("setConf: "), command);
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

  return true;
}

bool Trackio::applyConf (char * conf) {
  ___(F("applyConf: "), conf);
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

  // > = 0.3.0
  if      (strcmp(prop, "dslp") == 0) cfg.deepSleep = atoi(value);
  else if (strcmp(prop, "gpsi") == 0) cfg.gpsInterval = atoi(value);
  else if (strcmp(prop, "op") == 0)   cfg.opmode = atoi(value);
  else if (strcmp(prop, "pop") == 0)  cfg.primaryOpMode = atoi(value);
  else if (strcmp(prop, "slp") == 0)  cfg.sleep = atoi(value);
  // >= 0.3.2
  else if (strcmp(prop, "tclk") == 0)  cfg.transmissionClock = atoi(value);
  else if (strcmp(prop, "vbat") == 0)  cfg.requiredVbat = atoi(value);
  else if (strcmp(prop, "vin") == 0)  cfg.requiredVin = atoi(value);
  else if (strcmp(prop, "vsys") == 0)  cfg.requiredVsys5v = atoi(value);

  return true;
}

// #############################################################################

bool Trackio::enableGprs () {
  // Obtener el estado del servicio GPRS
  if (Trackio::gprsIsOpen()) {
    // GPRS ya está abierto
    return false;
  }

  char apnCmd[150];
  sprintf(
    apnCmd,
    "AT+CSTT=\"%s\",\"%s\",\"%s\"",
    RH_APN, RH_APN_USER, RH_APN_PASS
  );

  if (!Trackio::sendAt(apnCmd, 1, OK)) return false;
  Trackio::_delay(1000);

  if (!Trackio::sendAt((char *) "AT+CIICR", 1, OK)) return false;
  Trackio::_delay(1000);

  Trackio::sendAt((char *) "AT+CIFSR", 1, OK);
  Trackio::_delay(1000);

  return true;
}

bool Trackio::gprsIsOpen () {
  if (!Trackio::cregOk) return false;

  if (Trackio::sendAt((char *) "AT+CGATT?", 1, (char *) "+CGATT: 1")) {
    __(F("  == GPRS OK"));
    return true;
  }

  __(F("  == GPRS CLOSED"));
  return false;
}

// #############################################################################

bool Trackio::powerOnGps () {
  if (!Trackio::sendAt((char *) "AT+CGNSPWR=1", 2, OK)) {
    return false;
  }
  Trackio::_delay(1500);

  // 2
  digitalWrite(GPS_EN, HIGH);
  Trackio::_delay(1000);

  // 3
  Trackio::sendAt((char *) "AT+CGPSRST=0", 1);

  return true;
}

void Trackio::getGps () {
  if (cfg.gpsInterval < 1) return;

  Trackio::getBattery();
  Trackio::getSignalStrength();

  if (Trackio::sendAt((char *) "AT+CGNSINF", 1)) {
    char * split = strtok(buffer, " ");
    split = strtok(NULL, " ");
    Trackio::parseGps(split);
  }
}

void Trackio::resetGpsData () {
  Trackio::gps.fix = 0;
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

void Trackio::hardReset () {
  Trackio::sendCommand((char *) "AT+CFUN=0");
  delay(1000);
  Trackio::powerOff();
  while (1) {}
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

void Trackio::setupAccel () {
  Wire.begin();
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
}

void Trackio::readAccel () {
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDRESS, 14, true);

  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  Tmp = Wire.read() << 8 | Wire.read();
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();

  /*
  Serial.print("AcX = "); Serial.print(AcX);
  Serial.print(" | AcY = "); Serial.print(AcY);
  Serial.print(" | AcZ = "); Serial.print(AcZ);
  Serial.print(" | Tmp = "); Serial.print(Tmp / 340.00+36.53);  //equation for temperature in degrees C from datasheet
  */

  // ___(" | X: ", GyX); ___(" | Y: ", GyY); ___(" | Z: ", GyZ);
}

void Trackio::checkAccel () {
  if (digitalRead(IO6)) return;
  Trackio::readAccel();

  if (GyX > accelThreshold || GyX < -accelThreshold || GyY > accelThreshold || GyY < -accelThreshold || GyZ > accelThreshold || GyZ < -accelThreshold) {
    accelCounter++;
  } else {
    if (accelCounter > 0) accelCounter -= 10;
    if (accelCounter <= 0) {
      digitalWrite(MISO, LOW);
    }
  }

  if (accelCounter >= triggerVal) {
    Trackio::enableMovementAlert();
  }

  if (accelCounter > triggerVal) accelCounter = triggerVal;
}

// #############################################################################

void Trackio::enableMovementAlert () {
  if (digitalRead(IO6)) return;
  __("ALERT!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  digitalWrite(MISO, HIGH);
}

void Trackio::disableMovementAlert () {
  __("STOP!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  digitalWrite(MISO, LOW);
}

// #############################################################################

bool Trackio::sendCommand (char * cmd) {
  int count = 0;
  int indexPosition = 0;

  while (SerialSim.available()) SerialSim.read();

  // vaciamos el buffer previo
  Trackio::emptyBuffer();

  __(F(""));
  __(F(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"));
  _(F("SerialSim > ")); __(cmd);

  // enviamos el comando al modem
  SerialSim.println(cmd);
  SerialSim.flush();

  Trackio::_delay(50);

  // Esperamos respuesta comprobando el buffer cada 400 millis
  while (!SerialSim.available()) {
    Trackio::_delay(100);
    count++;
    if (count > 100) {
      __(F("sendSerialAt failed"));
      modemSerialsFails++;
      if (modemSerialsFails > 10) {
        __(F("Muchos fallos, reiniciando en 8 secs, o pulsa botón reset..."));
        delay(10000);
      }
      return false;
    }
  }

  modemSerialsFails = 0; // reset contador

  // acomodamos la respuesta en buffer
  __("START WHILE SENDCOMMAND");
  while(SerialSim.available() > 0 && indexPosition < UART_BUFFER_SIZE) {
    buffer[indexPosition] = SerialSim.read();
    indexPosition++;
    Trackio::_delay(1);
  }
  __("END WHILE SENDCOMMAND");

  buffer[indexPosition] = '\0';
  __(buffer);
  return true;
}

// #############################################################################

bool Trackio::sendAt (char * cmd) {
  return sendAt(cmd, 2, NULL, 0);
}

bool Trackio::sendAt (char * cmd, char * validate) {
  return sendAt(cmd, 2, NULL, 0);
}

bool Trackio::sendAt (char * cmd, int returnLine) {
  return sendAt(cmd, returnLine, NULL, 0);
}

bool Trackio::sendAt (char * cmd, int returnLine, char * validate) {
  return sendAt(cmd, returnLine, validate, 0);
}

bool Trackio::sendAt (char * cmd, int returnLine, int timeout) {
  return sendAt(cmd, returnLine, NULL, timeout);
}

bool Trackio::sendAt (char * cmd, int returnLine, char * validate, int timeout) {
  int loopCounter = 0;
  int lineCount = 0;
  bool hasLine = false;
  char * serialBuffer;

  // empty buffer
  while (SerialSim.available()) SerialSim.read();
  Trackio::emptyBuffer();

  // debug
  __(F(""));
  __(F(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"));
  _(F("sendAt > ")); __(cmd);

  // send command to the modem
  SerialSim.println(cmd);
  SerialSim.flush(); // ensure command is sent

  // algunos comandos requieren cierto tiempo antes de ser ejecutados
  if (timeout) Trackio::_delay(timeout);

  while (loopCounter < 40) {
    serialBuffer = readLine.feed(&SerialSim);
    if (serialBuffer != NULL) {
      lineCount++;

      if (lineCount == returnLine) {
        strcpy(buffer, serialBuffer);
        hasLine = true;
        ___(F("  -> "), serialBuffer); // -> es línea
        break;
      }

      ___(F("  -- "), serialBuffer); // -- no es linea
    }

    loopCounter++;
    Trackio::_delay(100);
  }

  if (!hasLine) {
    __(F("Serial Failed"));
    modemSerialsFails++;
    if (modemSerialsFails > 10) {
      Trackio::hardReset();
    }

    return false;
  } else {
    modemSerialsFails = 0;
    if (validate != NULL) {
      if (!strstr(buffer, validate)) return false;
    }
  }

  return true;
}
