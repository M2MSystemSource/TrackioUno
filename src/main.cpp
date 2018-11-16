/** @file */

/*
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
 */

/**
 * @brief TrackioUno - Realtime GPS tracker
 *
 * **TrackioUno** es un dispositivo desarrollado con el hardware de
 * [Rhomb.io](https://rhomb.io) y la infraestructura de
 * [M2M System Source](https://m2msystemsource.com) (m2mss).
 *
 * Por favor, lee el archivo README.md para obtener más detalles acerca del
 * funcionamiento de este software.
 *
 * @file main.cpp
 * @author Jordi Enguídanos <jenguidanos@rhomb.io>
 * @copyright (C) 2018 Tecnofingers S.L
 */

#include <Arduino.h>
#include <Adafruit_SleepyDog.h>
#include "Trackio.h"

Trackio trackio;

// Determina si se ha enviado la primera posición, esto permite a getGps()
// no tener que esperar al timer para el envío de la primera posición
byte firstPositionHasBeenSent = 0;

void op_startup();
void op_tcp();
void op_auto();
void op_low();

// funciones para su uso en los modos operacionales OP_TCP y OP_AUTO
bool getGps(bool manageTcp);
void checkCommand();
void transmitAlive();

// #############################################################################

/**
 * @brief Configuramos el Watchdog. Nada más... el resto se hace en el loop.
 */
void setup() {
  Watchdog.enable(8000);
}

/**
 * @brief Ejecuta las tareas según el modo operacional en "trackio.cfg.opmode"
 *
 * Durante todo el código se evita bloquear el loop por lo que constantemente
 * se está ejecutando el método op_tcp(). También se define el reloj
 * "trackio.timers.base = millis()" que es utilizado por el GPS para obtener
 * las posiciones.
 */
void loop() {
  trackio.timers.base = millis();

  if      (cfg.opmode == OP_TCP) op_tcp();
  else if (cfg.opmode == OP_AUTO) op_auto();
  else if (cfg.opmode == OP_STARTUP) op_startup();
  else if (cfg.opmode == OP_LOW) op_low();
  else if (cfg.opmode == OP_RST) {
    SerialMon.println("Reset Micro - Firing watchdog");
    while (1) {};
  }

  trackio._delay(1);
}

/**
 * @brief Ejecución del modo OP_STARTUP
 *
 * El modo operacional OP_STARTUP se establece por defecto cuando iniciamos la
 * aplicación. Comprueba el estado del modem (envía comandos AT y ATE0),
 * registro en la red y habilita el GPRS. Si todo es OK cambiará al modo
 * operacional definido en PRIMARY_OP_MODE.
 *
 * Si durante el transcurso de la aplicación se producen errores (el dispositivo
 * pierde cobertura GPRS/GPS, por ejem) se establecerá nuevamente este modo
 * para realizar nuevamente la inicialización de la aplicación
 */
void op_startup () {
  digitalWrite(LED, LOW);

  if (!trackio.begin()) {
    SerialMon.println(F("Trackio Critical FAIL - SIM868 can't start"));
    SerialMon.println(F("Try to powerof and check connection of master/slave modules"));
    trackio._delay(5000);
    return;
  }

  if (cfg.opmode == OP_LOW) {
    return;
  }

  // el imei es fundamental para construir las tramas que se envían al servidor
  // aquí lo obtenemos y se guarda en Trackio::imei
  trackio.getImei();

  // para OP_TCP y OP_AUTO iniciamos el registro de red gsm/gprs y puerto TCP
  if (cfg.primaryOpMode == OP_TCP || cfg.primaryOpMode == OP_AUTO) {
    trackio.checkStatus();
    trackio.enableGprs();
    trackio.openTcp();
    trackio.sayHello();
  }

  if (cfg.primaryOpMode == OP_TCP) {
    if (trackio.tcpOk) {
      cfg.opmode = OP_TCP;
    }
  }

  if (cfg.primaryOpMode == OP_AUTO) {
    if (trackio.tcpOk) {
      cfg.opmode = OP_AUTO;
      // cerramos el TCP que se a abierto antes (unas líneas más arriba)
      trackio.closeTcp(1);
    }
  }
}

// #############################################################################

/**
 * @brief Ejecución del modo OP_TCP
 *
 * Desde `trackio.sayHello()` se establece este modo si el dispositivo consigue
 * registrarse en el servidor. El modo OP_TCP da por hecho que la conexión TCP
 * está permanentemente abierta. Este método se ejecuta cada vez que entramos al
 * loop y utiliza el método trackio.tcpHasCommand() para comprobar si ha entrado
 * algún comando desde el servidor. Aquí también realizamos el envío del ACK o
 * KO en función de si el comando se ejecuta correctamente o no. Por último
 * si `cfg.gpsInterval > 0` enviará posición GPS utilizando el timer
 * Trackio::timers.gps()
 */
void op_tcp () {
  checkCommand();
  transmitAlive();
  getGps(false); // obtiene GPS y envía posición a servidor
}

// #############################################################################

void op_auto () {
  getGps(true);
  if (trackio.tcpOk) trackio.closeTcp(1);
}

// #############################################################################

void op_low () {
  trackio.getBattery();
  if (cfg.opmode == OP_LOW) {
    trackio.sleepNow(4);
  }
}

// #############################################################################

bool getGps (bool manageTcp) {
  // o es la primera posición (tras un reset) o esperamos a que se cumpla el ciclo de reloj
  // en trackio.timers.transmissionClock. El método transmitAlive() (en main.c) se
  // encarga de aumentar este contador (trackio.transmissionClockCounter).
  // Si venimos de un reset y esta es la primera posición que se intenta enviar,
  // le damos salida para no tener que esperar a que se cumpla el reloj
  if (trackio.transmissionClockCounter < cfg.gpsInterval) {
    // todavía no es momento de enviar el GPS
    return false;
  }

  trackio.listeningTcp = false;
  trackio.transmissionClockCounter = 0;

  if (manageTcp && !trackio.tcpIsOpen()) {
    // aseguramos que el TCP está abierto
    trackio.openTcp();
  }

  if (!trackio.transmitGps()) {
    SerialMon.println(F("FAIL GPS TRANSMIT!!"));
    cfg.opmode = OP_STARTUP;
    return false;
  }

  firstPositionHasBeenSent = 1;
  return true;
}

void transmitAlive () {
  unsigned long clockTimerDiff = (trackio.timers.base - trackio.timers.transmissionClock);
  if (clockTimerDiff >= (cfg.transmissionClock * 1000L)) {
    trackio.timers.transmissionClock = trackio.timers.base;

    if (firstPositionHasBeenSent == 0) {
      trackio.transmissionClockCounter = (cfg.gpsInterval + 1);
      return;
    }

    trackio.transmissionClockCounter++;
    if (trackio.transmissionClockCounter >= cfg.gpsInterval) {
      // si es la hora de enviar un GPS no queremos enviar también el alive
      return;
    }

    trackio.listeningTcp = false;
    if ((int) cfg.opmode == (int) OP_TCP) {
      trackio.transmit((char *) "%");
    }
  }
}

void checkCommand () {
  if (!trackio.listeningTcp) return;

  char ack[20];
  if (trackio.tcpHasCommand()) {
    if (trackio.processCommand(trackio.cmd)) {
      sprintf(ack, "ack|%i", digitalRead(IO6));
      trackio.transmit(ack);
    }
  }
}
