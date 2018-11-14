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
#include <avr/wdt.h>
#include "Trackio.h"
#include "TimerOne.h"

#if defined(__AVR_ATmega328P__)
  #include <SoftwareSerial.h>
  SoftwareSerial SoftSerial(RHIO_SOFT_RX, RHIO_SOFT_TX);
#endif

Trackio trackio;

// Determina si se ha enviado la primera posición, esto permite a getGps()
// no tener que esperar al timer para el envío de la primera posición
byte firstPositionHasBeenSent = 0;

// modos operacionales
void op_startup();
void op_tcp();
void op_auto();
void op_low();

// funciones para su uso en los modos operacionales OP_TCP y OP_AUTO
bool getGps(bool manageTcp);
void checkCommand();
void transmitAlive();

// Método para llamar al watchdog externo.
// Este watchdog será complementario al watchdog interno del 328p para ofrecer
// máxima seguridad ante posibles cuelgues del oscilador que incorpora el 328p.
// (si este oscilador se cuelga (hang) se ven afectados tanto el micro como el
// watchdog interno).
// Se inicializa en el setup y se asigna a un timer de la librería TimerOne.h
// que see ejecuta cada segundo.
void externalWatchdogInterrupt();

/**
 * @brief La inicializaicón de la aplicación se
 * realiza en el modo OP_STARTUP. Echa un ojo a op_startup()
 */
void setup() {
  // disable watchdog
  // en los docs de microchip podemos encontrar un método similar:
  // https://www.microchip.com/webdoc/AVRLibcReferenceManual/FAQ_1faq_softreset.html
  MCUSR = 0;
  wdt_disable();

  // Configure sleep
  trackio.configureSleep();

  Timer1.initialize(1000000);
  Timer1.attachInterrupt(externalWatchdogInterrupt);
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
  trackio.perroGuardian = 0;

  if      (cfg.opmode == OP_TCP) op_tcp();
  else if (cfg.opmode == OP_AUTO) op_auto();
  else if (cfg.opmode == OP_STARTUP) op_startup();
  else if (cfg.opmode == OP_LOW) op_low();
  else if (cfg.opmode == OP_RST) trackio.hardReset();

  trackio._delay(1);
}

/**
 * @brief https://www.arduino.cc/en/Reference/SerialEvent
 * Lo utilizamos principalmente en el modo OP_TCP para recibir los comandos
 * operacionales del server. Cuando el servidor escribe un comando (por TCP)
 * el simcom envía esos datos al Serial, esto hace saltar automáticamente este
 * método que a su vez llama a checkCommand() para verificar si los datos son
 * válidos
 */
void serialEvent() {
  checkCommand();
}

// #############################################################################

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
  digitalWrite(LED01, LOW);

  if (!trackio.begin()) {
    __(F("Trackio Critical FAIL - SIM868 can't start"));
    __(F("Try to powerof and check connection of master/slave modules"));
    trackio.hardReset();
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
    trackio.sleepNow(32);
  }
}

// #############################################################################

bool getGps (bool manageTcp) {
  // o es la primera posición (tras un reset) o esperamos a que se cumpla el ciclo de reloj
  // en trackio.timers.transmissionClock. El método transmitAlive() (en main.c) se
  // encarga de aumentar este contador (trackio.transmissionClockCounter).
  // Si venimos de un reset y esta es la primera posición que se intenta enviar,
  // le damos salida para no tener que esperar a que se cumpla el reloj
  if (trackio.transmissionClockCounter < cfg.gpsInterval && firstPositionHasBeenSent) {
    // todavía no es momento de enviar el GPS
    return false;
  }

  trackio.transmissionClockCounter = 0;

  if (manageTcp && !trackio.tcpIsOpen()) {
    // aseguramos que el TCP está abierto
    trackio.openTcp();
  }

  if (!trackio.transmitGps()) {
    __(F("FAIL GPS TRANSMIT!!"));
    cfg.opmode = OP_STARTUP;
    return false;
  }

  firstPositionHasBeenSent = true;
  return true;
}

void transmitAlive () {
  unsigned long clockTimerDiff = (trackio.timers.base - trackio.timers.transmissionClock);

  if (!firstPositionHasBeenSent) {
    // hasta que no se envía la primera posición no emitimos ningún alive.
    return;
  }

  if (clockTimerDiff >= (cfg.transmissionClock * 1000L)) {
    trackio.timers.transmissionClock = trackio.timers.base;

    trackio.transmissionClockCounter++;
    if (trackio.transmissionClockCounter >= cfg.gpsInterval) {
      // si es la hora de enviar un GPS no queremos enviar también el alive
      return;
    }

    if ((int) cfg.opmode == (int) OP_TCP) {
      trackio.transmit((char *) "%");
    }
  }
}

void checkCommand () {
  char ack[20];
  if (trackio.tcpHasCommand()) {
    if (trackio.processCommand(trackio.cmd)) {
      sprintf(ack, "ack|%i", digitalRead(IO06));
      trackio.transmit(ack);
    }
  }
}

void externalWatchdogInterrupt() {
  // perroGuardian se resetea a cero en el loop. Si su valor supera 120secs
  // supondría que desde hace más de 2 minutos no se accede al loop por lo que
  // el programa podría estar en un hang (no hay razón para estar tanto tiempo
  // fuera del loop)

  trackio.perroGuardian++; // aumentamos cada seg (tiempo definido en setup())
  // _(";"); _(trackio.perroGuardian);
  if (trackio.perroGuardian < 120) {
    trackio.clkPulse();
  } else {
    // _("#");
  }
}
