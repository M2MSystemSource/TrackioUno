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

// funciones para su uso en los modos operacionales OP_TCP
bool getGps();
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
    trackio.hardReset();
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
  // reset data...
  digitalWrite(LED, LOW);
  firstPositionHasBeenSent = false;

  if (!trackio.begin()) {
    SerialMon.println(F("Trackio Critical FAIL - SIM868 can't start"));
    SerialMon.println(F("Try to powerof and check connection of master/slave modules"));
    trackio.hardReset();
    return;
  }

  if (cfg.opmode == OP_LOW) {
    return;
  }

  //
  if (!trackio.checkStatus()) {
    // si algo falla volvemos a empezar...
    return;
  }
  trackio.openGprs();

  if (cfg.primaryOpMode == OP_TCP) {
    // si es modo TCP abrimos ya el puerto y hacemos el registro en servidor
    trackio.openTcp();
    if (trackio.tcpOk) {
      trackio.sayHello();
      cfg.opmode = OP_TCP;
    }
  } else {
    cfg.opmode = cfg.primaryOpMode;
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
  #if RH_accel == true
  trackio.checkAccel();
  #endif
  checkCommand();
  transmitAlive();
  getGps(); // obtiene GPS y envía posición a servidor
}

// #############################################################################

void op_auto () {
  // comprobamos si tickTimer se ha cumplido
  unsigned long clockTimerDiff = (trackio.timers.base - trackio.timers.tickTimer);
  if (clockTimerDiff >= (cfg.tickTimer * 1000L)) {
    // reset tickTimer
    trackio.timers.tickTimer = trackio.timers.base;

    _title(F("TRANSMIT MESSAGE"));

    // Creamos el mensaje que vamos a enviar
    trackio.createMessage();
    // con cfg.transmitAlways nos permite saber si el mensaje se ha enviado
    // en caso negativo lo podremos guardar en log y enviar más tarde
    bool transmitted = false;

    if (cfg.transmitAlways) {
      // realizamos la transmisión. en este punto es posible que la situación del
      // dispositivo no sea correcta, es decir, puede que nos hayan movido de sitio
      // y ahora mismo no tengamos cobertura GSM para el envío de datos.
      // prepareForTransmission se encargará de hacer las comprobaciones
      // y abrir TCP
      if (trackio.prepareForTransmission()) {
        delay(1000);
        if (trackio.sayHello()) {
          delay(1000);
          if (trackio.transmit(trackio.message)) {
            transmitted = true;
          }
        }
      } // else: el dispositivo no se encuentra en disposición de enviar datos

      trackio.closeTcp(1);
    }

    if (!transmitted || !cfg.transmitAlways) {
      __("SAVE MESSAGE");
      trackio.saveMessage();
    }

    if (!trackio.transmitLogIfFull()) {
      __("  == ERROR TRANSMITING LOG");
    }

    if (cfg.sleep) {
      delay(1000);
      if (cfg.deepSleep) {
        trackio.powerOff();
      }

      // el método sleep se va a dormir en ciclos de 8 segundos.
      // debemos deducir cuantos ciclos de 8 segundos tenemos en cfg.tickTimer.
      int sleepTimes = (int) cfg.tickTimer / 8;
      trackio.sleepNow(sleepTimes);

      if (cfg.deepSleep) {
        // si venimos de deepsleep tratamos de despertar el sim868
        if (!trackio.powerOn()) {
          // si falla realizamos la rutina de arranque desde el principio
          cfg.opmode = OP_STARTUP;
        } else {
          trackio.checkModem();
          if (cfg.useGps) {
            trackio.powerOnGps();
          }
        }
      }
    }
  }

  _(F("."));
}

// #############################################################################

void op_low () {
  trackio.getBattery();
  if (cfg.opmode == OP_LOW) {
    trackio.sleepNow(4);
  }
}

// #############################################################################

bool getGps () {
  // o es la primera posición (tras un reset) o esperamos a que se cumpla el ciclo de reloj
  // en trackio.timers.transmissionClock. El método transmitAlive() (en main.c) se
  // encarga de aumentar este contador (trackio.transmissionClockCounter).
  // Si venimos de un reset y esta es la primera posición que se intenta enviar,
  // le damos salida para no tener que esperar a que se cumpla el reloj
  if (cfg.gpsInterval == 0) {
    return false;
  }

  // Para que la transmission de gps se pueda llevar a cabo, transmissionClockCounter
  // debe ser >= cfg.gpsInterval
  if (trackio.transmissionClockCounter < cfg.gpsInterval) {
    // todavía no es momento de enviar el GPS
    return false;
  }

  trackio.listeningTcp = false;
  trackio.transmissionClockCounter = 0;

  if (!trackio.transmitGps()) {
    SerialMon.println(F("  == GPS TRANSMISSION FAIL"));
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
      char io6Status[1];
      sprintf(io6Status, "%i", digitalRead(IO6));
      if (!trackio.transmit(io6Status)) {
        // fallo al enviar el alive, intentamos recuperar la conexión o
        // reiniciamos el sistema
        __(F("  == FAIL ALIVE"));
        if (!trackio.tcpIsOpen()) {
          // hemos perdido la conexión.
          __(F("  => El TCP está cerrado"));
          if (trackio.openTcp()) {
            // hemos abierto el TCP, seguimos
            __(F("  => EL TCP se ha abierto"));
            // nos registramos para poder operar
            trackio.sayHello();
          } else {
            __(F("  => NO se ha podido abrir TCP, reiniciamos"));
            cfg.opmode = OP_STARTUP;
          }
        }
      }
    }
  }
}

void checkCommand () {
  if (!trackio.listeningTcp) return;
  if (trackio.tcpHasCommand()) {
    if (trackio.processCommand(trackio.cmd)) {
      char ack[20];
      sprintf(ack, "ack|%i", digitalRead(IO6));
      trackio.transmit(ack);
    }
  }
}
