/** @file */

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
 * @file Trackio.h
 * @author Jordi Enguídanos <jenguidanos@m2msystemsource.com>
 */

#ifndef TRACKIO
#define TRACKIO
#define VERSION "0.2.10"

#include <Arduino.h>
#include "static-conf.h"
#include "rhio-pins.h"

/**
 * @brief Indica si se mostrarán mensajes de log (solo afecta al método
 * Trackio::sendComman())
 */
#define __(x) if (RH_DEBUG) {SerialMon.println(x);}
#define _(x) if (RH_DEBUG) {SerialMon.print(x);}

// OFFSETS para el cálculo en las lecturas analógicas de baterías
const float mV_step_used = 0.00322265625;
/*const float aux_bat = 0.31972789115646258503401360544218;
const float aux_ext_bat = 0.04489016236867239732569245463228;*/
const float VBAT_aux=0.3197278911564626;
const float VIN_aux=0.0231513138614829;
const float VSYS_aux=0.3197278911564626;

/**
 * @brief Modo operacional por defecto al arrancar el dispositivo
 */
#define OP_STARTUP 0

/**
 * @brief Modo operacional conducido por comandos TCP
 *
 * Al habilitar este modo se establecerá una conexión TCP permanente con el
 * servidor, esto permite al servidor enviar comandos desde un cliente web o
 * similar. Este modo también puede habilitar el GPS para enviar posiciones
 * cada X segundos mientras se mantiene abierto el TCP y se recepcionan comandos.
 *
 * El consumo de batería será alto, no hay posibilidad de entrar en modo de
 * bajo consumo (sleep), el socket TCP permanece siempre abierto.
 */
#define OP_TCP 2
/**
 * @brief Modo operacional automático, no mantiene el socket abierto.
 */
#define OP_AUTO 1

/**
 * @brief Fuerza un RESET del sistema utilizando el watchdog mediante un loop
 * infinito
 */
#define OP_RST 4

/**
 * @brief Modo operacional de bajo consumo.
 *
 * Apagará el MCU y el modem, el dispositivo dejará de funcionar hasta que los
 * valores de batería determinados en cfg.requiredVbat/Vsys/Vin se repongan.
 */
#define OP_LOW 5

/**
 * @brief struct que almacena un parámetro de configuración, propiedad:valor
 *
 * Cuando se recibe un nuevo comando en Trackio::tcpHasCommand() esté se pasa al
 * método Trackio::splitCommand(), donde se parsea el comando y se separa en
 * propiedad:valor. Trackio::splitCommand() devolverá un objeto de este struct
 */
struct Command {
  char property[20];
  char value[61];
};

/**
 * @brief Configuracion de la aplicación
 *
 * Un struct con parámetros de configuración para almacenar en memoria
 * EEPROM. Se defienen unos parámetros por defecto, pero se pueden cambiar
 * y guardar en memoria EEPROM, tras un reinicio se leeará la memoria.
 */
struct Conf {

  /**
   * @brief establece el modo en el que se leerán las baterías.
   *
   * 1 = Utiliza la lectura de bateria del Simcom, se interpretará como vbat y
   *     vin/vsys quedaran a cero
   * 2 = Lectura de batería desde puertos analógicos definidos en VBAT_PIN,
   *     VIN_PIN y VSYS_PIN
   * 3 = Lectura de VBAT, VIN y VSYS utilizando el ADC TLA2024 de Halley BOX
   *
   * El método 3 requiere cargar la librería del TLA2024. Esto se realiza en
   * Trackio.cpp
   */
  char battMode;

  /**
   * @brief Establece si el dispositivo debe entrar en modo deepsleep
   *
   * Por defecto se utiliza un sleep normal donde el micro se pone en modo
   * IDLE y al modem se le envía comando para entrar en modo de bajo consumo.
   * Si establecemos esto a true el micro entrará en modo PWR_DOWN y al
   * modem se le cortará la corriente, por lo que obtendremos un consumo
   * mínimo entre 100 y 250 µV.
   *
   * Ten en cuenta que al despertarse el modem necesitará realizar de nuevo
   * todo el proceso de startup además se perderá el FIX GPS, por lo que ha
   * efectos supone un hard reset total.
   */
  bool deepSleep;

  /**
   * @brief Variable de control para especificar si se carga esta configuración
   * desde la eeprom o se sobreescribe con los valores de Trackio::loadConf()
   * Un valor diferente a 1 śobre escribirá los datos de eeprom al inicio,
   * un valor igual a 1 leerá
   */
  uint8_t eeprom;

  /**
   * @brief Intérvalo en segundos para obtener posiciones. 0 para desactivar.
   *
   * Valido para modos operacionales OP_TCP y OP_AUTO
   */
  unsigned int gpsInterval;

  /**
   * @brief Indica el modo operacional, por defecto OP_STARTUP.
   *
   * Cuando se ejecute op_startup() en el main.cpp se cambiará esta propiedad
   * al valor establecido en Trackio::Conf.primaryOpMode
   */
  uint8_t opmode;

  /**
   * @brief Modo operacional principal.
   *
   * Por defecto el sistema siempre se inicia en modo `OP_STARTUP`. Una vez
   * que el startup finaliza se cambia al modo operacional de
   * Trackio::Conf.mode que el que se indica aquí.
   */
  uint8_t primaryOpMode;

  /**
   * @brief En el modo auto indica si debe ir a dormir después de enviar
   * una posición al servidor.
   *
   * El tiempo que estará dormido será el valor de conf.gpsInterval
   */
  bool sleep;

  /**
   * @brief Valor mínimo de VBAT requerido para entrar en modo operacional OP_LOW.
   *
   * Al llegar a este límite el sistema entrará en modo de muy bajo consumo,
   * apagando tanto el mcu como el Sim868
   */
  unsigned int requiredVbat;

  /**
   * @brief Valor mínimo VIN requerido para entrar en modo operacional OP_LOW.
   *
   * Al llegar a este límite el sistema entrará en modo de muy bajo consumo,
   * apagando tanto el mcu como el Sim868
   */
  unsigned int requiredVin;

  /**
   * @brief Valor mínimo VSYS requerido para entrar en modo operacional OP_LOW.
   *
   * Este valor de Vsys5v puede verse representado en los esquemáticos de PCBs
   */
  unsigned int requiredVsys5v;

  /**
   * @brief Tiempo en el cual se realizará un envío de datos al servidor
   *
   * En todas las transmisiones se enviará un único byte, cada X transmisiones
   * se realizará un envío de GPS, esta X la determina la variable gpsInterval.
   *
   * El envío del byte es un modo continuo de indicar al servidor que "estoy vivo"
   */
  unsigned int transmissionClock;
};

extern struct Conf cfg;

class Trackio {
  public:
    /**
     * @brief Almacena una trama GPS. Trackio::transmitGps() utilizará esta trama
     * para realizar el envío de datos al servidor
     *
     */
    struct GPSData {
      int fix;
      char time[20];
      char lat[15];
      char lon[15];
      char alt[10];
      char sog[7];
      char cog[7];
      char hdop[5];
      char sats[5];
    } gps {0, "\0", "\0", "\0", "\0", "\0", "\0", "\0", "\0"};

    /**
     * @brief Struct para control de timers.
     *
     * Los timers permiten configurar cada cuanto tiempo se realiza una acción,
     * por ejemplo enviar una trama GPS. El timer base sirve como comparador
     * para el resto de timers (realiza el cálculo) y se inicializa en el método
     * Trackio::sayHello(), tras recibir el tiemstamp del servidor.
     */
    struct Timers {
      /**
       * @brief Timer principal. Al entrar al loop se actualiza con millis()
       *
       * El resto de timers utilizan este para comparar el tiempo pasado
       * desde la última medición. Trackio::Timers.base solo se inicializa cuando
       * se sincroniza el relog con el servidor, en el método Trackio::sayHello()
       *
       * Hasta que Trackio::Timers.base no se inicialice el resto de relojes
       * (timer1, timer,2 ...) tampoco lo hará
       */
      unsigned long base;

      /**
       * @brief Timer para obtener posición GPS
       *
       * Utiliza baseTimer como comparador y trackio.conf.gpsInterval para
       * establecer el tiempo
       */
      unsigned long transmissionClock;
    } timers {0, 0};

    /**
     * @brief Ejecuta un blink de 100ms con las repeticiones indicadas en time.
     *
     * @param times
     */
    void blink(uint8_t times);

    /**
     * @brief Almacena el voltaje de la batería interna (VBAT)
     * Se obtiene mediante el método Trackio::getBattery()
     */
    unsigned int vbat;

    /**
     * @brief Almacena el voltaje de la batería externa (VIN)
     * Se obtiene mediante el método Trackio::getBattery()
     */
    unsigned int vin;

    /**
     * @brief Valor de VSYS_5V (según esquemáticos en PCBs... deimos, halley)
     */
    unsigned int vsys_5v;

    /**
     * @brief Número de IMEI del simcom
     *
     * Se utiliza como contraseña del dispositivo para conectar con el servidor
     */
    char imei[16];

    /**
     * @brief Indica si el registro de la red GSM se ha llevado a cabo
     */
    bool cregOk;

    /**
     * @brief Indica si se ha realizado GPS FIX
     */
    bool gpsFix;

    /**
     * @brief Indica si el modem está conectado y funcionando
     *
     * checkModem() envía comando AT y ATE0, si hay respuesta válida
     * se establece esta variable como true.
     *
     * En caso de respuesta invalida, FALSE, podría indicar que el módulo del
     * modem (¿Sim868?) no está bien conectado o es defectuoso. También podría
     * darse el caso de una mala configuración de pines que impidan al
     * microcontrolador comunicar con el modem. Trackio.h contiene la
     * configuración de pines
     */
    bool atOk;

    /**
     * @brief Indica si se ha abierto la conexión TCP con el servidor
     */
    bool tcpOk;

    /**
     * @brief Indica si se ha conseguido enviar el mensaje de presentación al server.
     *
     * Cuando se conecta con el servidor (se abre TCP) el micro escribará el IMEI
     * del modem, el servidor lo recibirá y registrará al dispositivo. El modem
     * (su imei) tiene que darse de alta previamente en el servidor para que este
     * pueda validarlo.
     *
     * Si el servidor recibe un IMEI que no conoce cerrará automaticamente la
     * conexión y no se podrán realizar envíos.
     */
    bool sendOk;

    /**
     * @brief Almacena un comando recibido por TCP.
     *
     * Este comando no tiene porque ser válido, esta variable simplemente lo
     * almacena desde tcpHasCommand(). El método processCommand() se encarga
     * de validar y ejecutar la orden del comando.
     */
    char cmd[20];

    /**
     * @brief Almacena el ultimo dato de calidad de señal GSM.
     * @see Trackio::getSignalStrength()
     */
    char gsm[4];

    /**
     * @brief Cuenta el número de ciclos en el que se produce el timers.transmissionClock
     *
     * Este contador se utiliza para determinar cuando se debe enviar una posición GPS.
     * Por ejemplo si cfg.gpsInterval=3 se requerirá que transmissionClockCounter=3
     * para que se inicie el envío de GPS
     */
    uint8_t transmissionClockCounter;

    // #########################################################################

    /**
     * @brief Inicializa varios componentes como los puertos serial, Sim868...etc
     *
     * * Habilita los puertos UART
     * * Configura los pines del Sim868 (Trackio::Configure())
     * * Enciende el Sim868 (Trackio::powerOn())
     * * Carga la configuración guardad en EEPROM (Trackio::loadConf())
     *
     * @return true  Si powerOn() se ejecuta con éxito.
     * @return false De lo contrario
     */
    bool begin();

    /**
     * @brief Establece el modo de las distintas entradas digitales
     */
    void configure();

    /**
     * @brief Enciende el modem
     *
     * Encender el modem supone activar a HIGH la línea PWR_EN y activar
     * PWR_KEY dando un pulso de 1.5s a HIGH.
     *
     * @return true  Si la línea GSM_STATUS obtiene el valor alto. GSM_STATUS
     *               está definido en la configuración de pines de Trackio.h
     * @return false Si GSM_STATUS recibe valor LOW (el momdem no se enciende)
     */
    bool powerOn();

    /**
     * @brief Apaga el SIMCOM por completo (actualmente en desuso)
     *
     * @todo Test Implementar este método, apagar líneas PWR_KEY y GPS_EN del modem
     */
    void powerOff();

    /**
     * @brief Llama a printIccid() y getImei()
     *
     * No realiza nada con la info devuelta, simplemente la escribe en SerialMon
     * para verla en consola
     *
     * OJO: este método es el único encargado de llamar a getImei(). Si no ejecutamos
     * printInfo() en nuestro main.cpp deberemos ejecutar manualmente el método
     * printImei() ya y que este (el imei) se almacena en una variable para ser
     * usado a posteriori en las comunicaciones con el servidor y es IMPRESCINDIBLE
     *
     * @todo Se debe añadir más info, como version del firmware del modem, soporte
     * Bluetooth, etc...
     */
    void printInfo();

    /**
     * @brief Muestra el ICCID en consola
     *
     * El ICCID es un valor dado por la eSim, en caso de que falle esta lectura
     * probablemente sea debido a que el chip eSim tiene algún problema de conexión
     * (teniendo en cuenta que el resto de operaciones funcionan con normalidad)
     */
    void printIccid();

    /**
     * @brief Ejecuta comando AT+CBC para obtener lectura de voltage del modem
     */
    void printBattery();

    /**
     * @brief Lee el estado de la batería
     *
     * Si cfg.battMode=1 se leerá batería del simcom (AT+CBC), solo VBAT
     * Si cfg.battMode=2 se leerá batería de los pines analógicos.
     * Si cfg.battMode=3 se utilizará el ADC TLA2024
     *
     * El resultado se añadirá a la trama GPS en función de Trackio::Conf.addBattToGps
     */
    void getBattery();

    /**
     * @brief Lectura de batería del simcom
     *
     * En ocasiones es posible que las líneas analógicas no estén disponibles
     * para su lectura. Establecemos VBAT utilizando la lectura que nos de el
     * Simcom. VSYS y VIN quedarán en desuso
     */
    void getSimcomBattery();

    /**
     * @brief Lectura de baterías utilizando las líneas analógicas del simcom
     * Se leerá VBAT, VIN y VSYS. Los pines analógicos para cada línea se
     * indican en el archivo static-conf.h.
     *
     * Utiliza el método Trackio::readAnalogBatt() para obtener las lecturas.
     */
    void getAnalogBattery();

    /**
     * @brief Utiliza readAnalog() de arduino para obtener la lectura de los
     * pines analógicos. Se utiliza este método junto con getAnalogBattery()
     *
     * @param adc_pin Pin a leer
     */
    uint16_t readAnalogBatt(byte adc_pin);

    /**
     * @brief Lectura de baterías utilizando el ADC TLA2024
     *
     * Obtenemos la lectura para VBAT, VSYS y VIN. Este ADC está disponible en
     * algunas placas, como Halley Box. Utiliza el puerto I2C (Ocupará las
     * líneas SDA/SCL).
     *
     * Utiliza el método Trackio::readTLA2024Battery() para realizar la lectura
     */
    void getTLA2024Battery();

    /**
     * @brief Método para lectura de baterías con el ADC TLA2024. Utiliza la
     * clase `TLA2024.h` para leer por I2C.
     */
    float readTLA2024Battery(byte channel, float aux);

    /**
     * @brief Comprueba el estado de las baterías para determinar si se entra
     * en modo bajo consumo.
     *
     * En la configuración `Conf` se crean los valores mínimo por los cuales
     * se entrará en modo de b ajo consumo, estos son `cfg.requiredVbat`,
     * `cfg.requiredVsys5v` y `cfg.requiredVin`. Estos valores se pueden ajustar
     * desde el archivo `static-conf.h`.
     */
    void checkLowBattery();

    /**
     * @brief Muestra y guarda la calidad de la señal GSM
     *
     * Posible valores:
     *  - 2  Marginal
     *  - 3  Marginal
     *  - 4  Marginal
     *  - 5  Marginal
     *  - 6  Marginal
     *  - 7  Marginal
     *  - 8  Marginal
     *  - 9  Marginal
     *  - 10 OK
     *  - 11 OK
     *  - 12 OK
     *  - 13 OK
     *  - 14 OK
     *  - 15 Good
     *  - 16 Good
     *  - 17 Good
     *  - 18 Good
     *  - 19 Good
     *  - 20 Excellent
     *  - 21 Excellent
     *  - 22 Excellent
     *  - 23 Excellent
     *  - 24 Excellent
     *  - 25 Excellent
     *  - 26 Excellent
     *  - 27 Excellent
     *  - 28 Excellent
     *  - 29 Excellent
     *  - 30 Excellent
     */
    void getSignalStrength();

    /**
     * @brief Ejecuta comando `AT+CPIN?` para ver el estado del pin de la sim
     */
    void printPin();

    /**
     * @brief Muestra el IMEI en consola y lo almacena en la variable Trackio::imei
     */
    void getImei();

    // #########################################################################
    /**
     * @brief Llama a checkModem() y checkCreg(), devuelve TRUE si ambos métodos
     * también lo hacen.
     *
     * @return true Si el estado es válido para continuar
     * @return false Si hay algún problema. Por favor revisa checkModem() y checkCreg()
     */
    bool checkStatus();

    /**
     * @brief Envia comando AT y ATE0 y verifica OK en respuesta.
     *
     * @return true  Indicaría que el Simcom está listo para funcionar
     * @return false No se puede continuar con el aplicativo, hard reset podría
     *               ser necesario
     */
    bool checkModem();

    /**
     * @brief Verifica el registro de red
     *
     * Envía el comando `AT+CREG?` y espera que la respuesta contenga las cadenas
     * ",5" o ",1". `AT+CREG` devuelve dos valores, estado del registro y tipo
     * de tecnología. Se representa con dos números separados por coma: "0,5",
     * "0,1", "0,3", "3,1"...El primero indica el tipo de tecnología, el segundo
     * el estadoo del registro:
     *
     * Possible values of registration status are,
     *  - 0 not registered, MT is not currently searching a new operator to register to
     *  - 1 registered, home network
     *  - 2 not registered, but MT is currently searching a new operator to register to
     *  - 3 registration denied
     *  - 4 unknown (e.g. out of GERAN/UTRAN/E-UTRAN coverage)
     *  - 5 registered, roaming
     *  - 6 registered for "SMS only", home network (applicable only when indicates E-UTRAN)
     *  - 7 registered for "SMS only", roaming (applicable only when indicates E-UTRAN)
     *  - 8 attached for emergency bearer services only (see NOTE 2) (not applicable)
     *  - 9 registered for "CSFB not preferred", home network (applicable only when indicates E-UTRAN)
     *  - 10 registered for "CSFB not preferred", roaming (applicable only when indicates E-UTRAN)*

     * Possible values for access technology are,
     *  - 0 GSM
     *  - 1 GSM Compact
     *  - 2 UTRAN
     *  - 3 GSM w/EGPRS
     *  - 4 UTRAN w/HSDPA
     *  - 5 UTRAN w/HSUPA
     *  - 6 UTRAN w/HSDPA and HSUPA
     *  - 7 E-UTRAN
     *
     * @return true Si encontramos ",5" o ",1"
     * @return false
     */
    bool checkCreg();

    // #########################################################################

    /**
     * @brief comprueba si está conectado a GPRS
     *
     * Ejecuta comando AT+CGATT? sobre el modem y espera recibir un OK
     *
     * @return true  Si el GPRS está habilitado
     * @return false
     */
    bool gprsIsOpen();

    /**
     * @brief
     *
     * @return true
     * @return false
     */
    bool enableGprs ();

    // #########################################################################

    /**
     * @brief Habilita GPS
     *
     * Ejecuta los comandos `AT+CGNSPWR=1` y `AT+CGPSRST=0` y pone a HIGH la
     * línea `GPS_EN`
     *
     * @return true Si AT+CGNSPWR=1 devuelve OK
     * @return false
     */
    bool powerOnGps ();

    /**
     * @brief Obtiene una trama GPS con el comando `AT+CGNSINF`
     *
     * Al recibir la trama la enviará al método Trackio::parseGps()
     *
     * @todo Este método no debe enviar nada por TCP debería devolver un char
     * con la trama y utilizar sendGps() para el envío
     */
    void getGps();

    /**
     * @brief Resetea los valores del struct Trackio::GPSData
     *
     * Simplemente se establece la propiedad FIX a 0. Trackio::transmitGps()
     * comprobará el valor de Trackio::gps.fix antes de realizar el envío.
     */
    void resetGpsData();

    /**
     * @brief Verifica que una trama GPS tenga FIX.
     *
     * La trama debe ser de tipo GNSS, se obtiene con comando AT `CGNSINF`.
     * En esta trama el segundo valor es un 0 o 1 en función de si tiene fix
     * o no.
     *
     * @return true  Si hay fix
     * @return false
     */
    bool gpsHasFix(char * gps);

    /**
     * @brief Recibe una trama GPS de tipo CGNSINF y utiliza strtok() para
     * extraer los datos y almacenarlos en Trackio::gps
     *
     * @param gps
     */
    void parseGps(char * gps);

    /**
     * @brief Parsea el valor de time devuelto por simcom en la con CGNSINF
     *
     * El valor de entrada tiene este formato, 20181010115128.000 y queremos
     * convertirlo a 20181010115128
     *
     * @param time
     */
    void parseSimcomTime(char * time);

    /**
     * @brief Realiza todo el operativo para descargar y actualizar el EPO
     * file con las efemeridas de los satelites.
     *
     * Se trata de un proceso relativamente complejo, que requiere la ejecución
     * de muchos comandos, incluída la descarga de un fichero desde un servidor
     * FTP y su procesamiento para guardarlo en memoría fisica.
     *
     * Los detalles están especificados en el documento _SIM868 GNSS AGPS
     * Application Note_
     *
     * @see http://simcom.ee/documents/SIM868/SIM868_GNSS_AGPS_Application%20Note_V1.00.pdf
     * @return true
     * @return false
     */
    bool gpsDownloadEpo();

    // #########################################################################

    /**
     * @brief Inicializa el contexo PDP de la conexión y abre puerto TCP
     *
     * @return true Si el puerto se abre correctamente
     * @return false
     */
    bool openTcp();

    /**
     * @brief Envía comando al simcom para determinar si la conexión TCP está abierta
     *
     * En el modo OP_TCP se hace esto de forma recurrente para verificar que no
     * se ha perdido la conexión con el servidor.
     *
     * Se utiliza el comando `AT+CIPSTATUS`. Simcom dispone de otro modo para
     * verificar si el TCP está abierto: el pin DCD. A día de hoy este pin no
     * está ruteado a nivel de Hardware por lo que no es accesible (sim868 v1.1).
     *
     * @return true Si la conexión está abierta
     * @return false
     */
    bool tcpIsOpen();

    /**
     * @brief Cerrar conexión TCP
     *
     * Se utiliza el comando AT+CIPCLOSE que admite dos valores, 0=slow close y
     * 1=quick close. Según la documentación:
     *
     * User can use the command AT+CIPCLOSE=<mode> to close the TCP or UDP
     * connection. If <mode> is 0, it is slow closing, if <mode> is 1, it is
     * quick closing. In slow closing, the module will interactive with the
     * server when it closes the TCP connection. Thus, the time of returning
     * “CLOSE OK” will be a bit long. This method is suitable for steady network.
     * In quick closing, the module will disconnect the connection compulsorily
     * and return “CLOSE OK” immediately, without interaction with the server.
     *
     */
    void closeTcp(int mode);

    /**
     * @brief Escribe un comando en el TCP para que el servidor le de acceso
     *
     * Básicamente se escribe el imei, el servidor lo recibe y lo valida. Si
     * es OK mantiene el TCP abierto, si el servidor desconoce el IMEI cerrará
     * la conexión TCP y no permitirá envío de datos.
     *
     * @return true
     * @return false
     */
    bool sayHello();

    // #########################################################################

    /**
     * @brief Ejecuta un comando recibido por TCP o cualquier otro canal
     *
     * Los comandos están compuestos por 5 caracteres, ejemplo: #293$. Esto
     * se interpreta como:
     *
     *  - # = Inicio del comando
     *  - 293 = Comando
     *  - $ = fin del comando
     *
     * Cualquier variación en el formato de este comando se interpreta como
     * comando inválido. Por ejemplo: #23$ es incorrecto, debería ser #023$
     *
     * Por el bien de la integridad de los datos, es muy importante ser
     * estricto en cuanto al formato de los comandos y su implementación. Es
     * difícil, pero se puede dar el caso de que se pierda algún caracter
     * durante el envío de información entre cliente-servidor y viceversa. En
     * el caso del ejemplo anterior, `#23$` podría tener un caracter perdido
     * e inicialmente el comando era #213$. El comando #23$ podría ejecutar
     * una autodestrucción del dispositivo y la estaríamos liando parda.
     *
     * Cualquier comando inválido escribirá en consola `UNKNOW COMMAND` y
     * devolverá FALSE. Si se detecta que devuelve false se debe escribir
     * un comando KO al servidor para informar que el comando recibido no es
     * válido.
     *
     * @param cmd
     * @return true
     * @return false
     */
    bool processCommand(char * cmd);

    /**
     * @brief Separamos la propiedad y el valor de un comando.
     *
     * Por ejemplo, en el comando #IO1:ON$ la propiedad es IO1 y el valor ON,
     * es decir, activa el la linea digital IO1
     *
     * @param cmd
     * @return Command
     */
    Command splitCommand(char * cmd);

    /**
     * @brief Comprueba el formato de `char * cmd` para validar si es un comando.
     *
     * Se espera que empiece con $ y acabe con #. No se comprueba que el comando
     * exista y sea realmente válido. Utilizamos este método con Trackio::transmit()
     * para comprobar si el servidor devuelve algún comando cuando escribimos en
     * el socket.
     */
    bool isCommand(char * command);

    /**
     * @brief Lee el serial del simcom para ver si tiene algún dato.
     *
     * Esto se hace de forma recursiva en el loop de main.cpp mientras el
     * modo operacional es OP_TCP
     *
     * @return true
     * @return false
     */
    bool tcpHasCommand();

    /**
     * @brief Convierte un comando tipo $CMD:XXX# en CMD:XX
     *
     * @param cmd El comando con los símbolos de inicio/fin
     * @return char* El comando sin los símbolos de inicio/fin
     */
    char * extractCommand(char * cmd);

    /**
     * @brief Cambia el estado de un pin, ya sea su valor (digitalWrite())
     * o modo de funcionamiento (pinMode())
     *
     * @param IO Número de pin
     * @param status 4 posibiles valores: ON (HIGH), OFF (LOW), OUT (OUTPUT), IN (INPUT)
     * @todo Validar status, tiene que tener una de las 4 opciones
     */
    bool cmd_setIO(int io, char * status);

    /**
     * @brief Establece una propiedad de configuracioń en trackio::Conf.
     *
     * Se espera que el parámetro conf contenga una cadena con el formato
     * "propiedad:valor". Las propiedades de configuración se definen en el
     * struct Trackio::Conf y cada nombre de propiedad se ha acortado para
     * reducir el consumo de datos. Tabla de posibles valores:
     *
     * * airQuality: aq
     * * apn: apn
     * * apn_user: apnur
     * * apn_pass: apnp
     * * battInterval: bati
     * * deepSleep: dslp
     * * dht11_pin: dht
     * * gpsInterval: gpsi
     * * mqgas_pin: mq
     * * sensingInterval: seni
     * * opmode: op
     * * primaryOpMode: pop
     * * port1: prt1
     * * port2: prt2
     * * server1: srv1
     * * server2: srv2
     * * sleep: slp
     * * tcpAliveInterva:tcpi
     * * waitForGpsFix: wfix
     * * gpsFixBeforeTcpSend: gfbs
     *
     * La propiedades de tipo booleano tendrán valores char 0 y 1, las númericas
     * se convertirán con `atoi()`, los strings (char) se asignarán sin más.
     *
     * Un comando puede establecer varias propiedades, para ello se utilizará
     * el símbolo `;`. Ejemplos:
     *
     * ```C
     * // Comando simple
     * char conf[] = "gpsi:60" // intérvalo de GPS a 60 segundos
     *
     * // Multiples comandos
     * // GPS a 60 secs | Sensing a 60 secs | op mode auto
     * char conf[] = "CFG|gpsi:60;seni:60;op:auto"
     * ```
     *
     * @param conf Se espera un char* con dos valores separados por símbolo de
     * tabulacion, propiedad de configuración y valor.
     * @return bool True si la configuración se aplica con éxito
     */
    bool cmd_setConf(char * conf);

    /**
     * @brief Recibe una configuración, propiedad:valor, y la aplica en el struct
     * Trackio::Conf
     *
     * @param conf
     * @return true
     * @return false
     */
    bool applyConf(char * conf);

    /**
     * @brief Establece modo operacional primario y resetea el dispositivo.
     *
     * Tambien se puede cambiar el modo operacional através de un comando de
     * comanfiguración en Trackio::cmd_setConf() aunque en este caso no se
     * reiniciaría el dispositivo en modo STARTUP.
     *
     * @param op
     */
    void cmd_setOP(char * op);

    // #########################################################################

    /**
     * @brief Envía un mensaje al servidor, puede ser cualquier cosa, desde un
     * comando, a una trama GPS o la lactura de un sensor.
     *
     * Si el envío falla se establecerá el modo OP_STARTUP como medida de
     * precaución. Una opción mas viable en un futuro sería realizar el envío a
     * un segundo servidor (Trackio::Conf.server2) y verificar si el modem
     * tiene acceso a internet (puede ser el servidor el que haya caído)
     *
     * @param msg
     * @return true  Si la transmisión se realiza correctamente
     * @return false
     */
    bool transmit(char * msg);

    /**
     * @brief A
     *
     * @param buffer
     * @return char*
     */
    char * getTransmitResponse(char * buffer);

    /**
     * @brief Envía una trama al servidor
     *
     * Construye la trama utilizando un sprintf para luego transmitirla mediante
     * Trackio::transmit()
     *
     * El formato de las tramas está detallado en el README del proyecto.
     *
     * @return true
     * @return false
     */
    bool transmitData(char * gpsraw);

    /**
     * @brief Envía una trama GPS al servidor.
     *
     * La trama se obtiene del struct Trackio::gpsData. Requiere que
     * Trackio::conf.gpsInterval sea mayor que 1.
     *
     * Si no hay GPS Fix (Trackio::gps.fix == 0) se enviará la trama con un cero
     * en lugar de los datos GPS.
     *
     * Ejem:
     * Si hay GPS Fix: IMEI|GPSDATA1,GPSDATA2,GPSDATA3|BATT,EXTBATT
     * Si NO hay GPS Fix: IMEI|0|BATT,EXTBATT
     *
     * @return true
     * @return false
     */
    bool transmitGps();

    // #########################################################################

    /**
     * @brief Establece la configuración del Watchdog a 8 segundos.
     * delay interval patterns:
     * 16 ms:     0b000000
     * 500 ms:    0b000101
     * 1 second:  0b000110
     * 2 seconds: 0b000111
     * 4 seconds: 0b100000
     * 8 seconds: 0b100001
     */
    void configureSleep();

    /**
     * @brief
     */
    void sleepNow();
    void sleepNow(float cycles);

    /**
     * @brief Wrapper para el método delay() que incluye reset del watchdog
     *
     * Máximo 7.5 segundos (recomendable menos). Se realiza un reset antes y
     * después de ejecutar el delay. Si se requiere un delay más largo, p.e 1
     * minuto, se debe ejecutar el método tantas veces como sea necesario con
     * mediciones inferiores a 8000:
     *
     * ```c
     * // delay de 20 segundos
     * Trackio::delay(5000);
     * Trackio::delay(5000);
     * Trackio::delay(5000);
     * Trackio::delay(5000);
     * ```
     *
     * Un delay superior a 8 segundos hará que el watchdog salte y se reinicie
     * el dispositivo.
     *
     * @param time Tiempo en milisegundos. Máximo 8 segundos (8000) pero se
     * recomienda un valor inferior, p.e. 7000
     */
    void _delay(int time);

    /**
     * @brief Fuerza llamada al watchdog para realizar un hard reset
     *
     * Básicamente se ejecuta un loop infinito, esto hará saltar el watchdog
     * pasados 8 segundos y el micro se reiniciará
     */
    void hardReset();

    // #########################################################################

    /**
     * @brief Lee la configuración almacenada en EEPROM y la aplica a la
     * configuración de la aplicación (Trackio::Conf)
     */
    void loadConf();

    /**
     * @brief Guarda la configuración de Trackio::Conf en memoria EEPROM
     */
    void saveConf();

    /**
     * @brief Genera un código CRC con la configuración (Struct Conf)
     *
     * Este código indica que configuración de software utilizamos. A partir
     * de un mismo firmware se pueden generar distintas configuraciones. Esto
     * hace que el archivo HEX final, aunque lleve el mismo firmware,
     * corresponda a distintas versiones del software.
     *
     * Dos trackers pueden tener el mismo firmware pero distintas versiones de
     * configuración (distinto archivo .HEX). El CRC de la configuración se
     * muestra en consola en cada reset y debe formar parte del nombre del
     * archivo HEX en caso de que se quiera distribuir.
     */
    void confCRC();

    // #########################################################################

    /**
		 * Envía un comando por serial al modem. Se espera a recibir respuesta
		 * y almacena el resultado en `buffer` (definida en Trackio.cpp).
		 *
		 * Utiliza el método `println` de la clase serial, no existe alternativa
		 * para el método `print`, pero se puede acceder directamente al objeto
		 * HardwareSerial con `SerialSim` en el main.cpp.
		 *
		 * @param cmd Comando a ejecutar, se utiliza println
		 * @return true Si obtuvo respuesta
		 * @return false
		 */
    bool sendCommand(char * cmd);

    bool sendCommand(const char * cmd);


		/**
		 * Version de sendCommand en la que se comprueba que la respuesta recibida
		 * del comando AT contenga la cadena pasada por segundo parámetro. Es útil
		 * para comprobar si tenemos OK o ERROR.
		 *
		 * Devuelve true si hay respuesta y contiene el segundo parametro, false
		 * si no hay respuesta o no contiene el segundo parámetro
		 *
		 * @param cmd Comando a ejecutar, se utiliza println
		 * @param res Respuesta que se espera que devuelva. Se utiliza `strcmp` para
     *            la verificación
		 * @return true Si obtuvo respuesta
		 * @return false
		 */
		bool sendCommand(char * cmd, char * res);

		// Especifica un tiempo antes de llamar a Serial.available()
		bool sendCommand(char * cmd, char * res, int time);
		bool sendCommand(char * cmd, int time);

  private:
};

#endif
