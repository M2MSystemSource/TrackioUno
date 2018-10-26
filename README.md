# TrackioUno - GPS/GPRS Tracker and IoT Device

TrackioUno es un firmware escrito en C++ especialmente diseñado para la familia SAMD21 (Cortex-M0) de Microchip. Entre sus principales funcionalidades destaca la posibilidad de trabajar manteniendo un TCP permanentemente abierto, lo que permite una comunicación bidireccional completa entre cliente-servidor.

En Trackio.h se ha documentado cada método de la clase Trackio y hay muchas referencias sobre el uso del módulo Sim868 de Rhomb.io, por lo que también puede ser útil para aprender a utilizarlo.

El firmware está compuesto básicamente de 3 archivos, la clase Trackio en los ficheros Trackio.h y Trackio.cpp y el archivo  main.cpp. Se incluye además el archivo _rhio-pins.h_ para la configuración de pins con la plataforma Rhomb.io y _static-conf.h_ para la configuración de algunos parámetros estáticos no modificables en tiempo de ejecución. También se han incluído algunas dependencias de terceros para gestión de Watchdog/Sleep y FlashMemory

## Descargar el Código

A partir de de la versión 0.3.0 se da compapatibilidad a Duino Zero. Para Duino Uno/Mega se debe usar una versión 0.2.x

TrackioUno está disponible en [Github](https://github.com/M2MSystemSource/TrackioUno). Puedes descargar la [rama master](https://github.com/M2MSystemSource/TrackioUno/archive/master.zip) con los últimos cambios aunque puede no ser estable, si necesitas una versión segura es mejor descargar una [release](https://github.com/M2MSystemSource/TrackioUno/releases).

**IMPORTANTE!** Debes duplicar los archivos `platformio.ini.sample` y `lib/Trackio/static-conf.h.sample` eliminando la extensión `.sample` del modo que obtendrás `platformio.ini` y `lib/Trackio/static-conf.h`. **Si piensas publicar cambios en Git no elimines los archivos `.sample` para que no desaparezcan del repositorio.**

### Duino Uno

La anterior versión, v0.2.x es compatible con Duino Uno (atmega328p) y Mega (Atmega2560/1280). Se mantiene la rama [DuinoUno](https://github.com/M2MSystemSource/TrackioUno/tree/DuinoUno) con esta versión, ademas de todas las releases de esta rama.

## IDE de programación y upload

TrackioUno ha sido creado con [PlatformIO](https://platformio.org/) sobre VS Code. PlatformIO es una utilidad de consola ([PlatformIO Core](http://docs.platformio.org/en/latest/core.html)) que además tiene el plugin ([PlatformIO IDE](https://platformio.org/platformio-ide)) para los editores [Atom](https://atom.io) de Github y [VS Code](https://code.visualstudio.com/) de Microsoft, ambos multiplataforma (Windows/Linux/Mac) y Open Source.

PlatformIO dispone de guías muy completas para su instalación tanto en Atom como en VS Code. El que suscribe ha tenido una mejor experiencia con VS Code.

- [Instalación de PlatfomIO en Atom](http://docs.platformio.org/en/latest/ide/atom.html)
- [Instalación de PlatfomIO en VS Code](http://docs.platformio.org/en/latest/ide/vscode.html)

Al ser una aplicación de consola no es obligatorio usar uno de estos editores, puedes gastar cualquier otro y seguir los pasos en la documentación de Platformio para hacer los flasheos.

Resumiendo la instalación, primero tendremos que instalar el editor, entramos a su web y descargamos la versión para nuestro sistema operativo. En windows tendremos un instalador (siguiente, siguiente... finalizar) y para linux un archivo _.dev_ o _.rpm_. Seguimos los pasos de instalación en cualquier caso. La descarga es completamente gratuita y sin registro.

Una vez instalado el editor debemos utilizar su gestor de plugins, que se conecta a internet para obtener un listado de los plugins disponibles y entre ellos encontraremos PlatformIO (utiliza el buscador). Al instalar PlatformIO veremos su pantalla de Home, donde podemos crear un nuevo proyecto. La primera vez que crees un proyecto PlatformIO tendrá que descargar todas sus dependencias, en el caso de los master _Duino_ de _Rhomb.io_ esto incluye el framework de _Arduino_ (Arduino Core) y las librerías de _Avrlib_ (_avr-tools_, _avrdude_...) por lo que tardará un tiempo en iniciar el proyecto. Los siguientes proyectos se crearán de forma automática sin esperas.

> Si utilizas Directamente TrackioUno sin haber creado previamente un proyecto propio, cuando compiles y hagas el primer upload PlatformIO detectará que no tiene las dependencias por lo que empezará a descargarlas y verás el proceso en la consola. Al acabar la descarga continuará con el upload del código.

## Soporte para módulos master

Desde la version 0.3.x el soporte de módulos master se limita a Duino Zero. La serie 0.2.x Soporta los mcus con arquitectura AVR, Duino Uno y Mega

### Configuración de platformio.ini para upload

En la raíz del proyecto deberemos haber creado el archivo _platformio.ini_ a partir de _platformio.ini.sample_, donde le indicamos a PlatformIO que compilador y método de upload debe utilizar.

~~~ini
[env:zeroUSB]
platform = atmelsam
board = zeroUSB
framework = arduino
build_flags = -D SERIAL_BUFFER_SIZE=128
upload_port = /dev/ttyACM0
~~~

El punto y coma indica un comentario, por lo que ahora no estaría disponible ninguna opción, para programar un _Duino Uno_ descomentas sus variables al tiempo que el resto quedan comentadas:

~~~ini
[env:uno]
platform = atmelavr
board = uno
framework = arduino
upload_port = /dev/ttyUSB0
build_flags = -D SERIAL_RX_BUFFER_SIZE=128

;[env:megaatmega2560]
;platform = atmelavr
;board = megaatmega2560
;framework = arduino
;build_flags = -D SERIAL_RX_BUFFER_SIZE=128
;upload_port = /dev/ttyUSB0

;[env:zeroUSB]
;platform = atmelsam
;board = zeroUSB
;framework = arduino
;build_flags = -D SERIAL_BUFFER_SIZE=128
;upload_port = /dev/ttyACM0
~~~

En `upload_port` debes indicar el puerto en el que tienes conectada la placa, en windows será algo como `COM3`, `COM7`... etc. En Linux `/dev/ttyUSB0` (328P, 2560) o `/dev/ttyACM0` (Samd21, 32u4) entre otros. Si solo tienes un dispositivo serial conectado puedes comentar esta variable (`;upload_port = /dev/ttyUSB0`) y dejar que PlatformIO lo reconozca por si solo, esto es útil cuando al conectar y desconectar la placa varias veces del USB el sistema constantemente nos cambia el nombre del puerto y tenemos que volver a configurarlo.

> Es necesario definir el tamaño del buffer Serial debido a que las tramas GPS con frecuencia superan el límite de 64bytes que el framework de Arduino establece por defecto. 128bytes son sufucientes. En el caso de Duino Uno y Duino Mega podemos añadir el macro en tiempo de compilación através del archivo _platformio.ini_, tal cual lo puedes ver en el ejemplo de arriba. Para el Zero es distinto y tendremos que editar un fichero del framework de Arduino: _{$HOME}/.platformio/packages/framework-arduinosam/cores/samd/RingBuffer.h_ En esta librería está definida la constante `SERIAL_BUFFER_SIZE`, pero no tiene un `#ifndef SERIAL_BUFFER_SIZE` para evitar que se duplique la declaración cuando lo añadimos al _platformio.ini_. La solución pasa por editar directamente este fichero e indicar un valor de 256. **OJO**, esto es un fichero del core de Arduino, por lo que si actualizas PlatformIO o las librerías de Arduino el fichero será sobreescrito y habrá que volver a realizar el cambio.

>En ocasiones se ha producido un error de compilación al utilizar la plataforma atmelavr con los Duino Mega y Duino Uno. El error en concreto dice: `internal compiler error: Segmentation fault`. La solución se encontró en esta issue de Platform.IO y pasa por definir unos flags en el archivo _platformio.ini_. Editamos _platformio.ini_ y en la configuración del mega y uno añadimos: `build_unflags = -flto`. Parece que es un bug de GCC con gcc-avr 4.9.2.

## Configuración de aplicación

La configuración se encuentra predefinida en el struct `Conf` del archivo Trackio.h. Este struct se inicializa por defecto en el método `Trackio::loadConf()` (Trackio.cpp) y se almacena en EEPROM. Por favor, revisa la documentación de `struct Conf` en Trackio.h para obtener detalles de cada configuración

Para editar la configuración debes realizar las modificaciones necesarias en el archivo `static-conf.h` donde se incluyen todas las configuraciones disponibles. No edites directamente los ficheros de la librería como main.cpp, Trackio.cpp o Trackio.h.

### Configuración de Pines

En _platformio.ini_ le decimos al PlatformIO qué compilador y método de upload debe utilizar para nuestra placa, en _static-conf.h_ establecemos la configuración de pines según el microcontrolador y pcb Rhomb.io que se use. Para ello al inicio del archivo (static-conf.h) existen unos macros para establecer la configuración de pines, donde deberás seleccionar una placa:
~~~C
// boards:
//#define DEIMOS_SOCKET1_MEGA2560
//#define DEIMOS_SOCKET2_MEGA2560
//#define HALLEY_SOCKET2_MEGA2560
//#define HALLEY_SOCKET2_UNO
//#define DEIMOS_SOCKET1_ZEROSAMD21
#define DEIMOS_SOCKET1_UNO
~~~

y un microcontrolador:
~~~C
// microcontroller:
//#define PINS_MEGA2560
//#define PINS_ZEROSAMD21D
#define PINS_UNO
~~~

Con esta configuración se establecen los pines necesarios para utilizar el módulo Sim868 de Rhomb.io junto con la PCB y microcontrolador seleccionados.

## Modos operacionles

TrackioUno dispone de varios modos operacionales que establecen como se comporta el dispositivo:

* **OP_STARTUP (0)**: El modo inicial en el que se establecen las configuraciónes básicas: se configura el Sim868, se habilitan redes GPRS, se muestra información del modem en consola... etc.
* **OP_TCP (1)**: Modo de funcionamiento bidireccional. TrackioUno habrirá un puerto TCP y lo mantendrá abierto por siempre y al mismo tiempo enviará tramas en el intervalo de tiempo indicado. El servidor podrá enviar comandos operacionales para cambiar la configuración del sistema y controlar el estado de las GPIOS. En este modo las funciones de Sleep (ahorro de energía) no estarán disponibles.
* **OP_AUTO (2)**: Funcionamiento estandar. El dispositivo enviará tramas al servidor en el tiempo establecido en `cfg.gpsInterval`. Entre cada envío se podrá indicar si se desea entrar en modo de bajo consumo (PWR_DOWN en el mcu) o muy bajo consumo (mcu + sim868)
* **OP_LOW (4)**: Modo de bajo consumo. Se deshabilita cualquier envío de información al servidor y tanto el mcu como el Sim868 entran en modo de bajo consumo. Este modo se habilita automáticamente cuando se llega a alguno de los valores indicados en `cfg.requiredVbat`, `cfg.requiredVin` y `cfg.requiredVsys5v`
* **OP_RST (5)**: Ejecuta un reset del sistema. A efectos se lanzará un while infinito lo cual hará saltar el watchdog del mcu y se volverá a iniciar la aplicación en modo OP_STARTUP

Para habilitar uno de estos modos de forma remota deberemos enviar un comando desde el servidor con el siguiente formato:

```c
// Habilitar el modo OP_TCP
#CFG|op:1$
// Forzar un reset del sistema
#CFG|op:5$
```

Puedes ver en el anterior listado el resto de números para cada modo.

> El modo operacional por defecto siempre debe ser OP_STARTUP, en la configuración de `cfg.primaryOpmode`, se establece el modo al que se cambiará una vez el startup finalice, este debe ser OP_AUTO o OP_TCP. No tendría sentido utilizar como primarios los modos OP_LOW y OP_RST.

## Puertos UART y debug

Trackio necesita dos puertos UART, uno para comunicación con el modem y otro para mostrar información de debug por consola. Siempre que sea posible intentaremos utilizar UART's por hardware, pero en el caso del Atmega328p, donde solo tenemos 1 puerto hardware, este lo usamos para el modem y creamos un puerto via SoftwareSerial para debug. En UART de debug solo hacemos uso del canal TX y en el caso de Atmega328p por defecto seleccionamos el pin _SDA_ como TX.

La configuración de los puertos UART se realiza en el archivo _rhio-pins.h_ en función de la PCB y módulo seleccionado en _static-conf.h_.

Para obtener un debug del dispositivo deberemos conectarnos, en el caso del atmega328p, a los pines _SDA_ y _GND_ con una aplicación Serial a 9600 baudios. Si usamos otro mcu podemos seguir el estandar de Rhomb.io, donde `Serial` se encuentra en los pines `A_TX` y `A_RX` y `Serial1` en `B_TX` y `B_RX`.

## Formato de tramas GPS

Breve resumen. La trama se ha divido en varios grupos de información

|   IMEI    | GPS DATA | BATTERY DATA |
|    :-:    |   :-:    |     :-:      |
| 12 cifras | AT+CGNSINF | vbat,vin,vsys |

> El IMEI solo se enviará cuando se seleccione el modo operacional OP_AUTO, OP_TCP no enviará IMEIs ya y que el servidor remoto almacena el IMEI de la sesión cuando se realiza la primera conexión (`Trackio::sayHello()`)

Se utiliza un formato de texto plano, cada grupo se separa con el símbolo `|`. Dentro de cada grupo los valores se separan con comas

GPS data contiene los siguientes valores:

* Tipo de trama, generalmente un 1, pero puede ser 0 si no hay información de geolocalización
* latitude
* longitude
* altura
* velocidad (sog)
* orientación (cog)
* [HDOP](https://en.wikipedia.org/wiki/Dilution_of_precision_(navigation))
* Nº de satélites GPS
* Calidad de la [señal GSM](http://m2msupport.net/m2msupport/atcsq-signal-quality/)

Cuando el dispositivo no dispone de cobertura GPS se seguirán enviando datos al servidor, el tipo de trama se reemplazará por 0

~~~
// trama sin GPS disponible
867717036798487|0,23|3929,36923,5023
~~~

> La info de batería siempre aparecerá representada aunque sea con ceros: `{IMEI}|...|3872,0,0`

>Las tramas se construyen en el método Trackio::transmitGps()

## Copyright

This software is property of M2M System Source S.L. Developed by Jordi Enguídanos with the contribution of Guillermo Alonso and Pedro Peláez
