# Changelog
* **0.2.5 - 2018-10-16**
  * Todos los valores de configuración se han transferido a `static-conf.h` para evitar modificaciones "innecesarias" en archivos CORE.
  * Añadidas 3 nuevos GPIOS configurables de forma remota, IO7, MOSI y MISO. Eliminada LED01 como configurable de forma remota
  * FIX BUG. Se detecta un error al entrar en modo sleep (Trackio::sleepNow()) por el cual el dispositivo nunca llegaba a entrar en modo bajo consumo (OP_LOW) y esto podía impedir que la batería se cargara correctamente cuando el potenciomtro (P1) tuviera un nivel de carga bajo
  * FIX. Las variables requriedVbat/Vsys5v/Vin no estaban incluídas por defecto en la configuración inicial (Trackio::loadConf())
  * FIX. Trackio::openTcp() existía una comprobación redundante al verificar si el puerto estaba realmente abierto
  * Eliminadas algunas variables en desuso y ajustados los tipos de datos para reducir consumo de ram
  * Cambios menores.


* **0.2.4 - 2018-10-15**
  * En modo OP_TCP se utiliza el método `serialEvent()` (main.cpp) de Arduino para evitar el pool constante al `Serial.available()` en el loop principal.
  * En modo OP_TCP se añade un nuevo pool al servidor (`transmitAlive()`) y su variable de configuración `cfg.transmissionClock`. Este pool junto al anterior serialEvent() permiten que la transmisión de comandos servidor->cliente se realice de forma más eficiente.
  * Valores _required_ de vbat, vin y vsys incluídos en la configuración general
  * Actualizado README.md con información detallada
  * Los ACK al servidor siempre devuelven el estado de la IO6
  * Nuevo archivo de configuración static-conf.h
  * Se crean archivos con extensión .sample para static-conf.h y platformio.ini con el fin de evitar subir los archivos originales a los repositorios públicos.


* **0.2.3 - 2018-10-05**
  * Fix watchdog reset repentino en el startup, se añade wdt_disable() en el setup
  * Enviar en trama VSYS y VIN junto con VBAT
  * Enviar GSM junto con batería cuando no hay GPS
  * Eliminado cfg.battInterval, se mide batería siempre que se envía trama


* **0.2.2 - 2018-10-03**
  * Añadido powerOff en la secuencia de arranque
  * Añadido watchdog reset cuando openTcp falla


* **0.2.1 UNO - 2018-07-27**
  * Eliminado todo el código de Duino Zero, incluídas las librerías externas
    de RTC y SleepyDog para dar compatibilidad exclusiva a Duino Uno
  * Eliminadas variables de configuración y otras funciones (como bluetoth) para
    reducir consumo de memoria de ram.
  * Se utilizan las funciones de eeprom propias de AVR para guardar la configuración
    general. El objeto de configuración se saca de la clase trackio y se convierte
    en un struct global.
  * Añadida lectura analógica de baterías para VBAT, VIN y VSYS.


* **0.2.0 - 2018-07-12:**
  * OP_TCP: timer para verificar estado de la conexión tcp y reconectar en caso necesario
  * Duino Zero: Implementado watchdog+sleep con la librería SleepyDog
  * RTC: Implementado un reloj simple obteniendo la fecha desde servidor web. Se utiliza
    la librería timeLib.h para parsear el unix timestamp en fechas para humanos
  * Se vacía el setup() del main, OP_STARTUP lo reemplaza. Implementado mecanismo de
    protección ante fallos para reestablecer modo OP_STARTUP si necesario
  * OP_AUTO abre/cierra el TCP cuando necesario
  * Tramas de datos: Datos de GPS, Sensing y CAN unificados en una unica trama
    y agregado el método transmitData() para realizar el envío
  * Método sendCommand, agregada limpieza de buffer y aumentado de 15 a 25 el número
    de intentos para esperar respuesta
