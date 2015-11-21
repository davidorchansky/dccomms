(Actualizando README con instrucciones de uso...)
# ROVRadioTransmission

### DESCRIPCIÓN:
Interfaz de control de dos Arduinos que forman un sistema de comunicación por radio *half-duplex*.

### DEPENDENCIAS: 
Dependencia (temporal) con la Biblioteca "*crypto*". Para instalar:

		sudo apt-get install libssl-dev 

Esta biblioteca se usa para la comprobación MD5. Se eliminará esta dependencia próximamente.

### COMPILACIÓN DE BIBLIOTECA *libradio.a*:

Entrar en el directorio "*Release*" y ejecutar:  

		make -f radio.mk
        
Opciones especiales de compilación mediante:

		make -f radio.mk RADIOFLAGS="<definición-de-constantes-aquí>"
        
Opciones:
* **DEBUG**: se muestra toda la información de cada frame (datos y overhead) que se envía o recibe, y si contiene
o no errores.
* **TIMMING**: se muestra el tiempo de recepción de un bloque (imagen, fichero, etc.) cuando se llama al método "*Receive*" de *BlockRadioTransmitter*.
* Otras:
	* **BBB**: Se tienen en cuenta los puertos serie */dev/ttyO[1-5]* de una *Beablebone Black* cuando se llama al método *FindArduino* de un objeto de tipo *Arduino*.
	* **BADWRITE**: Indica que se escriba en el puerto serie 1 byte por llamada a la función POSIX write.
	* **BADWRITE2=X**: Indica que cada llamada a la función de POSIX write escriba un maximo de X bytes.
> Nota: Parece que el SO de la Beablebone Black tiene problemas al cuando se escriben demasiados bytes a la vez por el puerto Serie con la llamada a *write*... Por eso lo de las opciones "BADWRITE". Esto no es correcto, hay que investigar por que pasa y arreglarlo mejor.

Ejemplo con varias opciones de compilación:

		make -f radio.mk RADIOFLAGS="-DDEBUG -DBBB -DBADWRITE2=15"


### COMPILACIÓN DEL SISTEMA DE VIDEO:

#### En un PC con Linux instalado:

Entrar en el directorio "*ImageTransmission*" y compilar con: 

		make 
eso generará "*libradio.a*" en el directorio "*Release*" del repositorio, y los binarios necesarios para ejecutar el ejemplo "*ImageTransmission*"
en el directorio "*ImageTransmisssion*". Si se quiere pasar opciones de compilación para *libradio.a*, utilizar la variable *RARGS*. Por ejemplo:

		make RARGS="-DDEBUG -DTIMMING"

#### En ARM:

Entrar en el directorio "*ImageTransmission*" y compilar con: 

		make PRIVATELIBDIR=./libraries/arm
eso generará "*libradio.a*" en el directorio "*Release*" del repositorio, y los binarios necesarios para ejecutar el ejemplo "*ImageTransmission*"
en el directorio "*ImageTransmisssion*". Si se quiere pasar opciones de compilación para *libradio.a*, utilizar la variable *RARGS*. Por ejemplo:

		make PRIVATELIBDIR=./libraries/arm RARGS="-DDEBUG -DTIMMING"
        
Si el dispositivo transmisor/receptor está conectado en alguno de los puertos */dev/ttyO[1-5]*, compilar con:

		make PRIVATELIBDIR=./libraries/arm RARGS="-DBBB"
        
Si, además, el puerto */dev/ttyO[1-5]* es de una *Beaglebone Black*, añadir la opción *-DBADWRITE2=15*:

		make PRIVATELIBDIR=./libraries/arm RARGS="-DBBB -DBADWRITE2=15"
Esto último hace que cada llamada a la función POSIX *write* escriba un máximo de 15 bytes (ya que, por el motivo que sea, escribir más de 15 bytes hace que se pierdan los restantes. Esto se está investigando).
        
      

### INSTALACIÓN DEL PROGRAMA DE LA ARDUINO: 
Cargar el programa de cada Arduino (TX y RX) según módulo de radio a utilizar. El código se encuentra en *src/Arduino*:


* [*BiM2A 433 MHz*](http://www.radiometrix.com/content/bim2a).
* [*BiM3B 868 MHz*](http://www.radiometrix.com/content/bim3b) (El mismo programa que para BiM2A).
* [*FS1000A 315 MHz*](http://www.ananiahelectronics.com/fs100a.gif).
* [*nR24l01 2.4 GHz*](http://elecfreaks.com/store/download/datasheet/rf/rf24l01_PA_LAN/nRF24L01P.PDF
