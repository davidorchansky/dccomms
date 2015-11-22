# ROVRadioTransmission

### DESCRIPCIÓN:
Interfaz de control de dos Arduinos que forman un sistema de comunicación por radio *half-duplex* (ver [TFM Diego Centelles](https://drive.google.com/file/d/0ByiyD66UpEnfc3BMVE1abFF1TGM/view?usp=sharing)).

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
* [*nR24l01 2.4 GHz*](http://elecfreaks.com/store/download/datasheet/rf/rf24l01_PA_LAN/nRF24L01P.PDF)

### USO DEL SISTEMA DE VIDEO

En el directorio "*ImageTransmission*":

* En el "nodo emisor" ejecutar, según la implementación (ver [TFM Diego Centelles](https://drive.google.com/file/d/0ByiyD66UpEnfc3BMVE1abFF1TGM/view?usp=sharing)) del módulo 1 que se quiera usar:

	* Primera implementación (dependencia con *FFmpeg*):
        
       		./videoRovTx.sh <ImageWidth> <ImageHeight> <EncodedImageSize> <MaxPayloadSizePerRadioFrame> <DelayBetweenFrames> <MaxFrameAge>
        * *ImageWidth*: Anchura, en *pixels* de la imagen.
        * *ImageHeight*: Altura, en *pixels* de la imagen.
        * *EncodedImageSize*: El tamaño que debe tener cada cada cuadro de video (imagen) comprimido.
        * *MaxPayloadSizePerRadioFrame*: El tamaño máximo del campo de datos de cada trama de radio.
        * *DelayBetweenFrames*: Es el tiempo, en milisegundos, que hay entre el envío de una trama y la siguiente.
		* *MaxFrameAge*: Es el tiempo máximo permitido, en milisegundos, desde que se comprime la imagen hasta que el modulo transmisor (proceso) decide procesarla para enviarla a través del canal de radio. Si el módulo transmisor recibe una imagen comprimida muy antigua, la descarta. De esta manera podemos conseguir que el video en la recepción sea lo más reciente posible.
		
     * Segunda implementación:
    
    		./videoRovTx_v2_noFFmpeg.sh <ImageWidth> <ImageHeight> <EncodedImageSize> <MaxPayloadSizePerRadioFrame> <DelayBetweenFrames> <MaxFrameAge>
        >  Nota: El significado de los parámetros es el mismo que en la primera implementación.	

	* Tercera implementación:

			./videoRovTx_v3_joinedGrabberAndEncoder.sh <ImageWidth> <ImageHeight> <EncodedImageSize> <MaxPayloadSizePerRadioFrame> <DelayBetweenFrames> <MaxFrameAge>
         >  Nota: El significado de los parámetros es el mismo que en la primera y segunda implementación.	

	La compresión de la imagen se puede configurar editando los argumentos pasados al *encoder* dentro del script *videoRovTx\*.sh*. Los parámetros de compresión son los siguientes:

        -t transform     - Wavelet used (CDF-9/7) and (5/3, 9/7-M, 13/7-T, haar) <ibior-13/7-T>
        -n nbands        - Number of transformations <6>
        -k depth         - Decompose for depth <= otherwise partition <0>
        -y val           - Dynamic decomposition/partition <0>
        -r refmul        - Refinement mutliplier <0.800000>
        -q quality       - Quality value (0 = max) <0.000000>
        -u resolution    - Resolution order <0>
        -x maxsize       - Truncate output at maxsize bytes <0>

        -p pad           - Pad codes when mapping <0>
        -m map           - Output map codes <0>
        -s esc           - Escape 0xff on output stream <0>

        -l left-shift    - ROI left shift <0>
        -i x0,y0,x1,y1   - ROI rectangle (Top-Left and Bottom-Right) (repeat)

        -h               - Benchmark <0>

* En el "nodo receptor" ejecutar:

	    ./videoRovRx.sh <ImageWidth> <ImageHeight>

### INSTALACIÓN Y USO EN UN OpenROV 2.6

Editando
