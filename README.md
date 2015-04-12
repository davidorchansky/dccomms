# ROVRadioTransmission
**DESCRIPCIÓN:**  
Interfaz de control de dos Arduinos que forman un sistema de comunicación por radio half-duplex.

**INSTALACIÓN ARDUINO CODE:**  
Cargar el programa de cada Arduino (TX y RX). El código se encuentra en src/Arduino (radio_rx_micro.ino, radio_tx_micro.ino).

**COMPILACIÓN:**
* **Todo** ("libradio.a" y ejemplo "ImageTransmission"):  
Entrar en el directorio "ImageTransmission" y compilar con: 

		make
eso generará "libradio.a" en el directorio "Release" del repositorio, y los programas "TX" y "RX"  
en el directorio "ImageTransmisssion"
>Nota: Compilando con;:

		make RARGS="-DDEBUG"
>se muestra toda la información (datos y overhead) de cada frame que se envía o recibe, y si contiene
o no errores.

>Nota: Compilando con: 

		make RARGS="-DTIMMING"
>se muestra el tiempo de recepción de un bloque (imagen) cuando se  
>llama al método "Receive" de BlockRadioTransmitter.

* Obtener **sólo "libradio.a"**:  
Entrar en el directorio "Release" y ejecutar:  

		make -f radio.mk


**DEPENDENCIAS:**  
Biblioteca "crypto" (temporal). Para instalar:

		sudo apt-get install libssl-dev 

**USO:**  
Un ejemplo de uso se encuentra en el directorio "ImageTransmission" TX.cpp (sender) y en RX.cpp (receiver) 


**INSTRUCCIONES PARA EJECUTAR EL VIDEO STREAMING DEL ROV:**   

1. Ejecutar "make" en el directorio "ImageTransmission", tanto en el ROV (pasándole EDULIB="ezbtARM") como en el linux conectado al módulo receptor de radio (sin argumentos).  
2. Ejecutar la emisión de vídeo por radio en el ROV: Entrar en el directorio "ImageTransmission" y ejecutar:

        ./videoRovTx.sh <EncodedImageSize> <MaxPayloadSizePerRadioFrame> <DelayBetweenFrames>
3. Ejecutar la visualización del vídeo del ROV en el linux conectado al módulo receptor de radio: Entrar en el directorio "ImageTransmision" y ejecutar:

	    ./videoRovRx.sh <EncodedImageSize>
