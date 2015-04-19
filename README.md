# ROVRadioTransmission
**DESCRIPCIÓN:**  
Interfaz de control de dos Arduinos que forman un sistema de comunicación por radio half-duplex.

**INSTALACIÓN ARDUINO CODE:**  
Cargar el programa de cada Arduino (TX y RX). El código se encuentra en src/Arduino:

* Módulos Radiometrix BiM2A (http://www.radiometrix.com/content/bim2a):  
	*radio_tx_micro* y *radio_rx_micro*

* Módulos FS1000A (http://www.ananiahelectronics.com/fs100a.gif):   
 	*radio_tx_micro_FS1000A* y *radio_rx_micro_FS1000A*

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

        ./videoRovTx.sh <EncodedImageSize> <MaxPayloadSizePerRadioFrame> <DelayBetweenFrames> <MaxFrameAge>
        
>Nota: el último argumento, "MaxFrameAge", es el tiempo máximo permitido, en milisegundos, desde que se comprime la imagen hasta que el modulo transmisor (proceso) decide procesarla para enviarla a través del canal de radio. Si el módulo transmisor recibe una imagen comprimida muy antigua, la descarta. De esta manera podemos conseguir que el video en la recepción sea lo más reciente posible.

3. Ejecutar la visualización del vídeo del ROV en el linux conectado al módulo receptor de radio: Entrar en el directorio "ImageTransmision" y ejecutar:

	    ./videoRovRx.sh <EncodedImageSize>
