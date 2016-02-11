# Canny y Hough
## Compilación

		$ make
## Canny
Recibe por la entrada estandar una imagen RGB en formato ppm y escribe en ficheros pgm (XX-etapa.pgm) el resultado del algoritmo Canny en cada etapa. Tambien genera la version original de 'img.ppm' en ppm (rgb) y ppm (escala de grises)(ver [Algoritmo Canny](http://docs.opencv.org/2.4/doc/tutorials/imgproc/imgtrans/canny_detector/canny_detector.html)).

### Uso
        $ ./canny [opciones] < img.ppm
        
* **[opciones]**:

        -l               - Tamano del filtro gausiano (>=3 e impar).
        -s               - Sigma del filtro gausiano.
        -U               - Umbral superior para deteccion de borde (0 < x < 255).
        -L               - Umbral inferior para deteccion de borde (0 < x < 255).
        -e               - Enviar por la salida estandar la imagen pgm de bordes.
  
* **img.ppm**: imagen RGB en formato ppm.

#### Ejemplos

* Ejecutar el programa enviandole un ppm por la entrada estandar:

		$ ./canny -l 5 -s 1 -U 150 -L 50 < noise2.ppm


* Lo mismo, pero haciendo que el programa escriba el resultado por la salida estandar:

		$ ./canny -l 5 -s 1 -U 150 -L 50 -e < noise2.ppm > bordes.pgm


* Ejecutar el programa enviandole un ppm por la entrada estandar capturado con ffmpeg:

		$ ffmpeg -f video4linux2 -s 1280x720 -i /dev/video0 -f image2pipe -vframes 1 -vcodec ppm pipe:1 | ./canny -l 5 -s 1 -U 150 -L 50


* Capturar una imagen ppm y redireccionarla por la salida estandar:
		
		$ ffmpeg -f video4linux2 -s 1280x720 -i /dev/video0 -f image2pipe -vframes 1 -vcodec ppm pipe:1 > input.ppm

## Hough
Recibe por la entrada estandar una imagen PGM con los pixeles que forman parte del borde de una imagen (resultado final del detector de bordes Canny), y calcula la transformada de Hough (ver [Transformada de Hough](https://ca.wikipedia.org/wiki/Transformada_de_Hough))

### Uso
        $ ./hough [opciones] < bordes.ppm
        
* **[opciones]**:

        -a               - Numero de angulos.
        -e               - Escribir por la salida estandar los acumuladores del espacio de hough en pgm.


#### Ejemplos

* Obtener la transformada de hough de una imagen de bordes en pgm:

		$ ./hough -a 360 -e < bordes.pgm > acc.pgm

* Capturar de la camara /dev/video0, aplicar canny y, finalmente, hough:

		$ ffmpeg -f video4linux2 -s 1280x720 -i /dev/video0 -f image2pipe -vframes 1 -vcodec ppm pipe:1 | ./canny -l 5 -s 1 -U 100 -L 20 -e | ./hough -a 360 -e > acc.pgm
