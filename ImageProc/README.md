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
* **img.ppm**: imagen RGB en formato ppm.

#### Ejemplos

* Ejecutar el programa enviandole un ppm por la entrada estandar
*
        $ ./canny -l 5 -s 1 -U 150 -L 50 < noise2.ppm

* Ejecutar el programa enviandole un ppm por la entrada estandar capturado con ffmpeg
*
        $ ffmpeg -f video4linux2 -s 1280x720 -i /dev/video0 -f image2pipe -vframes 1 -vcodec ppm pipe:1 | ./canny -l 5 -s 1 -U 150 -L 50

* Capturar una imagen ppm y redireccionarla por la salida estandar

        $ ffmpeg -f video4linux2 -s 1280x720 -i /dev/video0 -f image2pipe -vframes 1 -vcodec ppm pipe:1 > input.ppm
