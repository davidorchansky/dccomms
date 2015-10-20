#!/bin/bash

if [ $# -lt 2 ]
then
	echo "Número de argumentos incorrecto: <ImageWidth> <ImageHeight>" >&2
	exit 1
fi

width=$1
heigth=$2


./bin/RX imagen 90000 | ./bin/debter -d -I imagen | ffmpeg -f rawvideo -s $1x$2 -i pipe:0 -f mjpeg  pipe:1
