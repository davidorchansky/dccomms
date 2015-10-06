#!/bin/bash

#if [ $# -lt 1 ]
#then
#	echo "Número de argumentos incorrecto: <EncodedImageSize>" >&2
#	exit 1
#fi
#
#imsize=$1


./bin/RX2 imagen 90000 | ./bin/debter -d -I imagen | ffmpeg -f rawvideo -s 1280x720 -i pipe:0 -f mjpeg  pipe:1
