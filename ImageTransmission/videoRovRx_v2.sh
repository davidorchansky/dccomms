#!/bin/bash

#if [ $# -lt 1 ]
#then
#	echo "Número de argumentos incorrecto: <EncodedImageSize>" >&2
#	exit 1
#fi
#
#imsize=$1


./bin/RX2 imagen 90000 | ./bin/debter -d -I imagen | ffmpeg -f rawvideo -s 352x288 -i pipe:0  -f rawvideo -pix_fmt yuv420p pipe:1 | ffplay -f rawvideo -s 352x288 -i pipe:0 -pix_fmt yuv420p -window_title "OpenROV Camera"


