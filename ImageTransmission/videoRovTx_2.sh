#!/bin/bash

if [ $# -lt 3 ]
then
	echo "Número de argumentos incorrecto: <EncodedImageSize> <MaxPayloadSizeInRadioFrame> <DelayBetweenFrames>" >&2
	exit 1
fi

headersize=24
imsize=0
let imsize=$1+$headersize
paysize=$2
delay=$3


ffmpeg -r 25 -f video4linux2 -s 352x288 -i /dev/video0 -pix_fmt yuv420p -f rawvideo pipe:1 | ./bin/header 352 288 | ./bin/encode 1 1 1 0 imagen | ./bin/TX2 imagen $imsize $paysize $delay


