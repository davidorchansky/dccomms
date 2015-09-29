#!/bin/bash

if [ $# -lt 4 ]
then
	echo "Número de argumentos incorrecto: <EncodedImageSize> <MaxPayloadSizeInRadioFrame> <DelayBetweenFrames> <MaxFrameAgeInMilliseconds>" >&2
	exit 1
fi

headersize=0
imsize=0
let imsize=$1+$headersize
paysize=$2
delay=$3
maxAge=$4


ffmpeg -r 25 -f video4linux2 -s 1280x720 -i /dev/video0 -pix_fmt yuv420p -f rawvideo pipe:1 | ./bin/header_v2 1280 720 | ./bin/debter -I imagen | ./bin/TX3 imagen $imsize $paysize $delay $maxAge


