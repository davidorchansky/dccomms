#!/bin/bash

if [ $# -lt 6 ]
then
	echo "Número de argumentos incorrecto: <ImageWidth> <ImageHeight> <EncodedImageSize> <MaxPayloadSizeInRadioFrame> <DelayBetweenFrames> <MaxFrameAgeInMilliseconds>" >&2
	exit 1
fi

headersize=0
imsize=0
let imsize=$3+$headersize
paysize=$4
delay=$5
maxAge=$6


./bin/grabber $1 $2 | ./bin/debter -I imagen | ./bin/TX imagen $imsize $paysize $delay $maxAge


