#!/bin/bash

if [ $# -lt 6 ]
then
	echo "Número de argumentos incorrecto: <ImageWidth> <ImageHeight> <EncodedImageSize> <MaxPayloadSizeInRadioFrame> <DelayBetweenFrames> <MaxFrameAgeInMilliseconds>" >&2
	exit 1
fi

width=$1
height=$2
headersize=0
imsize=0
let imsize=$3+$headersize
paysize=$4
delay=$5
maxAge=$6


#./bin/grabber 352 288 | 

./bin/grabberDebter -I imagen -W $width -H $height | ./bin/TX imagen $imsize $paysize $delay $maxAge


