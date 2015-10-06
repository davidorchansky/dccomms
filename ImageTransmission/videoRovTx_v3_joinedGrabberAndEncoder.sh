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


#./bin/grabber 352 288 | 

./bin/grabberDebter -I imagen | ./bin/TX imagen $imsize $paysize $delay $maxAge


