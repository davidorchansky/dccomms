#!/bin/bash

if [ $# -lt 7 ]
then
	echo "Número de argumentos incorrecto: <ImageWidth> <ImageHeight> <EncodedImageSize> <MaxPayloadSizeInRadioFrame> <DelayBetweenFrames> <MaxImages> <delayIni>" >&2
	exit 1
fi

width=$1
height=$2
headersize=0
imsize=0
let imsize=$3+$headersize
paysize=$4
delay=$5
maxImages=$6
iniDelay=$7

echo ESPERANDO $iniDelay SEGUNDOS...
sleep $iniDelay

echo INICIANDO TRANSMISION...

./bin/test -W $width -H $height -I imagen -F $imsize -L $paysize -D $delay -A 1000 -N $maxImages

