#!/bin/bash

if [ $# -lt 4 ]
then
	echo "Número de argumentos incorrecto: <EncodedImageSize> <MaxPayloadSizeInRadioFrame> <DelayBetweenFrames> <MaxFrameAgeInMilliseconds>" >&2
	exit 1
fi

headersize=24
imsize=0
let imsize=$1+$headersize
paysize=$2
delay=$3
maxAge=$4

#ffmpeg -i cut.mp4  -filter:v "crop=210:205:20:37" out.mp4
#ffmpeg -i iros.mp4 -vf trim=4:29 cut0.mp4 -y
#ffmpeg -i iros.mp4 -vf trim=36:58 cut1.mp4 -y

#ffmpeg -f concat -i mylist.txt -c copy def.mp4

{ ffplay -rtsp_flags listen -analyzeduration 0.1 rtsp://localhost:8888/live.sdp?udp; } &

sleep 3s

ffmpeg -re -f mp4 -i ~/def.mp4 -f rawvideo -s 352x288 -pix_fmt yuv420p - \
 -s 352x288 -f rtsp -rtsp_transport udp rtsp://localhost:8888/live.sdp  | ./bin/header 352 288 | ./bin/encode_2 1 1 1 0 imagen | ./bin/TX3 imagen $imsize $paysize $delay $maxAge


