#!/bin/bash


#./bin/RX_allInOne -I imagen | ./bin/ppmVisor

./bin/RX_hackrf -I imagen -d -P $3 | ffmpeg -f rawvideo -s $1x$2 -i pipe:0  -f rawvideo -pix_fmt yuv420p pipe:1 | ffplay -f rawvideo -s $1x$2 -i pipe:0 -pix_fmt yuv420p -window_title "OpenROV Camera"





