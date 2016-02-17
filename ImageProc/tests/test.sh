#!/bin/bash
dir=../
##./arm-join-v1 -a 1000 -l 5 -s 1 -U 150 -L 50 -e < noise2.ppm > acc.pgm

ffmpeg -f video4linux2 -s 1280x720 -i /dev/video0 -f image2pipe -vframes 1 -vcodec ppm pipe:1 | $dir/arm-join-v1 -a 1000 -l 5 -s 0.5 -U 50 -L 20 -e > acc.pgm
