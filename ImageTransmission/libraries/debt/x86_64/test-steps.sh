#!/bin/bash

dir=tmp

mkdir -p $dir
 
#get an image from the cammera in yuv420 format
ffmpeg -f video4linux2 -s 352x288 -i /dev/video0 -pix_fmt yuv420p -vframes 1 -f rawvideo pipe:1  > $dir/im.raw; echo -e "P7\n352 288\n255" > $dir/input; cat $dir/im.raw >> $dir/input

#encode and trunk 1000 bytes:
./debter -x 1000 < $dir/input > $dir/encoded  

#decode only 500 bytes from these 1000
./debter -d -c 500 < $dir/encoded > $dir/decoded

#delete the header
dd if="$dir/decoded" of="$dir/result.raw" bs=1 skip=15

#convert to png
ffmpeg -pix_fmt yuv420p -s 352x288 -i $dir/result.raw salida.png -y
