#!/bin/bash

DBASE=res/cache/v

WIDTH=120
HEIGHT=16
cd ..

make join ARGS="-mfpu=neon -DNEON -DNEON_VF -DTIMMING -DTHREADS=4 -DFS_5 -DREG_W=$WIDTH -DREG_H=$HEIGHT -DNORESULT"
cd - 
DIR=$DBASE$WIDTH\_$HEIGHT
echo $DIR
mkdir -p $DIR
./compute-times.sh set-0/ $DIR ../bin/join

