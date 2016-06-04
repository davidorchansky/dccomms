#!/bin/bash

DBASE=res/cache/v

WIDTH=64
HEIGHT=31
cd ..
make EXTRARGS="-mfpu=neon -DNEON -DNEON_VF -DTIMMING -DREG_W=200 -DREG_H=16 -DNORESULT"
cd - 
DIR=$DBASE$WIDTH\_$HEIGHT
echo $DIR
mkdir -p $DIR
./compute-times.sh set-0/ $DIR ../bin/join

