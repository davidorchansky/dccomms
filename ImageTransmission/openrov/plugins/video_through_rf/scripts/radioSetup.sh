#!/bin/bash

basedir=/home/rov/ROVRadioTransmission/ImageTransmission/

/etc/init.d/openrov stop
python $basedir/radioSetup.py &



