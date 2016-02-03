#!/bin/bash

path=/home/diego/ROVRadioTransmission/ImageTransmission/

cd $path

while [ \( ! -e /dev/ttyACM0 \) -a \( ! -e /dev/ttyACM1 \) ]
do
	true
done

sleep 10

#./videoRovTx_v3_joinedGrabberAndEncoder.sh 1280 720 430 1400 0 10

#./bin/videoServer_2procOnly 1280 720 430 1500 0

#./bin/TX_allInOne_noServer -W 1280 -H 720 -I imagen -F 15000 -L 1450 -D 1500 -A 1000

#./bin/TX_allInOne_noServer -W 1280 -H 720 -I imagen -F 430 -L 1450 -D 0 -A 1000

#./bin/TX_allInOne_noServer -W 1280 -H 720 -I imagen -F 1440 -L 1480 -D 0 -A 1000

#./bin/TX_allInOne_noServer -W 1280 -H 720 -I imagen -F 1440 -L 1480 -D 0 -A 1000

#./bin/TX_allInOne_noServer -W 1280 -H 720 -I imagen -F 200 -L 1480 -D 10000 -A 1000


./bin/TX_allInOne_noServer -W 1280 -H 720 -I imagen -F 500 -L 1480 -D 0 -A 1000

cd -
