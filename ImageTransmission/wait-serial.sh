#!/bin/bash

while [ \( ! -e /dev/ttyACM0 \) -a \( ! -e /dev/ttyACM1 \) ]
do
	true
done

echo "Done!"
