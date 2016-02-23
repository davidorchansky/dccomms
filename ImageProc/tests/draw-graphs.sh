#!/bin/bash

if [ $# -lt 1 ]
then
	echo "Numero de argumentos incorrecto"
	exit 1
fi

function getTiempoPaso
{
	reg="$1-[^0-9]*([1-9][0-9]*)"
	if [[ $2 =~ $reg ]]; then
		echo ${BASH_REMATCH[1]}
	fi
}

function getLastNumber
{
	reg="[^0-9]*([1-9][0-9]*)"
	if [[ $1 =~ $reg ]]; then
		echo ${BASH_REMATCH[1]}
	fi
}


rm -rf $expdir
for i in $1/*
do
	echo Tratando imagen $i...
	for per in $(ls $i); do
		perdir=$i/$per
		for filterSize in $(ls $perdir); do
			filterdir=$perdir/$filterSize
			if [ -d $filterdir ]; then
			for angles in $(ls $filterdir); do
				anglesdir=$filterdir/$angles
				for ratio in $(ls $anglesdir); do
					ratiodir=$anglesdir/$ratio
					for lthold in $(ls $ratiodir); do
						ltholddir=$ratiodir/$lthold
						for sigma in $(ls $ltholddir); do
							sigmadir=$ltholddir/$sigma
							lthold=$(getLastNumber $lthold)
							sigma=$(getLastNumber $sigma)
							angles=$(getLastNumber $angles)
							filterSize=$(getLastNumber $filterSize)
							echo "Obteniendo grafica de $sigmadir..."

							plotDataFile=$sigmadir/plotDataFile.txt
							histogramFile=$sigmadir/tiempos.pdf
							uthold=$(echo "$lthold * $ratio" | bc)
 							gnuplot -e "inputfile=\"$plotDataFile\"; imagen=\"$(basename $i)-$per\\%\"; sigma=$sigma; tfiltro=$filterSize; ut=$uthold; lt=$lthold; angles=$angles; outputfile=\"$histogramFile\"" plotData
				
						done

					done
				done
			done
			fi
		done
	done
done
