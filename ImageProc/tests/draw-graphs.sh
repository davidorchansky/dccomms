#!/bin/bash

if [ $# -lt 2 ]
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

function getScheduleName
{
	reg="schedule([^0-9]+)$"
	if [[ $1 =~ $reg ]]; then
		echo ${BASH_REMATCH[1]}
	fi
}

arch=$2



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
						conf=0
						paralconf=$ltholddir/plotData.txt
						comp=$ltholddir/comparative.pdf
						rm -f $paralconf $comp
						for chunk in $(ls $ltholddir); do
							chunkDir=$ltholddir/$chunk
							for schedule in $(ls $chunkDir); do
								scheduleDir=$chunkDir/$schedule
								schedule=$(getScheduleName $schedule)
								chunk=$(getLastNumber $chunk)
								sigmadir=$scheduleDir
								lthold=$(getLastNumber $lthold)
								sigma=1
								angles=$(getLastNumber $angles)
								filterSize=$(getLastNumber $filterSize)
								echo "Obteniendo grafica de $sigmadir..."
								plotDataFile=$sigmadir/plotDataFile.txt
								histogramFile=$sigmadir/tiempos.pdf
								ratio=$(getLastNumber $ratio)
								uthold=$(echo "$lthold * $ratio" | bc)
								it=0
								t01=0; t02=0; t03=0;
								t04=0; t05=0; t06=0;
								t07=0; t08=0; t09=0;

								for rawOutputFile in $sigmadir/rawOutput_*.txt; do
									rawOutput=$(cat $rawOutputFile)

									t01tmp=$(getTiempoPaso 01 "$rawOutput")
									t02tmp=$(getTiempoPaso 02 "$rawOutput")
									t03tmp=$(getTiempoPaso 03 "$rawOutput")
									t04tmp=$(getTiempoPaso 04 "$rawOutput")
									t05tmp=$(getTiempoPaso 05 "$rawOutput")
									t06tmp=$(getTiempoPaso 06 "$rawOutput")
									t07tmp=$(getTiempoPaso 07 "$rawOutput")
									t08tmp=$(getTiempoPaso 08 "$rawOutput")
									t09tmp=$(getTiempoPaso 09 "$rawOutput")

									let t01=t01+t01tmp
									let t02=t02+t02tmp
									let t03=t03+t03tmp
									let t04=t04+t04tmp
									let t05=t05+t05tmp
									let t06=t06+t06tmp
									let t07=t07+t07tmp
									let t08=t08+t08tmp
									let t09=t09+t09tmp

									let it=it+1
								done
								iterations=$it
								ut01=$(echo "$t01 / $iterations" | bc)
								ut02=$(echo "$t02 / $iterations" | bc)
								ut03=$(echo "$t03 / $iterations" | bc)
								ut04=$(echo "$t04 / $iterations" | bc)
								ut05=$(echo "$t05 / $iterations" | bc)
								ut06=$(echo "$t06 / $iterations" | bc)
								ut07=$(echo "$t07 / $iterations" | bc)
								ut08=$(echo "$t08 / $iterations" | bc)
								ut09=$(echo "$t09 / $iterations" | bc)

								#calculamos la desviacion tipica

								dt01=0
								dt02=0
								dt03=0
								dt04=0
								dt05=0
								dt06=0
								dt07=0
								dt08=0
								dt09=0
								for rawOutputFile in $sigmadir/rawOutput_*.txt; do

									rawOutput=$(cat $rawOutputFile)

									t01tmp=$(getTiempoPaso 01 "$rawOutput")
									t02tmp=$(getTiempoPaso 02 "$rawOutput")
									t03tmp=$(getTiempoPaso 03 "$rawOutput")
									t04tmp=$(getTiempoPaso 04 "$rawOutput")
									t05tmp=$(getTiempoPaso 05 "$rawOutput")
									t06tmp=$(getTiempoPaso 06 "$rawOutput")
									t07tmp=$(getTiempoPaso 07 "$rawOutput")
									t08tmp=$(getTiempoPaso 08 "$rawOutput")
									t09tmp=$(getTiempoPaso 09 "$rawOutput")

									dt01=$(echo "$dt01 + ($t01tmp-$ut01)^2" | bc -l)
									dt02=$(echo "$dt02 + ($t02tmp-$ut02)^2" | bc -l)
									dt03=$(echo "$dt03 + ($t03tmp-$ut03)^2" | bc -l)
									dt04=$(echo "$dt04 + ($t04tmp-$ut04)^2" | bc -l)
									dt05=$(echo "$dt05 + ($t05tmp-$ut05)^2" | bc -l)
									dt06=$(echo "$dt06 + ($t06tmp-$ut06)^2" | bc -l)
									dt07=$(echo "$dt07 + ($t07tmp-$ut07)^2" | bc -l)
									dt08=$(echo "$dt08 + ($t08tmp-$ut08)^2" | bc -l)
									dt09=$(echo "$dt09 + ($t09tmp-$ut09)^2" | bc -l)
								done

								e01=$(echo "sqrt($dt01 / $iterations)" | bc -l)
								e02=$(echo "sqrt($dt02 / $iterations)" | bc -l)
								e03=$(echo "sqrt($dt03 / $iterations)" | bc -l)
								e04=$(echo "sqrt($dt04 / $iterations)" | bc -l)
								e05=$(echo "sqrt($dt05 / $iterations)" | bc -l)
								e06=$(echo "sqrt($dt06 / $iterations)" | bc -l)
								e07=$(echo "sqrt($dt07 / $iterations)" | bc -l)
								e08=$(echo "sqrt($dt08 / $iterations)" | bc -l)
								e09=$(echo "sqrt($dt09 / $iterations)" | bc -l)

								echo -e "Filtrado \t $ut01 \t $e01 \t 0" > $plotDataFile
								echo -e "GradienteX \t $ut02 \t $e02 \t 1" >> $plotDataFile
								echo -e "GradienteY \t $ut03 \t $e03 \t 2" >> $plotDataFile
								echo -e "ModuloGrad \t $ut04 \t $e04 \t 3" >> $plotDataFile
								echo -e "DirGrad \t $ut05 \t $e05 \t 4" >> $plotDataFile
								echo -e "DirGradDis \t $ut06 \t $e06 \t 5" >> $plotDataFile
								echo -e "SupNoMax \t $ut07 \t $e07 \t 6" >> $plotDataFile
								echo -e "Umbralizacion \t $ut08 \t $e08 \t 7" >> $plotDataFile
								echo -e "HoughSpace \t $ut09 \t $e09 \t 8" >> $plotDataFile

								let ttotal=ut01+ut02+ut03+ut04+ut05+ut06+ut07+ut08+ut09
								a=($ut01 $ut02 $ut03 $ut04 $ut05 $ut06 $ut07 $ut08 $ut09)

								maxv=0
								for v in ${a[@]}; do
								if (( $v > $maxv )); then maxv=$v; fi; 
								done

								gnuplot -e "maxv=$maxv; ttotal=$ttotal; inputfile=\"$plotDataFile\"; imagen=\"$(basename $i)-$per\\%\"; sigma=$sigma; tfiltro=$filterSize; ut=$uthold; lt=$lthold; angles=$angles; outputfile=\"$histogramFile\"; arch=\"$arch\"; schedule=\"$schedule\"; chunkSize=$chunk" plotData
								
							
							
							echo -e "$schedule-$chunk\t$ttotal\t0\t$conf" >> $paralconf
							let conf=conf+1
							done
						done
						gnuplot -e "maxv=$maxv; ttotal=$ttotal; inputfile=\"$paralconf\"; imagen=\"$(basename $i)-$per\\%\"; sigma=1; tfiltro=$filterSize; ut=$uthold; lt=$lthold; angles=$angles; outputfile=\"$comp\"; arch=\"$arch\";" plotData2


					done
				done
			done
			fi
		done
	done
							
done
