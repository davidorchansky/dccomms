#!/bin/bash

if [ $# -lt 3 ]
then
	echo "Numero de argumentos incorrecto"
	exit 1
fi

contador=0
expdir=$2
inputImgExt=.ppm
tester=$3
imgSizes=(100)
filterSizes=(5)
degrees=(360)
lowThresholds=(40)
ratios=(3)
sigmas=(1)
#schedules=("static")
schedules=("static dynamic guided auto")
chunk_sizes=(1 2 4 300 400 482 500, 600 700 900)
#chunk_sizes=(1 2 4 241 482 500 700 1000)
iterations=10

function getTiempoPaso
{
	reg="$1-[^0-9]*([1-9][0-9]*)"
	if [[ $2 =~ $reg ]]; then
		echo ${BASH_REMATCH[1]}
	fi
}


rm -rf $expdir
for i in $1/*
do
	echo Tratando imagen $i
	
	imgname=$(basename $i)
	imgnameNoext=$(basename $imgname $inputImgExt)
	imgdir=$expdir/$imgnameNoext
	mkdir -p $imgdir

	#Experimentamos con tamanos 25%, 50%, 75%, y 100%

	for per in ${imgSizes[*]}; do

		sizeDir=$imgdir/resized$per
		mkdir -p $sizeDir
		inputImg=$sizeDir/$imgnameNoext\_$per$inputImgExt
		convert $i -resize $per% $inputImg

		for filterSize in ${filterSizes[*]}; do

			filterSizeDir=$sizeDir/filterSize$filterSize
			#mkdir -p $filterSizeDir

			for angles in ${degrees[*]}; do

				anglesDir=$filterSizeDir/degrees$angles
				#mkdir -p $anglesDir

				for ratio in ${ratios[*]}; do
					ratioDir=$anglesDir/ratio$ratio
					#mkdir -p $ratioDir

					for lthold in ${lowThresholds[*]}; do
						ltholdDir=$ratioDir/lowThreshold$lthold
						#mkdir -p $ltholdDir
						for chunk in ${chunk_sizes[*]}; do
							chunkDir=$ltholdDir/chunkSize$chunk
							for schedule in ${schedules[*]}; do
								scheduleDir=$chunkDir/schedule$schedule
								sigmaDir=$scheduleDir
								sigma=1
								mkdir -p $sigmaDir
							
								uthold=$(echo "$lthold * $ratio" | bc)


								it=0
								t01=0; t02=0; t03=0;
								t04=0; t05=0; t06=0;
								t07=0; t08=0; t09=0;

								plotDataFile=$sigmaDir/plotDataFile.txt
								tiempoTotalFile=$sigmaDir/tiempoTotal.txt
								histogramFile=$sigmaDir/tiempos.pdf
								while [ $it -lt $iterations ]; do

									rawOutputFile=$sigmaDir/rawOutput_$it.txt
									paralconf="\"$schedule,$chunk\""
									echo $paralconf
									export OMP_SCHEDULE=$paralconf
									$tester -a $angles -U $uthold -L $lthold -s $sigma -l $filterSize -d $sigmaDir < $inputImg 2> $rawOutputFile
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

								rm -f $plotDataFile

								ut01=$(echo "$t01 / $iterations" | bc)
								ut02=$(echo "$t02 / $iterations" | bc)
								ut03=$(echo "$t03 / $iterations" | bc)
								ut04=$(echo "$t04 / $iterations" | bc)
								ut05=$(echo "$t05 / $iterations" | bc)
								ut06=$(echo "$t06 / $iterations" | bc)
								ut07=$(echo "$t07 / $iterations" | bc)
								ut08=$(echo "$t08 / $iterations" | bc)
								ut09=$(echo "$t09 / $iterations" | bc)

								e01=0; e02=0; e03=0; e04=0; e05=0; e06=0; e07=0; e08=0; e09=0

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
								for rawOutputFile in $sigmaDir/rawOutput_*.txt; do

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

								echo -e "Filtrado \t $ut01 \t $e01 \t 0" >> $plotDataFile
								echo -e "GradienteX \t $ut02 \t $e02 \t 1" >> $plotDataFile
								echo -e "GradienteY \t $ut03 \t $e03 \t 2" >> $plotDataFile
								echo -e "ModuloGrad \t $ut04 \t $e04 \t 3" >> $plotDataFile
								echo -e "DirGrad \t $ut05 \t $e05 \t 4" >> $plotDataFile
								echo -e "DirGradDis \t $ut06 \t $e06 \t 5" >> $plotDataFile
								echo -e "SupNoMax \t $ut07 \t $e07 \t 6" >> $plotDataFile
								echo -e "Umbralizacion \t $ut08 \t $e08 \t 7" >> $plotDataFile
								echo -e "HoughSpace \t $ut09 \t $e09 \t 8" >> $plotDataFile


								#echo "inputfile=\"$plotDataFile\"; imagen=\"$imgnameNoext\"; sigma=$sigma; tfiltro=$filterSize; ut=$uthold; lt=$lthold; angles=$angles; outputfile=\"comparativa.pdf\""
								
								let ttotal=ut01+ut02+ut03+ut04+ut05+ut06+ut07+ut08+ut09
								echo "TIEMPO TOTAL: $ttotal" > $tiempoTotalFile
								
								a=($ut01 $ut02 $ut03 $ut04 $ut05 $ut06 $ut07 $ut08 $ut09)

								maxv=0
								for v in ${a[@]}; do
								if (( $v > $maxv )); then maxv=$v; fi; 
								done

							 #	gnuplot -e "maxv=$maxv; ttotal=$ttotal; inputfile=\"$plotDataFile\"; imagen=\"$(basename $imgnameNoext)-$per\\%\"; sigma=$sigma; tfiltro=$filterSize; ut=$uthold; lt=$lthold; angles=$angles; outputfile=\"$histogramFile\"" plotData
								

							done
						done

					done
				done
			done
		done
	done
done
