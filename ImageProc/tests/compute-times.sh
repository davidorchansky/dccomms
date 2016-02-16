#!/bin/bash

if [ $# -lt 1 ]
then
	echo "Numero de argumentos incorrecto"
	exit 1
fi

echo $1

#output=$(./test.sh 2>&1 | cat)


contador=0

expdir=experimentacion/
inputImgExt=.ppm
tester=../arm-join-v1
imgSizes=(25 50 75 100)
filterSizes=(3 7 11 15)
angles=(100 400 700 1000)
lowThresholds=(25 35 40 50 60)
ratios=(3)
sigmas=(1)

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

			for angles in ${angles[*]}; do

				anglesDir=$filterSizeDir/angles$angles
				#mkdir -p $anglesDir

				for ratio in ${ratios[*]}; do
					ratioDir=$anglesDir/ratio$ratio
					#mkdir -p $ratioDir

					for lthold in ${lowThresholds[*]}; do
						ltholdDir=$ratioDir/lowThreshold$lthold
						#mkdir -p $ltholdDir
						
						for sigma in ${sigmas[*]}; do
							sigmaDir=$ltholdDir/sigma$sigma
							mkdir -p $sigmaDir
						
							rawOutputFile=$sigmaDir/rawOutput.txt
							uthold=$(echo "$lthold * $ratio" | bc)
							$tester -a $angles -U $uthold -L $lthold -s $sigma -l $filterSize -d $sigmaDir < $inputImg 2> $rawOutputFile
						done

					done
				done
			done
		done
	done
done
