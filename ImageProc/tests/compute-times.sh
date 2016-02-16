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
rm -rf $expdir
for i in $1/*
do
	echo Tratando imagen $i
	
	imgname=$(basename $i)
	echo $imgname
	imgnameNoext=$(basename $imgname $inputImgExt)
	echo $imgnameNoext
	imgdir=$expdir/$imgnameNoext
	echo $imgdir
	mkdir -p $imgdir

	#Experimentamos con tamanos 25%, 50%, 75%, y 100%

	per=25
	while [ $per -le 100 ]; do
		sizeDir=$imgdir/$per
		mkdir -p $sizeDir
		inputImg=$sizeDir/$imgnameNoext\_$per$inputImgExt
		convert $i -resize $per% $inputImg

		sigma=1
		filterSize=5

		$tester -s $sigma -l $filterSize -d 

		let per=per+25 
	done
done
