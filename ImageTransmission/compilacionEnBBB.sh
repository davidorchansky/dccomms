#Este script sirve:
# - Le dice a make que utilice la biblioteca de compresion compilada para ARM y para 
# - Le pasa a make las opciones de compilacion de la librería de transmision adecuadas para 
#   que funcione correctamente. Esas opciones son:
#   - BBB: Indica que compilamos una BeagleBoneBlack. Así, se chequearan tambien los ficheros /dev/ttyO[X]
#   - BADWRITE: Indica que se escriba en el puerto serie 1 byte por llamada a la función POSIX write.
#   - BADWRITE2=X: Indica que cada llamada a la función de POSIX write escriba un maximo de X bytes.
#	Nota: Parece que el SO tiene problemas al cuando se escriben demasiados bytes a la vez por el puerto Serie con write... 
#	      Por eso lo de las opciones "BADWRITE". Esto no es correcto, hay que investigar por que pasa y arreglarlo mejor.

make PRIVATELIBDIR=./libraries/arm RARGS=""
