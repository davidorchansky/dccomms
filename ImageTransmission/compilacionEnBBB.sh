#Este script sirve:
# - Le dice a make que utilice la biblioteca de compresion compilada para ARM y para 
# - Le pasa a make las opciones de compilacion de la librería de transmision adecuadas para 
#   que funcione correctamente:

make -f makefile.raspi PRIVATELIBDIR=./libraries/arm RARGS="" EXTRAFLAGS="-DRASPI"
