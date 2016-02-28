
ARGS=

exec=pruebas

CFLAGS=$(ARGS) -fopenmp -O3
CC=gcc
all: $(exec) install

install:
	mkdir -p bin
	mv *.o bin
	mv $(exec) bin	

	
pruebas: pruebas.o
	gcc $< $(ARGS) $(CFLAGS) -lm -o $@



clean:
	rm -f *.o $(exec)
	rm -f ./bin/*

pruebas.o: pruebas.c
