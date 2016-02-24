
ARGS=

exec=canny hough join arm-join-v1 arm-join-v2 arm-join-v2-opt pruebas

CFLAGS=$(ARGS)

CC=gcc
all: $(exec) install

install:
	mkdir -p bin
	mv *.o bin
	mv $(exec) bin	

	
canny: Canny.o
	gcc $< $(ARGS) -lm -o $@

hough: Hough.o
	gcc $< $(ARGS) -lm -o $@

join: join.o
	gcc $< $(ARGS) -lm -o $@

arm-join-v1: arm-join-v1.o
	gcc $< $(ARGS) -lm -o $@

arm-join-v2: arm-join-v2.o
	gcc $< $(ARGS) -lm -o $@

arm-join-v2-opt: arm-join-v2-opt.o
	gcc $< $(ARGS) -O3 -lm -o $@
pruebas: pruebas.o
	gcc $< $(ARGS) -lm -o $@



clean:
	rm -f *.o $(exec)
	rm -f ./bin/*

Canny.o: Canny.c
Hough.o: Hough.c
join.o: join.c
arm-join-v1.o: arm-join-v1.c
arm-join-v2.o: arm-join-v2.c
arm-join-v2-opt.o: arm-join-v2.c
	gcc $< $(CFLAGS) -lm -O3 -c -o arm-join-v2-opt.o
pruebas.o: pruebas.c
