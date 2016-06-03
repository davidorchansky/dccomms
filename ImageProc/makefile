

exec=join

REG_W=64
REG_H=16

EXTRARGS=
#ARGS=-DFS_5 -DTIMMING -DTHREADS=2 -DSAVE_STEPS -DFILTRO_NOCACHE -DGRAD_NOCACHE -DNO_DEG_LOOKUPTABLE $(EXTRARGS)

ARGS=-DFS_5 -DTHREADS=4 -DGRAD_NOCACHE -DTIMMING $(EXTRARGS)

CFLAGS= $(ARGS) -fopenmp -O3
CC=gcc
all: $(exec) install

install:
	mkdir -p bin
	mv *.o bin
	mv $(exec) bin	

	
join: join.o
	gcc $< $(CFLAGS) -lm -o $@


clean:
	rm -f *.o $(exec)
	rm -f ./bin/*

join.o: join.c
