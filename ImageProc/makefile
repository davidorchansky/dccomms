
ARGS=

exec=canny hough join arm-join-v1

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



clean:
	rm -f *.o $(exec)

Canny.o: Canny.c
Hough.o: Hough.c
join.o: join.c
arm-join-v1.o: arm-join-v1.c
