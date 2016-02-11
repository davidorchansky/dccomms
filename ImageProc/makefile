
CONSTANTS=UNA

exec=canny hough join

CFLAGS=-D$(CONSTANTS)

CC=gcc

all: $(exec)
canny: Canny.o
	gcc $< -D$(CONSTANTS) -lm -o $@

hough: Hough.o
	gcc $< -D$(CONSTANTS) -lm -o $@

join: join.o
	gcc $< -D$(CONSTANTS) -lm -o $@

clean:
	rm -f *.o $(exec)

Canny.o: Canny.c
Hough.o: Hough.c
join.o: join.c
