
CONSTANTS=UNA
all: canny hough

canny: Canny.o
	gcc $< -D$(CONSTANTS) -lm -o $@

hough: Hough.o
	gcc $< -D$(CONSTANTS) -lm -o $@

Canny.o: Canny.c
Hough.o: Hough.c
