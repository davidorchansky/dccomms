
ARGS=

exec=pruebas join pruebas-pg

CFLAGS=$(ARGS) -fopenmp -O3
CC=gcc
all: $(exec) install

install:
	mkdir -p bin
	mv *.o bin
	mv $(exec) bin	

	
pruebas: pruebas.o
	gcc $< $(ARGS) $(CFLAGS) -lm -o $@
join: join.o
	gcc $< $(ARGS) $(CFLAGS) -lm -o $@

pruebas-pg: pruebas-pg.o
	gcc $< $(ARGS) -fopenmp -lm -o $@ -pg


clean:
	rm -f *.o $(exec)
	rm -f ./bin/*

pruebas.o: pruebas.c
join.o: join.c
pruebas-pg.o: pruebas.c
	gcc -c $< $(ARGS) -fopenmp -lm -o $@ -pg
