LIBDIR = ../Release
RARGS = 
BINDIR = bin/
PRIVATELIBDIR = ./libraries/x86_64
INCLUDES = includes 
all: bindir radio $(BINDIR)control

CC = gcc
FLAGS = -O3

bindir:
	mkdir -p $(BINDIR)

radio:
	make -C $(LIBDIR) -f radio.mk RADIOFLAGS="$(RARGS)"


$(BINDIR)control: control.cpp
	$(CXX) $(FLAGS) -std=c++0x -I"../includes/" -L$(LIBDIR) $< -o $@ -lradio -lcrypto

clean:
	rm -f $(BINDIR)/*
	make -C $(LIBDIR) clean

