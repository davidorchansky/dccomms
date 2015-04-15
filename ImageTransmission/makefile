LIBDIR = ../Release
RARGS = 
BINDIR = bin/
PRIVATELIBDIR = libraries
INCLUDES = includes
EDULIB = ezbtx86_64
all: bindir radio improc $(BINDIR)TX3 $(BINDIR)TX $(BINDIR)RX $(BINDIR)RX1 $(BINDIR)RX2 $(BINDIR)TX2 $(BINDIR)encode $(BINDIR)decode $(BINDIR)header $(BINDIR)encode_2

bindir:
	mkdir -p $(BINDIR)

radio:
	make -C $(LIBDIR) -f radio.mk RADIOFLAGS="$(RARGS)"

improc:
	sudo cp ./libraries/lib$(EDULIB).so /usr/lib
	sudo ldconfig

$(BINDIR)TX: TX.cpp
	g++ -std=c++0x -I"../includes/" -L $(LIBDIR)  $< -o $@ -lradio -lcrypto

$(BINDIR)TX2: TX2.cpp
	g++ -std=c++0x -I"../includes/" -L $(LIBDIR)  $< -o $@ -lradio -lcrypto

$(BINDIR)TX3: TX3.cpp
	g++ -std=c++0x -I"../includes/" -L $(LIBDIR)  $< -o $@ -lradio -lcrypto

$(BINDIR)RX: RX.cpp
	g++ -std=c++0x -I"../includes/" -L $(LIBDIR)  $< -o $@ -lradio -lcrypto

$(BINDIR)RX1: RX1.cpp
	g++ -std=c++0x -I"../includes/" -L $(LIBDIR)  $< -o $@ -lradio -lcrypto

$(BINDIR)RX2: RX2.cpp
	g++ -std=c++0x -I"../includes/" -L $(LIBDIR)  $< -o $@ -lradio -lcrypto

$(BINDIR)encode: encode.c colorconv.c
	gcc -I $(INCLUDES) $^ -o $@ -l$(EDULIB) -lm

$(BINDIR)encode_2: encode_2.c colorconv.c
	gcc -I $(INCLUDES) $^ -o $@ -l$(EDULIB) -lm


$(BINDIR)decode: decode.c
	gcc -I $(INCLUDES) $< -o $@ -l$(EDULIB) -lm

$(BINDIR)header: header.c
	gcc -I $(INCLUDES) $< -o $@ 


clean:
	rm -f $(BINDIR)/*
	make -C $(LIBDIR) clean

