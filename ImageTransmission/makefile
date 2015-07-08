LIBDIR = ../Release
RARGS = 
BINDIR = bin/
PRIVATELIBDIR = ./libraries/x86_64
INCLUDES = includes 
EDULIB_V2_INCLUDES = libraries/debt/src
EDULIB = ezbt
EDULIB_V2 = debt
all: bindir radio improc $(BINDIR)TX3 $(BINDIR)TX $(BINDIR)RX $(BINDIR)RX1 $(BINDIR)RX2 $(BINDIR)TX2 $(BINDIR)encode $(BINDIR)decode $(BINDIR)header $(BINDIR)header_v2 $(BINDIR)encode_2 $(BINDIR)debter $(BINDIR)grabber

bindir:
	mkdir -p $(BINDIR)

radio:
	make -C $(LIBDIR) -f radio.mk RADIOFLAGS="$(RARGS)"

improc:
	sudo cp $(PRIVATELIBDIR)/lib$(EDULIB).so /usr/lib
	sudo cp $(PRIVATELIBDIR)/lib$(EDULIB_V2).so /usr/lib
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

$(BINDIR)header_v2: header_v2.c
	gcc -I $(INCLUDES) $< -o $@ 

$(BINDIR)grabber: grabber.c
	gcc -I $(INCLUDES) $< -o $@ 


$(BINDIR)debter: debt.c
	gcc -I $(INCLUDES) -I $(EDULIB_V2_INCLUDES) -L $(PRIVATELIBDIR) $< -o $@ -l$(EDULIB_V2) -lm

#gcc -L ../x86_64/static-libraries/ -I ../src/ debt.c -ldebt -lm -o debter
clean:
	rm -f $(BINDIR)/*
	make -C $(LIBDIR) clean

