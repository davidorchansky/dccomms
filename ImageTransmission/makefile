LIBDIR = ../Release
RARGS = 
BINDIR = bin/
all: radio $(BINDIR)TX $(BINDIR)RX $(BINDIR)RX1 $(BINDIR)RX2

radio:
	make -C $(LIBDIR) -f radio.mk RADIOFLAGS="$(RARGS)"

$(BINDIR)TX: radio
	mkdir -p $(BINDIR)
	g++ -std=c++0x -I"../includes/" -L $(LIBDIR)  TX.cpp -o $@ -lradio -lcrypto

$(BINDIR)RX: radio
	mkdir -p $(BINDIR)
	g++ -std=c++0x -I"../includes/" -L $(LIBDIR)  RX.cpp -o $@ -lradio -lcrypto

$(BINDIR)RX1: radio
	mkdir -p $(BINDIR)
	g++ -std=c++0x -I"../includes/" -L $(LIBDIR)  RX1.cpp -o $@ -lradio -lcrypto

$(BINDIR)RX2: radio
	mkdir -p $(BINDIR)
	g++ -std=c++0x -I"../includes/" -L $(LIBDIR)  RX2.cpp -o $@ -lradio -lcrypto


clean:
	rm -f $(BINDIR)/*
	make -C $(LIBDIR) clean

