OBJS = TX RX
LIBDIR = ../Release
RARGS = 

all: radio TX RX
radio:
	make -C $(LIBDIR) -f radio.mk RADIOFLAGS="$(RARGS)"

TX: radio
	g++ -std=c++0x -I"../includes/" -L $(LIBDIR)  TX.cpp -o $@ -lradio -lcrypto

RX: radio
	g++ -std=c++0x -I"../includes/" -L $(LIBDIR)  RX.cpp -o $@ -lradio -lcrypto

clean:
	rm -f test *.o $(OBJS)
	make -C $(LIBDIR) clean

