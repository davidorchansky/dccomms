LIBDIR = ../Release
RARGS = 
BINDIR = bin/
PRIVATELIBDIR = ./libraries/x86_64
INCLUDES = includes 
EDULIB_V2_INCLUDES = libraries/debt/src
EDULIB_V2 = debt
all: bindir radio improc $(BINDIR)TX4 $(BINDIR)videoServer $(BINDIR)TX3 $(BINDIR)TX $(BINDIR)RX $(BINDIR)RX1 $(BINDIR)RX2 $(BINDIR)TX2 $(BINDIR)debter $(BINDIR)grabber $(BINDIR)grabberDebter_v2 $(BINDIR)grabberDebter $(BINDIR)header_v2

bindir:
	mkdir -p $(BINDIR)

radio:
	make -C $(LIBDIR) -f radio.mk RADIOFLAGS="$(RARGS)"

improc:
	sudo cp $(PRIVATELIBDIR)/lib$(EDULIB_V2).so /usr/lib
	sudo ldconfig

$(BINDIR)TX: TX.cpp
	$(CXX) -std=c++0x -I"../includes/" -L $(LIBDIR)  $< -o $@ -lradio -lcrypto

$(BINDIR)TX2: TX2.cpp
	$(CXX) -std=c++0x -I"../includes/" -L $(LIBDIR)  $< -o $@ -lradio -lcrypto

$(BINDIR)TX3: TX3.cpp
	$(CXX) -std=c++0x -I"../includes/" -L $(LIBDIR)  $< -o $@ -lradio -lcrypto

$(BINDIR)TX4: TX4.cpp
	$(CXX) -std=c++0x -I"../includes/" -I $(INCLUDES) -L $(LIBDIR)  $< -o $@ -lradio -lcrypto

#$(BINDIR)videoServer: videoServer.cpp
#	$(CXX) -std=c++0x -I"../includes/" -I $(INCLUDES) -I $(EDULIB_V2_INCLUDES) -L $(LIBDIR) -L $(PRIVATELIBDIR)  $< -lpthread -lradio -lcrypto -l$(EDULIB_V2) -lm -o $@ 
$(BINDIR)videoServer: videoServer.c
	$(CC) $< -lpthread -I $(INCLUDES)  -o $@ 

$(BINDIR)RX: RX.cpp
	$(CXX) -std=c++0x -I"../includes/" -L $(LIBDIR)  $< -o $@ -lradio -lcrypto

$(BINDIR)RX1: RX1.cpp
	$(CXX) -std=c++0x -I"../includes/" -L $(LIBDIR)  $< -o $@ -lradio -lcrypto

$(BINDIR)RX2: RX2.cpp
	$(CXX) -std=c++0x -I"../includes/" -L $(LIBDIR)  $< -o $@ -lradio -lcrypto

$(BINDIR)header_v2: header.c
	$(CC) -I $(INCLUDES) $< -o $@ 

$(BINDIR)grabber: grabber.c
	$(CC) -I $(INCLUDES) $< -o $@ 

$(BINDIR)grabberDebter: grabberDebter.c
	$(CC) -I $(INCLUDES) -I $(EDULIB_V2_INCLUDES) -L $(PRIVATELIBDIR) $< -o $@ -l$(EDULIB_V2) -lm

$(BINDIR)grabberDebter_v2: grabberDebter_v2.c
	$(CC) -I $(INCLUDES) -I $(EDULIB_V2_INCLUDES) -L $(PRIVATELIBDIR) $< -o $@ -l$(EDULIB_V2) -lm

$(BINDIR)debter: debt.c
	$(CC) -I $(INCLUDES) -I $(EDULIB_V2_INCLUDES) -L $(PRIVATELIBDIR) $< -o $@ -l$(EDULIB_V2) -lm

clean:
	rm -f $(BINDIR)/*
	make -C $(LIBDIR) clean

