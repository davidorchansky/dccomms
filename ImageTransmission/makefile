LIBDIR = ../Release
RARGS = 
BINDIR = bin/
PRIVATELIBDIR = ./libraries/x86_64
INCLUDES = includes 
EDULIB_V2_INCLUDES = libraries/debt/src
EDULIB_V2 = debt
all: bindir radio improc $(BINDIR)videoServer_2procOnly_noArduino $(BINDIR)TX_allInOne_noArduino $(BINDIR)TX_dynamic_noArduino $(BINDIR)TX_dynamic $(BINDIR)TX_noArduino $(BINDIR)videoServer $(BINDIR)TX $(BINDIR)RX $(BINDIR)debter $(BINDIR)grabber $(BINDIR)grabberDebter_dynamic $(BINDIR)grabberDebter $(BINDIR)header $(BINDIR)fileReceiver $(BINDIR)fileTransmitter $(BINDIR)videoServer_noArduino

bindir:
	mkdir -p $(BINDIR)

radio:
	make -C $(LIBDIR) -f radio.mk RADIOFLAGS="$(RARGS)"

improc:
	sudo cp $(PRIVATELIBDIR)/lib$(EDULIB_V2).so /usr/lib
	sudo ldconfig

$(BINDIR)fileReceiver: fileReceiver.cpp
	$(CXX) -std=c++0x -I"../includes/" -L $(LIBDIR)  $< -o $@ -lradio -lcrypto

$(BINDIR)fileTransmitter: fileTransmitter.cpp
	$(CXX) -std=c++0x -I"../includes/" -L $(LIBDIR)  $< -o $@ -lradio -lcrypto

$(BINDIR)TX: TX.cpp
	$(CXX) -std=c++0x -I"../includes/" -L $(LIBDIR)  $< -o $@ -lradio -lcrypto

$(BINDIR)TX_noArduino: TX_noArduino.cpp
	$(CXX) -std=c++0x -I"../includes/" -L $(LIBDIR)  $< -o $@ -lradio -lcrypto

$(BINDIR)TX_allInOne_noArduino: TX_allInOne_noArduino.cpp includes/videoServer.h includes/grabberDebter.h
	$(CXX) -std=c++0x -I"../includes/" -I $(INCLUDES) -I $(EDULIB_V2_INCLUDES) -L $(PRIVATELIBDIR) -L $(LIBDIR)  $< -o $@ -lradio -lcrypto -l$(EDULIB_V2) -lm

$(BINDIR)TX_dynamic: TX_dynamic.cpp includes/videoServer.h
	$(CXX) -std=c++0x -I"../includes/" -I $(INCLUDES) -L $(LIBDIR)  $< -o $@ -lradio -lcrypto

$(BINDIR)TX_dynamic_noArduino: TX_dynamic_noArduino.cpp includes/videoServer.h
	$(CXX) -std=c++0x -I"../includes/" -I $(INCLUDES) -L $(LIBDIR)  $< -o $@ -lradio -lcrypto

$(BINDIR)videoServer: videoServer.c includes/videoServer.h
	$(CC) $< -lpthread -I $(INCLUDES)  -o $@ 

$(BINDIR)videoServer_noArduino: videoServer_noArduino.c includes/videoServer.h
	$(CC) $< -lpthread -I $(INCLUDES)  -o $@ 

$(BINDIR)videoServer_2procOnly_noArduino: videoServer_2procOnly_noArduino.c includes/videoServer.h
	$(CC) $< -lpthread -I $(INCLUDES)  -o $@ 

$(BINDIR)RX: RX.cpp
	$(CXX) -std=c++0x -I"../includes/" -L $(LIBDIR)  $< -o $@ -lradio -lcrypto

$(BINDIR)header: header.c
	$(CC) -I $(INCLUDES) $< -o $@ 

$(BINDIR)grabber: grabber.c
	$(CC) -I $(INCLUDES) $< -o $@ 

$(BINDIR)grabberDebter: grabberDebter.c
	$(CC) -I $(INCLUDES) -I $(EDULIB_V2_INCLUDES) -L $(PRIVATELIBDIR) $< -o $@ -l$(EDULIB_V2) -lm

$(BINDIR)grabberDebter_dynamic: grabberDebter_dynamic.c
	$(CC) -I $(INCLUDES) -I $(EDULIB_V2_INCLUDES) -L $(PRIVATELIBDIR) $< -o $@ -l$(EDULIB_V2) -lm

$(BINDIR)debter: debt.c
	$(CC) -I $(INCLUDES) -I $(EDULIB_V2_INCLUDES) -L $(PRIVATELIBDIR) $< -o $@ -l$(EDULIB_V2) -lm

clean:
	rm -f $(BINDIR)/*
	make -C $(LIBDIR) clean

