LIBDIR = ../Release
RARGS = 
BINDIR = bin/
PRIVATELIBDIR = ./libraries/x86_64
INCLUDES = includes 
EDULIB_V2_INCLUDES = libraries/debt/src
EDULIB_V2 = debt
all: bindir radio improc $(BINDIR)TX3 $(BINDIR)TX $(BINDIR)RX $(BINDIR)RX1 $(BINDIR)RX2 $(BINDIR)TX2 $(BINDIR)debter $(BINDIR)grabber

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

$(BINDIR)RX: RX.cpp
	$(CXX) -std=c++0x -I"../includes/" -L $(LIBDIR)  $< -o $@ -lradio -lcrypto

$(BINDIR)RX1: RX1.cpp
	$(CXX) -std=c++0x -I"../includes/" -L $(LIBDIR)  $< -o $@ -lradio -lcrypto

$(BINDIR)RX2: RX2.cpp
	$(CXX) -std=c++0x -I"../includes/" -L $(LIBDIR)  $< -o $@ -lradio -lcrypto

$(BINDIR)grabber: grabber.c
	$(CXX) -I $(INCLUDES) $< -o $@ 


$(BINDIR)debter: debt.c
	$(CXX) -I $(INCLUDES) -I $(EDULIB_V2_INCLUDES) -L $(PRIVATELIBDIR) $< -o $@ -l$(EDULIB_V2) -lm

#gcc -L ../x86_64/static-libraries/ -I ../src/ debt.c -ldebt -lm -o debter
clean:
	rm -f $(BINDIR)/*
	make -C $(LIBDIR) clean

