LIBDIR = ../Release
RARGS = 
BINDIR = bin/
PRIVATELIBDIR = ./libraries/x86_64
INCLUDES = includes 
EDULIB_V2_INCLUDES = libraries/debt/src
EDULIB_V2 = debt


all: bindir radio improc $(BINDIR)videoServer_2procOnly_noArduino $(BINDIR)TX_allInOne $(BINDIR)TX_allInOne_noServer $(BINDIR)TX_allInOne_noArduino $(BINDIR)TX_dynamic_noArduino $(BINDIR)TX_dynamic $(BINDIR)TX_noArduino $(BINDIR)videoServer $(BINDIR)TX $(BINDIR)RX $(BINDIR)debter $(BINDIR)grabber $(BINDIR)grabberDebter_dynamic $(BINDIR)grabberDebter $(BINDIR)header $(BINDIR)fileReceiver $(BINDIR)fileTransmitter $(BINDIR)videoServer_noArduino $(BINDIR)videoServer_2procOnly $(BINDIR)RX_test $(BINDIR)RX_allInOne $(BINDIR)ppmVisor $(BINDIR)test $(BINDIR)RX_hackrf $(BINDIR)TX_hackrf

CC = gcc
FLAGS = -O3

opencvLibs = -lopencv_calib3d -lopencv_contrib -lopencv_core -lopencv_features2d -lopencv_flann -lopencv_gpu -lopencv_highgui -lopencv_imgproc -lopencv_legacy -lopencv_ml -lopencv_objdetect -lopencv_ocl -lopencv_photo -lopencv_stitching -lopencv_superres -lopencv_ts -lopencv_video -lopencv_videostab
bindir:
	mkdir -p $(BINDIR)

radio:
	make -C $(LIBDIR) -f radio.mk RADIOFLAGS="$(RARGS)"

improc:
	sudo cp $(PRIVATELIBDIR)/lib$(EDULIB_V2).so /usr/lib
	sudo ldconfig

$(BINDIR)fileReceiver: fileReceiver.cpp
	$(CXX) $(FLAGS) -std=c++0x -I"../includes/" -L $(LIBDIR)  $< -o $@ -lradio -lcrypto

$(BINDIR)fileTransmitter: fileTransmitter.cpp
	$(CXX) $(FLAGS) -std=c++0x -I"../includes/" -L $(LIBDIR)  $< -o $@ -lradio -lcrypto

$(BINDIR)TX: TX.cpp
	$(CXX) $(FLAGS) -std=c++0x -I"../includes/" -L $(LIBDIR)  $< -o $@ -lradio -lcrypto

$(BINDIR)TX_noArduino: TX_noArduino.cpp
	$(CXX) $(FLAGS) -std=c++0x -I"../includes/" -L $(LIBDIR)  $< -o $@ -lradio -lcrypto

$(BINDIR)TX_allInOne_noArduino: TX_allInOne_noArduino.cpp includes/videoServer.h includes/grabberDebter.h
	$(CXX) $(FLAGS) -std=c++0x -I"../includes/" -I $(INCLUDES) -I $(EDULIB_V2_INCLUDES) -L $(PRIVATELIBDIR) -L $(LIBDIR)  $< -o $@ -lpthread -lradio -lcrypto -l$(EDULIB_V2) -lm

$(BINDIR)TX_allInOne: TX_allInOne.cpp includes/videoServer.h includes/grabberDebter.h
	$(CXX) $(FLAGS) -std=c++0x -I"../includes/" -I $(INCLUDES) -I $(EDULIB_V2_INCLUDES) -L $(PRIVATELIBDIR) -L $(LIBDIR)  $< -o $@ -lpthread -lradio -lcrypto -l$(EDULIB_V2) -lm

$(BINDIR)TX_allInOne_noServer: TX_allInOne_noServer.cpp includes/videoServer.h includes/grabberDebter.h
	$(CXX) $(FLAGS) -std=c++0x -I"../includes/" -I $(INCLUDES) -I $(EDULIB_V2_INCLUDES) -L $(PRIVATELIBDIR) -L $(LIBDIR)  $< -o $@ -lpthread -lradio -lcrypto -l$(EDULIB_V2) -lm

$(BINDIR)TX_hackrf: TX_hackrf.cpp includes/videoServer.h includes/grabberDebter.h
	$(CXX) $(FLAGS) -std=c++0x -I"../includes/" -I $(INCLUDES) -I $(EDULIB_V2_INCLUDES) -L $(PRIVATELIBDIR) -L $(LIBDIR)  $< -o $@ -lpthread -lradio -lcrypto -l$(EDULIB_V2) -lm


$(BINDIR)test: test.cpp includes/grabberDebter.h
	$(CXX) $(FLAGS) -std=c++0x -I"../includes/" -I $(INCLUDES) -I $(EDULIB_V2_INCLUDES) -L $(PRIVATELIBDIR) -L $(LIBDIR)  $< -o $@ -lpthread -lradio -lcrypto -l$(EDULIB_V2) -lm

$(BINDIR)TX_dynamic: TX_dynamic.cpp includes/videoServer.h
	$(CXX) $(FLAGS) -std=c++0x -I"../includes/" -I $(INCLUDES) -L $(LIBDIR)  $< -o $@ -lradio -lcrypto

$(BINDIR)TX_dynamic_noArduino: TX_dynamic_noArduino.cpp includes/videoServer.h
	$(CXX) $(FLAGS) -std=c++0x -I"../includes/" -I $(INCLUDES) -L $(LIBDIR)  $< -o $@ -lradio -lcrypto

$(BINDIR)videoServer: videoServer.c includes/videoServer.h
	$(CC) $(FLAGS) $< -lpthread -I $(INCLUDES)  -o $@ 

$(BINDIR)videoServer_noArduino: videoServer_noArduino.c includes/videoServer.h
	$(CC) $(FLAGS) $< -lpthread -I $(INCLUDES)  -o $@ 

$(BINDIR)videoServer_2procOnly_noArduino: videoServer_2procOnly_noArduino.c includes/videoServer.h
	$(CC) $(FLAGS) $< -lpthread -I $(INCLUDES)  -o $@ 

$(BINDIR)videoServer_2procOnly: videoServer_2procOnly.c includes/videoServer.h
	$(CC) $(FLAGS) $< -lpthread -I $(INCLUDES)  -o $@ 


$(BINDIR)RX: RX.cpp
	$(CXX) $(FLAGS) -std=c++0x -I"../includes/" -L $(LIBDIR)  $< -o $@ -lradio -lcrypto

$(BINDIR)RX_allInOne: RX_allInOne.cpp includes/grabberDebter.h includes/videoServer.h
	$(CXX) $(FLAGS) -std=c++0x -I"../includes/" -I $(INCLUDES) -I $(EDULIB_V2_INCLUDES) -L $(PRIVATELIBDIR) -L $(LIBDIR)  $< -o $@ -lpthread -lradio -lcrypto -l$(EDULIB_V2) -lm $(opencvLibs)

$(BINDIR)RX_hackrf: RX_hackrf.cpp includes/grabberDebter.h includes/videoServer.h
	$(CXX) $(FLAGS) -std=c++0x -I"../includes/" -I $(INCLUDES) -I $(EDULIB_V2_INCLUDES) -L $(PRIVATELIBDIR) -L $(LIBDIR)  $< -o $@ -lpthread -lradio -lcrypto -l$(EDULIB_V2) -lm $(opencvLibs)


$(BINDIR)ppmVisor: ppmVisor.cpp
	$(CXX) $(FLAGS) -std=c++0x -I"../includes/" -I $(INCLUDES) -I $(EDULIB_V2_INCLUDES) -L $(PRIVATELIBDIR) -L $(LIBDIR)  $< -o $@ -lpthread -lradio -lcrypto -l$(EDULIB_V2) -lm $(opencvLibs)


$(BINDIR)RX_test: RX_test.cpp
	$(CXX) $(FLAGS) -std=c++0x -I"../includes/" -L $(LIBDIR)  $< -o $@ -lradio -lcrypto


$(BINDIR)header: header.c
	$(CC) $(FLAGS) -I $(INCLUDES) $< -o $@ 

$(BINDIR)grabber: grabber.c
	$(CC) $(FLAGS) -I $(INCLUDES) $< -o $@ 

$(BINDIR)grabberDebter: grabberDebter.c
	$(CC) $(FLAGS) -I $(INCLUDES) -I $(EDULIB_V2_INCLUDES) -L $(PRIVATELIBDIR) $< -o $@ -l$(EDULIB_V2) -lm

$(BINDIR)grabberDebter_dynamic: grabberDebter_dynamic.c
	$(CC) $(FLAGS) -I $(INCLUDES) -I $(EDULIB_V2_INCLUDES) -L $(PRIVATELIBDIR) $< -o $@ -l$(EDULIB_V2) -lm

$(BINDIR)debter: debt.c
	$(CC) $(FLAGS) -I $(INCLUDES) -I $(EDULIB_V2_INCLUDES) -L $(PRIVATELIBDIR) $< -o $@ -l$(EDULIB_V2) -lm

clean:
	rm -f $(BINDIR)/*
	make -C $(LIBDIR) clean

