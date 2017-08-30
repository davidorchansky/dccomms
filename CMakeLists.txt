cmake_minimum_required (VERSION 3.5.1)
project (dccomms CXX)


set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_COLOR_MAKEFILE   ON)

# Remove 'lib' prefix for shared libraries on Windows
if (WIN32)
  set(CMAKE_SHARED_LIBRARY_PREFIX "")
endif ()
 
####### LOAD DEPENDENCIES ######

SET(${PROJECT_NAME}_DEPS_DIR "deps/")

if (NOT TARGET cpplogging)
	add_subdirectory(${${PROJECT_NAME}_DEPS_DIR}/cpplogging)
endif()

###### 

SET(${PROJECT_NAME}_INCLUDE_DIRS "include")
SET(SRCPATH src/)
file(GLOB ${PROJECT_NAME}_CPP_FILES
    ${SRCPATH}Arduino.cpp
    ${SRCPATH}BlockRadioTransmitter.cpp
    ${SRCPATH}Checksum.cpp
    ${SRCPATH}CommsBridge.cpp
    ${SRCPATH}CommsDeviceService.cpp
    ${SRCPATH}CommsException.cpp
    ${SRCPATH}DataLinkFrame.cpp
    ${SRCPATH}ICommsDevice.cpp
    ${SRCPATH}ICommsLink.cpp
    ${SRCPATH}IStream.cpp
    ${SRCPATH}IStreamCommsDevice.cpp
    ${SRCPATH}NamedPipeStream.cpp
    ${SRCPATH}Packet.cpp
    ${SRCPATH}Radio.cpp
    ${SRCPATH}SerialPortStream.cpp
    ${SRCPATH}TCPStream.cpp
    ${SRCPATH}TransportPDU.cpp
    ${SRCPATH}Utils.cpp
    )
SET(${PROJECT_NAME}_CXX_COMPILE_FLAGS "-fPIC -std=c++0x")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${${PROJECT_NAME}_CXX_COMPILE_FLAGS}")

add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_CPP_FILES})
target_include_directories(${PROJECT_NAME} PUBLIC ${${PROJECT_NAME}_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} m pthread rt crypto cpplogging)

install(TARGETS ${PROJECT_NAME}
	LIBRARY DESTINATION "lib"
	)
install(DIRECTORY "include/dccomms" DESTINATION "include")
	
