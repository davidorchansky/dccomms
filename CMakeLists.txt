cmake_minimum_required (VERSION 2.6)
project (dccomms CXX)


set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_COLOR_MAKEFILE   ON)

# Remove 'lib' prefix for shared libraries on Windows
if (WIN32)
  set(CMAKE_SHARED_LIBRARY_PREFIX "")
endif ()
 
####### LOAD DEPENDENCIES ######

SET(COMMS_DEPS_DIR "deps/")

if (NOT TARGET cpplogging)
	add_subdirectory(${COMMS_DEPS_DIR}/cpplogging)
endif()

###### 

SET(COMMS_INCLUDE_DIRS "includes")
file(GLOB COMMS_CPP_FILES "src/*.cpp") 
SET(COMMS_CXX_COMPILE_FLAGS "-fPIC -std=c++0x")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${COMMS_CXX_COMPILE_FLAGS}")
SET(CMAKE_BUILD_TYPE Release)

add_library(${PROJECT_NAME} SHARED ${COMMS_CPP_FILES})
target_include_directories(${PROJECT_NAME} PUBLIC ${COMMS_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} m pthread rt crypto cpplogging)
