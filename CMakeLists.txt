cmake_minimum_required (VERSION 3.5)

###############################################################################
################## Baseline CMAKE Settings
###############################################################################
if(DEFINED ENV{AutoSM_DIR} AND NOT DEFINED ${AutoSM_DIR})
    set(AutoSM_DIR $ENV{AutoSM_DIR})
else()
    set(AutoSM_DIR ${CMAKE_CURRENT_SOURCE_DIR})
endif()

# Project
project(AutoSM CXX C)


###############################################################################
#### Compiler Settings
###############################################################################

# Cpp compiler version
set(CMAKE_CXX_STANDARD 14)

# Set the output folder where your program will be created
set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

###############################################################################
### Components / 3rd party software that can be turned ON/OFF across platforms.
###############################################################################

# 3rd party modules
option(WITH_CERES            "Build with CERES"                             ON)
option(WITH_SUITESPARSE      "Build with SuitSparse"                        OFF)
option(WITH_NLOPT            "Enable the nlopt optimization lib"            ON)


###############################################################################
################## Core include directories and definitions
###############################################################################
include_directories(${AutoSM_DIR})
include_directories(${CMAKE_BINARY_DIR})
include_directories(nodelab)

#Additional Libraries
list(APPEND THIRD_PARTY_LIBS pthread)


##############################################################################
################## 3rd Party Packages.
##############################################################################

#### Eigen3
FIND_PACKAGE( Eigen3 REQUIRED )
INCLUDE_DIRECTORIES ( ${EIGEN3_INCLUDE_DIR} )
add_definitions(-DHAVE_EIGEN)
list(APPEND THIRD_PARTY_LIBS ${EIGEN3_LIBS} )

#### OpenCV
FIND_PACKAGE( OpenCV REQUIRED )
INCLUDE_DIRECTORIES( ${OpenCV_INCLUDE_DIRS} )
add_definitions(-DHAVE_OPENCV)
list(APPEND THIRD_PARTY_LIBS ${OpenCV_LIBS} )

#### CERES
if (${WITH_CERES})
	add_definitions(-DHAVE_CERES)
	include_directories(SYSTEM ${AutoSM_DIR}/third_party/cere-solver/include)
	set(CERES_LIBS ${AutoSM_DIR}/third_party/ceres-solver/lib/libceres.a)
	list(APPEND THIRD_PARTY_LIBS ${CERES_LIBS} -lglog -lpthread -lcholmod -lgomp -lcxsparse)
	
endif (${WITH_CERES})

#### SUITESPARSE
if(${WITH_SUITESPARSE})
    add_definitions("-DWITH_SUITESPARSE")
    #set(SUITESPARSE_LIBS ${AutoSM_DIR}/third_party/suitesparse/lib)
    #list(APPEND THIRD_PARTY_LIBS ${SUITESPARSE_LIBS}/libcholmod.so  ${SUITESPARSE_LIBS}/libspqr.so)
    #include_directories(SYSTEM ${AutoSM_DIR}/third_party/SuiteSparse/include)
    FIND_PACKAGE( SuiteSparse )
    INCLUDE_DIRECTORIES( ${SuiteSparse_INCLUDE_DIRS} )
endif(${WITH_SUITESPARSE})

#### NL_OPT
if(${WITH_NLOPT})
	add_definitions(-DHAVE_NLOPT)
	set(NLOPT_DIR ${AutoSM_DIR}/third_party/nlopt)
	set(NLOPT_LIBS ${NLOPT_DIR}/lib/libnlopt.so)
	list(APPEND THIRD_PARTY_LIBS ${NLOPT_LIBS})
    	include_directories(SYSTEM ${NLOPT_DIR}/include)
endif (${WITH_NLOPT})

# Add all the source files for the project
add_library(
	AutoSM	SHARED

	math/transformation.h

	common/constants.h
	common/types.h

	simulation/dynamical_system.cpp
	simulation/sensor_models.cpp
	simulation/vehicle_models.cpp

	control/controllers.cpp

)

#add_executable(
#	first_test
#
#	first_test.cpp
#)

# Target 3rd party libraries
target_link_libraries(AutoSM ${THIRD_PARTY_LIBS})








