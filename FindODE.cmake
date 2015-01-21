# Base Io build system
# Written by there.exists.teslos<there.exists.teslos.gmail.com>
#
# Find ODE Open Dynamics Engine
FIND_PATH(ODE_INCLUDE_DIR 
    NAMES ode/ode.h
    PATHS /usr/include
          /usr/local/include
)

FIND_PATH(DRAWSTUFF_INCLUDE_DIR 
    NAMES drawstuff/drawstuff.h
    PATHS /usr/include
          /usr/local/include
)

FIND_LIBRARY(ODE_LIBRARY 
    NAMES ode libode
    PATHS /usr/lib
          /usr/local/lib
)

FIND_LIBRARY(DRAWSTUFF_LIBRARY 
    NAMES drawstuff libdrawstuff 
    PATHS /usr/lib
          /usr/local/lib
)

IF(ODE_INCLUDE_DIR)
    MESSAGE(STATUS "Found ODE include dir: ${ODE_INCLUDE_DIR}")
ELSE(ODE_INCLUDE_DIR)
    MESSAGE(STATUS "Couldn't find ODE include dir: ${ODE_INCLUDE_DIR}")
ENDIF(ODE_INCLUDE_DIR)

IF(ODE_LIBRARY)
    MESSAGE(STATUS "Found ODE library: ${ODE_LIBRARY}")
ELSE(ODE_LIBRARY)
    MESSAGE(STATUS "Couldn't find ODE library: ${ODE_LIBRARY}")
ENDIF(ODE_LIBRARY)

IF(DRAWSTUFF_LIBRARY)
    MESSAGE(STATUS "Found DrawStuff library: ${DRAWSTUFF_LIBRARY}")
ELSE(DRAWSTUFF_LIBRARY)
    MESSAGE(STATUS "Coulddn't find DrawStuff library: ${DRAWSTUFF_LIBRARY}")
ENDIF(DRAWSTUFF_LIBRARY)

IF(ODE_INCLUDE_DIR AND ODE_LIBRARY)
	SET(ODE_FOUND TRUE CACHE STRING "Whether ODE was found or not")
ENDIF(ODE_INCLUDE_DIR AND ODE_LIBRARY)

IF(ODE_FOUND)
    SET(CMAKE_C_FLAGS "-DdSINGLE")
	IF(NOT ODE_FIND_QUIETLY)
		MESSAGE(STATUS "Found ODE: ${ODE_LIBRARY}")
	ENDIF (NOT ODE_FIND_QUIETLY)
ELSE(ODE_FOUND)
	IF(ODE_FIND_REQUIRED)
		MESSAGE(FATAL_ERROR "Could not find ODE")
	ENDIF(ODE_FIND_REQUIRED)
ENDIF(ODE_FOUND)
