CMAKE_MINIMUM_REQUIRED(VERSION 2.8 FATAL_ERROR)
PROJECT("human_walk")

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR})
FIND_PACKAGE(ODE REQUIRED)
INCLUDE_DIRECTORIES(${ODE_INCLUDE_DIR} ${DRAWSTUFF_INCLUDE_DIR})
SET(CORELIBS ${ODE_LIBRARY} ${DRAWSTUFF_LIBRARY})

FIND_PACKAGE(GLUT)
FIND_PACKAGE(OpenGL)
SET(CORELIBS ${CORELIBS} ${GLUT_LIBRARY} ${OPENGL_LIBRARY})

ADD_EXECUTABLE(human_walk main.cpp)
TARGET_LINK_LIBRARIES(human_walk ${CORELIBS})
