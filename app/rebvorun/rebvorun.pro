

CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt


DESTDIR=./
OBJECTS_DIR=../../build/app/rebvorun

TARGET = rebvorun
CONFIG += console
INCLUDEPATH += ../../include/

LIBS += ../../rebvolib/librebvolib.a
PRE_TARGETDEPS += ../../rebvolib/librebvolib.a

LIBS += -L/usr/local/lib \
    -lX11 \
    -lv4l2 \
    -lm \
    -lgd\
    -llapack\
    -lpthread #-lNE10


QMAKE_CXXFLAGS += -std=c++11 -Wno-unused-parameter -Wno-sign-compare $(REBVOFLAGS)# -mcpu=cortex-a15 -mtune=cortex-a15 -mfpu=neon -O2
QMAKE_CFLAGS += -std=c++11 -Wno-unused-parameter -Wno-sign-compare $(REBVOFLAGS)# -mcpu=cortex-a15 -mtune=cortex-a15 -mfpu=neon -O2
QMAKE_LFLAGS += -std=c++11 



SOURCES +=  main.cpp \
            archimu.cpp 

HEADERS += archimu.h 

OTHER_FILES += GlobalConfig

