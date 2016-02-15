# -------------------------------------------------
# Project created by QtCreator 2011-11-08T18:16:13
# -------------------------------------------------
QT -= core \
    gui
TARGET = controlOn
CONFIG += console
CONFIG -= app_bundle
INCLUDEPATH += /usr/local/include/lapackpp/ \
		../Blocks/\
		../mtracklib/\
                ../CommLib/\
                ../VideoLib\
                ../UtilLib\
                ../Ne10\
                ../NeonLib
LIBS += -L/usr/local/lib \
    -lX11 \
    -lv4l2 \
    -lm \
    -lgd\
    -llapack #-lNE10

QMAKE_CXXFLAGS += -std=c++11 -Wno-unused-parameter -Wno-sign-compare# -g -mcpu=cortex-a15 -mtune=cortex-a15 -mfpu=neon -O2
QMAKE_CFLAGS += -std=c++11 -Wno-unused-parameter -Wno-sign-compare# -g -mcpu=cortex-a15 -mtune=cortex-a15 -mfpu=neon -O2
QMAKE_LFLAGS += -std=c++11 -g

target.path=/home/odroid
INSTALLS += target



#QMAKE_CXXFLAGS+=-O3

#QMAKE_CXXFLAGS-=-O2

#QMAKE_CXXFLAGS += -frepo

#QMAKE_CXXFLAGS += -mfpu=neon
#QMAKE_CFLAGS += -mfpu=neon

#QMAKE_CXXFLAGS += -pg  -fprofile-arcs
#QMAKE_CFLAGS += -pg  -fprofile-arcs
#QMAKE_LFLAGS += -pg  -fprofile-arcs



SOURCES +=  main.cpp \
    ../VideoLib/video_io.c \
    ../Blocks/configurator.cpp \
    ../Blocks/camaraFrontal.cpp\
    ../mtracklib/iimage.cpp \
    ../mtracklib/sspace.cpp \
    ../mtracklib/iigauss.cpp \
    ../mtracklib/net_keypoint.cpp \
    ../mtracklib/edge_finder.cpp \
    ../VideoLib/video_mfc.cpp\
    ../mtracklib/edge_tracker.cpp \
    ../VideoLib/v4lcam.cpp \
    ../VideoLib/video_mjpeg.cpp \
    ../VideoLib/video_encoder.cpp \
    ../VideoLib/videocam.cpp\
    ../UtilLib/ttimer.cpp\
    ../VideoLib/simcam.cpp \
    ../CommLib/udp_port.cpp\
    ../mtracklib/global_tracker.cpp \
    ../UtilLib/ne10wrapper.cpp

HEADERS += ../Blocks/incfiles.h \
    ../VideoLib/video_io.h \
    ../VideoLib/image.h \
    ../Blocks/configurator.h\
    ../Blocks/camaraFrontal.h\
    ../mtracklib/iimage.h \
    ../mtracklib/sspace.h \
    ../mtracklib/iigauss.h \
    ../UtilLib/util.h \
    ../CommLib/tcp_server.h \
    ../CommLib/tcp_client.h \
    ../mtracklib/net_keypoint.h \
    ../mtracklib/nav_data_defs.h \
    ../mtracklib/edge_finder.h \
    ../VideoLib/video_mfc.h\
    ../mtracklib/edge_tracker.h \
    ../VideoLib/v4lcam.h \
    ../VideoLib/video_mjpeg.h \
    ../VideoLib/video_encoder.h \
    ../VideoLib/videocam.h\
    ../UtilLib/ttimer.h\
    ../VideoLib/simcam.h \
    ../UtilLib/pipeline.h \
    ../CommLib/udp_port.h \
    ../UtilLib/CircList.h\
    ../mtracklib/global_tracker.h\
    ../UtilLib/NormalDistribution.h \
    ../UtilLib/ne10wrapper.h\
    ../UtilLib/linefitting.h \
    ../UtilLib/cam_model.h \
    ../UtilLib/timer.h\
    ../VideoLib/common_types.h \
    ../UtilLib/fvector.h

OTHER_FILES += GlobalConfig \
		adq.conf \
    Calibracion.txt
