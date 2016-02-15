# -------------------------------------------------
# Project created by QtCreator 2011-11-08T18:16:13
# -------------------------------------------------
QT -= core \
    gui
TARGET = controlOff
CONFIG += console
CONFIG -= app_bundle
QMAKE_CXXFLAGS += -std=c++11 -Wno-unused-parameter -Wno-sign-compare
INCLUDEPATH += /usr/local/include/lapackpp/ \
		../Blocks/ \
		../mtracklib/\
                ../CommLib/\
                ../VideoLib/\
                ../UtilLib/\
                ../NeonLib/
LIBS += -L/usr/local/lib \
    -lX11 \
    -lv4l2 \
    -lm \
    -lfftw3 \
    -llapack \
    -lavcodec \
    -lGL\
    -lGLU\
    -lglut\
    -lgd
TEMPLATE = app
SOURCES += main.cpp \
    ../VideoLib/video_io.c \
    ../Blocks/configurator.cpp \
    ../Blocks/displayFrontal.cpp \
    ../mtracklib/iimage.cpp \
    ../mtracklib/sspace.cpp \
    ../mtracklib/iigauss.cpp  \
    ../mtracklib/net_keypoint.cpp\
    ../VideoLib/videodecoder.cpp\
    ../mtracklib/gl_viewer.cpp \
    ../mtracklib/depth_filler.cpp \
    ../mtracklib/crash_detector.cpp\
    ../VideoLib/v4lcam.cpp\
    ../VideoLib/video_mjpeg.cpp\
    ../VideoLib/video_encoder.cpp\
    ../VideoLib/videocam.cpp \
    ../UtilLib/ttimer.cpp\
    ../VideoLib/simcam.cpp\
    ../CommLib/udp_port.cpp

HEADERS += ../VideoLib/video_io.h \
    ../VideoLib/image.h \
    ../Blocks/configurator.h \
    ../Blocks/camaraFrontal.h\
    ../Blocks/displayFrontal.h\
    ../mtracklib/iimage.h \
    ../mtracklib/sspace.h \
    ../mtracklib/iigauss.h \
    ../UtilLib/util.h \
    ../mtracklib/trail.h \
    ../mtracklib/net_keypoint.h\
    ../mtracklib/nav_data_defs.h \
    ../mtracklib/ransacizer.h \
    ../VideoLib/videodecoder.h\
    ../mtracklib/edge_finder.h \
    ../mtracklib/edge_tracker.h \
    ../mtracklib/gl_viewer.h \
    ../mtracklib/depth_filler.h \
    ../mtracklib/crash_detector.h\
    ../VideoLib/v4lcam.h\
    ../VideoLib/video_mjpeg.h\
    ../VideoLib/video_encoder.h\
    ../VideoLib/videocam.h \
    ../VideoLib/common_types.h \
    ../UtilLib/ttimer.h\
    ../VideoLib/simcam.h\
    ../UtilLib/pipeline.h\
    ../CommLib/udp_port.h\
    ../UtilLib/CircList.h\
    ../UtilLib/cam_model.h\
    ../UtilLib/fvector.h

OTHER_FILES += GlobalConfig
