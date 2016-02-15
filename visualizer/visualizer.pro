
QT -= core \
    gui
TARGET = visualizer
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
    ../Blocks/visualizer.cpp \
    ../VideoLib/video_io.c \
    ../VideoLib/videodecoder.cpp\
    ../VideoLib/simcam.cpp\
    ../VideoLib/v4lcam.cpp\
    ../VideoLib/video_mjpeg.cpp\
    ../VideoLib/video_encoder.cpp\
    ../VideoLib/videocam.cpp \
    ../mtracklib/iimage.cpp \
    ../mtracklib/sspace.cpp \
    ../mtracklib/iigauss.cpp  \
    ../mtracklib/net_keypoint.cpp\
    ../mtracklib/gl_viewer.cpp \
    ../mtracklib/depth_filler.cpp \
    ../mtracklib/crash_detector.cpp\
    ../UtilLib/ttimer.cpp\
    ../UtilLib/configurator.cpp \
    ../CommLib/udp_port.cpp

HEADERS += ../UtilLib/configurator.h \
    ../Blocks/rebvo.h\
    ../Blocks/visualizer.h\
    ../mtracklib/trail.h \
    ../mtracklib/net_keypoint.h\
    ../mtracklib/nav_data_defs.h \
    ../mtracklib/ransacizer.h \
    ../mtracklib/edge_finder.h \
    ../mtracklib/edge_tracker.h \
    ../mtracklib/gl_viewer.h \
    ../mtracklib/depth_filler.h \
    ../mtracklib/crash_detector.h\
    ../mtracklib/iimage.h \
    ../mtracklib/sspace.h \
    ../mtracklib/iigauss.h \
    ../VideoLib/v4lcam.h\
    ../VideoLib/video_mjpeg.h\
    ../VideoLib/video_encoder.h\
    ../VideoLib/videocam.h \
    ../VideoLib/common_types.h \
    ../VideoLib/simcam.h\
    ../VideoLib/videodecoder.h\
    ../VideoLib/video_io.h \
    ../VideoLib/image.h \
    ../UtilLib/ttimer.h\
    ../UtilLib/pipeline.h\
    ../UtilLib/CircList.h\
    ../UtilLib/cam_model.h\
    ../UtilLib/fvector.h\
    ../UtilLib/util.h \
    ../CommLib/udp_port.h

OTHER_FILES += GlobalConfig
