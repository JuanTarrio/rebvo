

QT -= core \
    gui
TARGET = rebvo
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

QMAKE_CXXFLAGS += -std=c++11 -Wno-unused-parameter -Wno-sign-compare # -mcpu=cortex-a15 -mtune=cortex-a15 -mfpu=neon -O2
QMAKE_CFLAGS += -std=c++11 -Wno-unused-parameter -Wno-sign-compare # -mcpu=cortex-a15 -mtune=cortex-a15 -mfpu=neon -O2
QMAKE_LFLAGS += -std=c++11 

target.path=/home/odroid
INSTALLS += target



SOURCES +=  main.cpp \
    ../Blocks/rebvo.cpp\
    ../VideoLib/video_io.c \
    ../VideoLib/video_mfc.cpp\
    ../VideoLib/v4lcam.cpp \
    ../VideoLib/video_mjpeg.cpp \
    ../VideoLib/video_encoder.cpp \
    ../VideoLib/videocam.cpp\
    ../VideoLib/simcam.cpp \
    ../VideoLib/datasetcam.cpp \
    ../mtracklib/iimage.cpp \
    ../mtracklib/sspace.cpp \
    ../mtracklib/iigauss.cpp \
    ../mtracklib/net_keypoint.cpp \
    ../mtracklib/edge_finder.cpp \
    ../mtracklib/edge_tracker.cpp \
    ../mtracklib/global_tracker.cpp \
    ../UtilLib/ttimer.cpp\
    ../UtilLib/ne10wrapper.cpp\
    ../UtilLib/configurator.cpp \
    ../CommLib/udp_port.cpp \
    ../mtracklib/keyframe.cpp \
    ../VideoLib/imugrabber.cpp\
    ../UtilLib/minimizer.cpp \
    ../mtracklib/scaleestimator.cpp \
    ../Blocks/rebvo_first_t.cpp \
    ../Blocks/rebvo_second_t.cpp \
    ../Blocks/rebvo_third_t.cpp \
    ../Blocks/archimu.cpp \
    ../UtilLib/libcrc.cpp \
    ../mtracklib/image_undistort.cpp

HEADERS += ../Blocks/incfiles.h \
    ../Blocks/rebvo.h\
    ../VideoLib/video_io.h \
    ../VideoLib/image.h \
    ../VideoLib/video_mfc.h\
    ../VideoLib/v4lcam.h \
    ../VideoLib/video_mjpeg.h \
    ../VideoLib/video_encoder.h \
    ../VideoLib/videocam.h\
    ../VideoLib/simcam.h \
    ../VideoLib/common_types.h \
    ../VideoLib/datasetcam.h \
    ../mtracklib/iimage.h \
    ../mtracklib/sspace.h \
    ../mtracklib/iigauss.h \
    ../mtracklib/edge_tracker.h \
    ../mtracklib/global_tracker.h\
    ../mtracklib/net_keypoint.h \
    ../mtracklib/nav_data_defs.h \
    ../mtracklib/edge_finder.h \
    ../UtilLib/configurator.h\
    ../UtilLib/util.h \
    ../UtilLib/NormalDistribution.h \
    ../UtilLib/ne10wrapper.h\
    ../UtilLib/linefitting.h \
    ../UtilLib/cam_model.h \
    ../UtilLib/timer.h\
    ../UtilLib/fvector.h\
    ../UtilLib/ttimer.h\
    ../UtilLib/pipeline.h\
    ../UtilLib/CircList.h\
    ../CommLib/udp_port.h \
    ../mtracklib/keyframe.h \
    ../VideoLib/imugrabber.h\
    ../UtilLib/minimizer.h \
    ../mtracklib/scaleestimator.h \
    ../Blocks/archimu.h \
    ../UtilLib/libcrc.h \
    ../mtracklib/image_undistort.h


OTHER_FILES += GlobalConfig
