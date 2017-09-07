

CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

CONFIG+=staticlib
TEMPLATE=lib

DESTDIR=./
OBJECTS_DIR=../build/rebvolib
TARGET=rebvolib

INCLUDEPATH += ../include/

LIBS += -L/usr/local/lib \
    -lX11 \
    -lv4l2 \
    -lm \
    -lgd\
    -llapack \
    $(REBVOLIBS) #-lNE10

QMAKE_CXXFLAGS += -std=c++11 -Wno-unused-parameter -Wno-sign-compare $(REBVOFLAGS)# -mcpu=cortex-a15 -mtune=cortex-a15 -mfpu=neon -O2
QMAKE_CFLAGS += -std=c++11 -Wno-unused-parameter -Wno-sign-compare $(REBVOFLAGS) # -mcpu=cortex-a15 -mtune=cortex-a15 -mfpu=neon -O2
QMAKE_LFLAGS += -std=c++11 $(REBVOFLAGS)



SOURCES +=  ../src/rebvo/rebvo.cpp\
    ../src/rebvo/rebvo_first_t.cpp \
    ../src/rebvo/rebvo_second_t.cpp \
    ../src/rebvo/rebvo_third_t.cpp \
    ../src/visualizer/visualizer.cpp\
    ../src/visualizer/depth_filler.cpp \
    ../src/visualizer/gl_viewer.cpp \
    ../src/VideoLib/video_io.cpp \
    ../src/VideoLib/video_mfc.cpp\
    ../src/VideoLib/v4lcam.cpp \
    ../src/VideoLib/video_mjpeg.cpp \
    ../src/VideoLib/video_encoder.cpp \
    ../src/VideoLib/videocam.cpp\
    ../src/VideoLib/simcam.cpp \
    ../src/VideoLib/datasetcam.cpp \
    ../src/VideoLib/videodecoder.cpp\
    ../src/VideoLib/customcam.cpp\
    ../src/VideoLib/image_undistort.cpp \
    ../src/mtracklib/iimage.cpp \
    ../src/mtracklib/sspace.cpp \
    ../src/mtracklib/iigauss.cpp \
    ../src/mtracklib/edge_finder.cpp \
    ../src/mtracklib/edge_tracker.cpp \
    ../src/mtracklib/global_tracker.cpp \
    ../src/mtracklib/keyframe.cpp \
    ../src/mtracklib/scaleestimator.cpp \
    ../src/mtracklib/kfvo.cpp\
    ../src/mtracklib/pose_graph.cpp\
    ../src/UtilLib/ttimer.cpp\
    ../src/UtilLib/ne10wrapper.cpp\
    ../src/UtilLib/configurator.cpp \
    ../src/UtilLib/minimizer.cpp \
    ../src/UtilLib/libcrc.cpp \
    ../src/UtilLib/imugrabber.cpp\
    ../src/UtilLib/linefitting.cpp\
    ../src/CommLib/udp_port.cpp \
    ../src/CommLib/net_keypoint.cpp \
    ../src/CommLib/edgemap_com.cpp \
    ../src/visualizer/surface_integrator.cpp

HEADERS += ../include/rebvo/rebvo.h\
    ../include/visualizer/visualizer.h\
    ../include/visualizer/depth_filler.h \
    ../include/visualizer/gl_viewer.h \
    ../include/VideoLib/video_io.h \
    ../include/VideoLib/image.h \
    ../include/VideoLib/video_mfc.h\
    ../include/VideoLib/v4lcam.h \
    ../include/VideoLib/video_mjpeg.h \
    ../include/VideoLib/video_encoder.h \
    ../include/VideoLib/videocam.h\
    ../include/VideoLib/simcam.h \
    ../include/VideoLib/common_types.h \
    ../include/VideoLib/datasetcam.h \
    ../include/VideoLib/videodecoder.h\
    ../include/VideoLib/customcam.h\
    ../include/VideoLib/image_undistort.h \
    ../include/mtracklib/iimage.h \
    ../include/mtracklib/sspace.h \
    ../include/mtracklib/iigauss.h \
    ../include/mtracklib/edge_tracker.h \
    ../include/mtracklib/global_tracker.h\
    ../include/mtracklib/nav_data_defs.h \
    ../include/mtracklib/edge_finder.h \
    ../include/mtracklib/keyframe.h \
    ../include/mtracklib/scaleestimator.h \
    ../include/mtracklib/kfvo.h\
    ../include/mtracklib/pose_graph.h\
    ../include/UtilLib/configurator.h\
    ../include/UtilLib/util.h \
    ../include/UtilLib/NormalDistribution.h \
    ../include/UtilLib/ne10wrapper.h\
    ../include/UtilLib/linefitting.h \
    ../include/UtilLib/cam_model.h \
    ../include/UtilLib/timer.h\
    ../include/UtilLib/fvector.h\
    ../include/UtilLib/ttimer.h\
    ../include/UtilLib/pipeline.h\
    ../include/UtilLib/CircList.h\
    ../include/UtilLib/minimizer.h \
    ../include/UtilLib/libcrc.h \
    ../include/UtilLib/imugrabber.h\
    ../include/UtilLib/linefitting.h\
    ../include/CommLib/net_keypoint.h \
    ../include/CommLib/udp_port.h \
    ../include/CommLib/edgemap_com.h \
    ../include/visualizer/surface_integrator.h \
    ../include/UtilLib/toon_util.h


