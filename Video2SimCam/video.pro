# -------------------------------------------------
# Project created by QtCreator 2011-11-08T18:16:13
# -------------------------------------------------
QT -= core \
    gui
TARGET = video2simcam
CONFIG += console
CONFIG -= app_bundle

QMAKE_CXXFLAGS += -std=c++11  -Wno-unused-parameter -Wno-sign-compare
INCLUDEPATH += /usr/local/include/lapackpp/\
                ../Blocks/\
                ../mtracklib/\
                ../CommLib/\
                ../VideoLib\
                ../UtilLib

LIBS += -L/usr/local/lib \
    -L/usr/lib/x86_64-linux-gnu/mesa \
    -L/usr/lib/x86_64-linux-gnu/ \
    -lX11 \
    -lv4l2 \
    -lm \
    -lfftw3 \
    -llapack \
    -lavutil \
    -lavcodec \
    -lavformat \
    -lGL\
    -lGLU\
    -lglut\
    /usr/local/lib/libopencv_calib3d.so /usr/local/lib/libopencv_core.so /usr/local/lib/libopencv_features2d.so /usr/local/lib/libopencv_flann.so /usr/local/lib/libopencv_highgui.so /usr/local/lib/libopencv_imgcodecs.so /usr/local/lib/libopencv_imgproc.so /usr/local/lib/libopencv_ml.so /usr/local/lib/libopencv_objdetect.so /usr/local/lib/libopencv_photo.so /usr/local/lib/libopencv_shape.so /usr/local/lib/libopencv_stitching.so /usr/local/lib/libopencv_superres.so /usr/local/lib/libopencv_ts.a /usr/local/lib/libopencv_video.so /usr/local/lib/libopencv_videoio.so /usr/local/lib/libopencv_videostab.so

TEMPLATE = app
SOURCES += video.cpp

#HEADERS +=


#OTHER_FILES +=
