#-------------------------------------------------
#
# Project created by QtCreator 2012-08-16T20:46:42
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = sift_match
TEMPLATE = app


SOURCES += main.cpp\
        siftmatch.cpp \
    utils.c \
    xform.c \
    sift.c \
    minpq.c \
    kdtree.c \
    imgfeatures.c

HEADERS  += siftmatch.h \
    utils.h \
    xform.h \
    sift.h \
    minpq.h \
    kdtree.h \
    imgfeatures.h

FORMS    += siftmatch.ui

INCLUDEPATH +=  /usr/local/include\
                /usr/local/include/opencv\
                /usr/local/include/opencv2

LIBS += /usr/local/lib/libopencv_core.so.2.4.9\
        /usr/local/lib/libopencv_highgui.so.2.4.9\
        /usr/local/lib/libopencv_imgproc.so.2.4.9\
        /usr/local/lib/libopencv_stitching.so.2.4.9\  #Í¼ÏñÆ´½ÓÄ£¿é
        /usr/local/lib/libopencv_nonfree.so.2.4.9\     #SIFT,SURF
        /usr/local/lib/libopencv_features2d.so.2.4.9   #ÌØÕ÷¼ì²â
