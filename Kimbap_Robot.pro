QT       += core gui network sql

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    DialogCamera.cpp \
    DialogDetector.cpp \
    DialogRobot.cpp \
    Logger.cpp \
    Scheduler.cpp \
    main.cpp \
    mainwindow.cpp

HEADERS += \
    DialogCamera.h \
    DialogDetector.h \
    DialogRobot.h \
    GlobalHeader.h \
    Logger.h \
    Scheduler.h \
    mainwindow.h

FORMS += \
    DialogCamera.ui \
    DialogRobot.ui \
    mainwindow.ui

# OpenCV
OPENCV_PATH = /usr/local

INCLUDEPATH += $${OPENCV_PATH}/include/opencv4
LIBS += -L$${OPENCV_PATH}/lib \
        -lopencv_core \
        -lopencv_imgproc \
        -lopencv_highgui \
        -lopencv_dnn

# If pkg-config is available and opencv4.pc is found
system(pkg-config --exists opencv4) {
    CONFIG += link_pkgconfig
    PKGCONFIG += opencv4
}

# RealSense
LIBS += -lrealsense2

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
