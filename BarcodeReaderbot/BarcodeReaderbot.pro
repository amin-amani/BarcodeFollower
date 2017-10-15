#-------------------------------------------------
#
# Project created by QtCreator 2017-10-16T04:02:08
#
#-------------------------------------------------

QT       += core serialport testlib network

QT       -= gui

TARGET = BarcodeReaderbot
CONFIG   += console
CONFIG   -= app_bundle

target.path += /home/pi/Desktop
INSTALLS += target

TEMPLATE = app


SOURCES += main.cpp \
    controller.cpp \
    datamodel.cpp \
    dynamixel.cpp \
    robot.cpp \
    scanner.cpp \
    tcpserver.cpp \
    Media/media.cpp

HEADERS += \
    controller.h \
    datamodel.h \
    dynamixel.h \
    robot.h \
    scanner.h \
    tcpserver.h \
    Media/media.h
