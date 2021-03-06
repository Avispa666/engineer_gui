#-------------------------------------------------
#
# Project created by QtCreator 2017-11-23T18:41:31
#
#-------------------------------------------------

QT       += network

QT       -= gui

TARGET = robot_control
TEMPLATE = lib

DEFINES += ROBOT_CONTROL_LIBRARY
CONFIG += c++1z
CONFIG -= static
QMAKE_LFLAGS += -Wl,--no-allow-shlib-undefined,--no-undefined
# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
        myudpclient.cpp \
        robot.cpp \
        robotconfiguration.cpp \
        robotcontroller.cpp \
        robotpositioncontroller.cpp \
    robotsi.cpp

HEADERS += \
        myudpclient.h \
        robot.h \
        robotconfiguration.h \
        robotcontroller.h \
        robotpositioncontroller.h \
        robot_control_global.h \ 
        robotpackets.h \
    robotsi.h

unix {
    target.path = /usr/lib
    INSTALLS += target
}
