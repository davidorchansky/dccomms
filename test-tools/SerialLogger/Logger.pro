#-------------------------------------------------
#
# Project created by QtCreator 2014-05-08T13:37:44
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = Logger
TEMPLATE = app


SOURCES += main.cpp\
        loggerwindow.cpp \
    connectionthread.cpp \
    tcpserver.cpp

HEADERS  += loggerwindow.h \
    connectionthread.h \
    tcpserver.h

FORMS    += \
    loggerwindow.ui

QT += serialport
QT += network

