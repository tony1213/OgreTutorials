#-------------------------------------------------
#
# Project created by QtCreator 2018-04-11T17:11:40
#
#-------------------------------------------------

QT       += core gui x11extras

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = OgreTutorials
TEMPLATE = app

DEFINES += QT_DEPRECATED_WARNINGS

unix {
    INCLUDEPATH += /usr/include/OGRE
    CONFIG += link_pkgconfig
    PKGCONFIG += OGRE
}

LIBS += -lboost_system


SOURCES += \
        main.cpp \
        mainwindow.cpp

HEADERS += \
        mainwindow.h

FORMS += \
        mainwindow.ui
