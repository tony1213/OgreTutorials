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
    INCLUDEPATH += /usr/local/include/OGRE
    CONFIG += link_pkgconfig
    PKGCONFIG += OGRE
}

workdir=$(cd $(dirname $0); pwd)

LIBS += -L"/opt/ros/kinetic/lib" -lurdf -lrosconsole_bridge
#LIBS += -L"/usr/lib/x86_64-linux-gnu" -lcurl

LIBS += -lboost_filesystem -lboost_system  -ltinyxml -lcurl 
INCLUDEPATH += /opt/ros/kinetic/include


SOURCES += \
        main.cpp \
        mainwindow.cpp \
        robot_link.cpp \
        robot.cpp \
        stl_loader.cpp \
        retriever.cpp 

HEADERS += \
        mainwindow.h \
        robot_link.h \
        robot.h \
        stl_loader.h \
        retriever.h

FORMS += \
        mainwindow.ui
