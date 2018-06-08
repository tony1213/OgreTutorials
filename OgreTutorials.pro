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
    INCLUDEPATH  += /usr/include/OGRE
    CONFIG += link_pkgconfig
    PKGCONFIG += OGRE
}

workdir=$(cd $(dirname $0); pwd)

LIBS += -L"/opt/ros/kinetic/lib" -lurdf  -lroscpp  -lrostime  -ltf -lroscpp_serialization  -lrosconsole_log4cxx 
LIBS+=/opt/ros/kinetic/lib/librosconsole.so
LIBS+=/usr/lib/x86_64-linux-gnu/libOgreMain.so 

LIBS += -lboost_filesystem -lboost_system  -ltinyxml -lcurl 
INCLUDEPATH += /opt/ros/kinetic/include

 ROS_MASTER_URI=http://localhost:11311

SOURCES += \
        main.cpp \
        mainwindow.cpp \
        robot_link.cpp \
        robot.cpp \
        stl_loader.cpp \
        retriever.cpp \
        robot_joint.cpp \
        property.cpp \
        property_tree_model.cpp \
        config.cpp \
        vector_property.cpp \
        quaternion_property.cpp \
        status_property.cpp \
        load_resource.cpp \
        tf_link_updater.cpp \
        coordinate_transform.cpp \
        panelview.cpp \
        mycustomslider.cpp \
        frame_manager.cpp 

HEADERS += \
        mainwindow.h \
        robot_link.h \
        robot.h \
        stl_loader.h \
        retriever.h \
        robot_joint.h \
        property.h \
        property_tree_model.h \
        config.h \
        vector_property.h \
        quaternion_property.h \
        link_updater.h \
        status_property.h \
        load_resource.h \
        tf_link_updater.h \
        coordinate_transform.h \
        panelview.h \
        mycustomslider.h \
        frame_manager.h 

FORMS += \
        mainwindow.ui
