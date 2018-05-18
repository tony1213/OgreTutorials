#include <QApplication>
#include <QWidget>
#include "mainwindow.h"
#include <QVBoxLayout>
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    QWidget window;

    window.resize(1600, 1200);
    window.setWindowTitle("Simple example");

    ros::init(argc, argv, "test");

    OgreView* ogreWidget = new OgreView(&window);

    QVBoxLayout *layout = new QVBoxLayout;
    layout->addWidget(ogreWidget);
    window.setLayout(layout);
    window.show();

    return app.exec();
}
