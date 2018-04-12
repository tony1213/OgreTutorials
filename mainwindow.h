#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <OGRE/Ogre.h>
#include <QWidget>
#include <QKeyEvent>
#include <QMouseEvent>

class OgreView : public QWidget
{
    Q_OBJECT

public:
    OgreView(QWidget* parent );
    ~OgreView();
protected:
    void setupRenderSystem();
    void setupView();
    void update();
    void setupResources();
    void createScene();
    void createLight();

    void paintEvent(QPaintEvent* event);
    void mousePressEvent(QMouseEvent* event);
    void mouseMoveEvent(QMouseEvent* event);
    void wheelEvent(QWheelEvent* event);

    Ogre::RenderWindow* mRenderWindow;
    Ogre::RenderSystem *mRenderSystem;
    Ogre::SceneManager* mSceneManager;
    Ogre::Camera* mCamera;
    Ogre::Viewport* mViewport;
    Ogre::Root* mRoot;
    Ogre::Light* mainLight;
    Ogre::Entity* mainEntity;
    Ogre::SceneNode* mSceneNode;

    Ogre::Real mZoom;

    Ogre::Vector2 mouseRightPosOriginal;
    Ogre::Vector2 mouseRightPosNew;
    Ogre::Vector2 mouseLeftPosOriginal;
    Ogre::Vector2 mouseLeftPosNew;
};
#endif // MAINWINDOW_H
