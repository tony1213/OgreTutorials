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

    void paintEvent(QPaintEvent* evt);
    void keyPressEvent(QKeyEvent* evt);
    void mousePressEvent(QMouseEvent* evt);
    void mouseReleaseEvent(QMouseEvent* evt);
    void wheelEvent(QWheelEvent* evt);

    Ogre::RenderWindow* mRenderWindow;
    Ogre::RenderSystem *mRenderSystem;
    Ogre::SceneManager* mSceneManager;
    Ogre::Camera* mCamera;
    Ogre::Viewport* mViewport;
    Ogre::Root* mRoot;
    Ogre::Light* mainLight;
    Ogre::Entity* mainEntity;
    Ogre::SceneNode* mSceneNode;
    Ogre::Real rotX;
    Ogre::Real rotY;
    Ogre::Real mZoom;
    Ogre::Vector2 mousePos;
    Ogre::Vector3 mDirection;

    bool mouseLeftPressed;
    bool mouseRightPressed;
    bool mouseMiddleBtn;

};
#endif // MAINWINDOW_H
