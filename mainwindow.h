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
    void setupView();//创建Ogre使用的内容，场景内容;
    void update();//手动更新场景内容
    void setupResources();//
    void createScene();
    void createLight();
    void paintEvent(QPaintEvent* evt);
    //消息响应
    void keyPressEvent(QKeyEvent* evt);
    void mousePressEvent(QMouseEvent* evt);
    void mouseReleaseEvent(QMouseEvent* evt);
    void wheelEvent(QWheelEvent* evt);

    //有关Ogre
    Ogre::RenderWindow* mRenderWindow;
    Ogre::RenderSystem *mRenderSystem;
    Ogre::SceneManager* mSceneManager;
    Ogre::Camera* mCamera;
    Ogre::Viewport* mViewport;
    Ogre::Root* mRoot;
    Ogre::Light* mainLight;
    Ogre::Entity* mainEntity;
    Ogre::SceneNode* mSceneNode;
    Ogre::Real angleX;
    Ogre::Real angleY;
    Ogre::Real rotX;
    Ogre::Real rotY;
    Ogre::Real mZoom;
    Ogre::Real mRotate;
    Ogre::Vector2 mousePos;
    Ogre::Vector3 mDirection;

    bool mouseLeftPressed;
    bool mouseRightPressed;
    bool mouseMiddleBtn;

};
#endif // MAINWINDOW_H
