#include "mainwindow.h"
#include <QX11Info>
#include <QDebug>

OgreView::OgreView(QWidget* parent) : QWidget(parent,Qt::WindowFlags(Qt::MSWindowsOwnDC))
{
    mRenderWindow = NULL;
    mSceneManager = NULL;
    mViewport = NULL;
    mainEntity = NULL;
    mSceneNode = NULL;
    rotX = 1;
    rotY = 1;
    mZoom = 1;
    mouseLeftPressed = false;
    mouseRightPressed = false;
    mouseMiddleBtn = false;

    //TODO
    mRoot = new Ogre::Root("/home/tony/work/ogre/tutorials/OgreTutorials/media/plugins.cfg");
    setupRenderSystem();
    mRoot->initialise(false);
    setupResources();
}

OgreView::~OgreView()
{
    if( mViewport )
    {
        mRenderWindow->removeViewport(mViewport->getZOrder());
        mViewport = 0;
    }
    Ogre::Root::getSingleton().detachRenderTarget(mRenderWindow);
    mRenderWindow = 0;

    if(mRoot != NULL)
    {
        delete mRoot;
        mRoot = 0;
    }
}

void OgreView::setupRenderSystem(){
    qDebug("setupResources");
    const Ogre::RenderSystemList *rsList;
#if OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR == 6
    rsList = mRoot->getAvailableRenderers();
#else
    rsList = &(mRoot->getAvailableRenderers());
#endif
    mRenderSystem = NULL;
    for(unsigned int i=0;i<rsList->size();i++ )
    {
        mRenderSystem = rsList->at( i );
        if(mRenderSystem->getName().compare("OpenGL Rendering Subsystem")== 0 )
        {
            break;
        }
    }
    if( mRenderSystem == NULL )
    {
        throw std::runtime_error( "Could not find the opengl rendering subsystem!\n" );
    }
    mRenderSystem->setConfigOption("Full Screen","No");
    mRoot->setRenderSystem(mRenderSystem);
}

void OgreView::setupResources()
{
    qDebug("setupResources");
    Ogre::ConfigFile cf;
    //TODO
    cf.load("/home/tony/work/ogre/tutorials/OgreTutorials/media/resources.cfg");
    Ogre::ConfigFile::SectionIterator seci = cf.getSectionIterator();
    Ogre::String secName, typeName, archName;
    while(seci.hasMoreElements())
    {
        secName = seci.peekNextKey();
        Ogre::ConfigFile::SettingsMultiMap* settings = seci.getNext();
        Ogre::ConfigFile::SettingsMultiMap::iterator i;
        for(i=settings->begin(); i!=settings->end(); ++i)
        {
            typeName = i->first;
            archName = i->second;
            Ogre::ResourceGroupManager::getSingleton().addResourceLocation(archName, typeName, secName);
        }
    }
}

void OgreView::setupView()
{
    qDebug("setupView");
    if(mRenderWindow)
        return;
    Ogre::NameValuePairList params;
    QWidget *q_parent = dynamic_cast <QWidget *> (parent());
    params["parentWindowHandle"] = Ogre::StringConverter::toString ((unsigned long)QX11Info::display()) +
            ":" + Ogre::StringConverter::toString ((unsigned int)QX11Info::appScreen()) +
            ":" + Ogre::StringConverter::toString ((unsigned long)q_parent->winId());
    mRenderWindow = mRoot->createRenderWindow("View", width(), height(), false, &params);

    mSceneManager = mRoot->createSceneManager(Ogre::ST_GENERIC);
    mCamera = mSceneManager->createCamera("PlayerCam");
    mCamera->setPosition(Ogre::Vector3(0,0,80));
    mCamera->lookAt(Ogre::Vector3(0,0,-300));
    mCamera->setNearClipDistance(5);

    mViewport = mRenderWindow->addViewport(mCamera);
    mViewport->setBackgroundColour(Ogre::ColourValue(0.0, 0.0, 0.0, 1));

    mCamera->setAspectRatio(Ogre::Real(mViewport->getActualWidth()) / Ogre::Real(mViewport->getActualHeight()));
    createScene();
    createLight();
}

void OgreView::createScene()
{
    mainEntity = mSceneManager->createEntity("Head", "Sinbad.mesh");
    mSceneNode = mSceneManager->getRootSceneNode()->createChildSceneNode();
    mSceneNode->attachObject(mainEntity);
}

void OgreView::createLight()
{
    mSceneManager->setAmbientLight(Ogre::ColourValue(0.5, 0.5, 0.5));
    mainLight = mSceneManager->createLight("MainLight");
    mainLight->setPosition(20,80,50);
}

void OgreView::paintEvent(QPaintEvent *evt)
{
    Q_UNUSED(evt);
    if(mRenderWindow == NULL)
        setupView();
    update();
}

void OgreView::update()
{
    if(mRenderWindow != NULL)
    {
        mRoot->renderOneFrame();
    }
}

void OgreView::keyPressEvent(QKeyEvent* evt)
{
    qDebug("keypressevent");
    if(mainEntity != NULL && mSceneNode != NULL)
    {
        switch(evt->key())
        {
        case Qt::Key_W:
        case Qt::Key_Up:
        {
            qDebug("W/UP");
            rotX = -0.1;
            mSceneNode->pitch(Ogre::Radian(rotX));

            break;
        }
        case Qt::Key_S:
        case Qt::Key_Down:
        {
            qDebug("S/Down");
            rotX = 0.1;
            mSceneNode->pitch(Ogre::Radian(rotX));
            break;
        }
        case Qt::Key_A:
        case Qt::Key_Left:
        {
            qDebug("A/Left");
            rotY = -0.1;
            mSceneNode->yaw(Ogre::Radian(rotY));
            break;
        }
        case Qt::Key_D:
        case Qt::Key_Right:
        {
            qDebug("D/Right");
            rotY = 0.1;
            mSceneNode->yaw(Ogre::Radian(rotY));
            break;
        }
        }
    }
}


void OgreView::mousePressEvent(QMouseEvent* evt)
{
    qDebug("mousePressEvent");
    if(evt->button() == Qt::LeftButton)
    {
        mouseLeftPressed = true;
    }
    if(evt->button() == Qt::RightButton)
    {
        mouseRightPressed = true;
        mousePos = Ogre::Vector2(evt->x(), evt->y());
    }
    if(evt->button() == Qt::MidButton)
    {
        mouseMiddleBtn = true;
    }
}

void OgreView::mouseReleaseEvent(QMouseEvent *evt)
{
    qDebug("mouseReleaseEvent");
    if(evt->button() == Qt::LeftButton)
    {
        mouseLeftPressed = false;
    }
    if(evt->button() == Qt::RightButton)
    {
        mouseRightPressed = false;
        mousePos = Ogre::Vector2(evt->x(), evt->y());
    }
    if(evt->button() == Qt::MidButton)
    {
        mouseMiddleBtn = false;
    }
}

void OgreView::wheelEvent(QWheelEvent* evt)
{
    if(evt->delta()>0){
        mSceneNode->setPosition(0,0,mZoom++);
    }
    else{
        mSceneNode->setPosition(0,0,mZoom--);
    }
    update();
}
