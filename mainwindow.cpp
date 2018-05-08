#include "mainwindow.h"
#include <QX11Info>
#include <QDebug>
#include <QSlider>
#include <QLabel>


#include "robot.h"
#include "panelview.h"

OgreView::OgreView(QWidget* parent)
 :QWidget(parent,Qt::WindowFlags(Qt::MSWindowsOwnDC)),
 mPanelView(NULL)
{
    mRenderWindow = NULL;
    mSceneManager = NULL;
    mViewport = NULL;
    mainEntity = NULL;
    mSceneNode = NULL;

    mZoom = 1;

    //TODO
    mRoot = new Ogre::Root("/home/chenrui/new_Ogre_git/OgreTutorials/Media/plugins.cfg");
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
    cf.load("/home/chenrui/new_Ogre_git/OgreTutorials/Media/resources.cfg");
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
    Ogre::ResourceGroupManager::getSingleton().initialiseResourceGroup(secName);
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
   // mRenderWindow = mRoot->createRenderWindow("View", width(), height(), false, &params);
     mRenderWindow = mRoot->createRenderWindow("View", 1000, height(), false, &params);


     qDebug("setupView>>>>will create PanelView");
     mPanelView = new PanelView(q_parent);
     mPanelView->setGeometry(1000, 0,650,height());
     mPanelView->show();
     qDebug("setupView>>>>after: mPanelView->show();");


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
    //mainEntity = mSceneManager->createEntity("Head", "Sinbad.mesh");
    mSceneNode = mSceneManager->getRootSceneNode()->createChildSceneNode();
    //mSceneNode->attachObject(mainEntity);
   
    //here, we will create node tree using  robot.h
    robot_ = new Robot(mSceneNode, mSceneManager,  "SimulatorRobot ");
    robot_->load("/home/chenrui/new_Ogre_git/OgreTutorials/urdf/H3.urdf",true,false);
    robot_->updateRobot();
    mPanelView->setRobot(robot_);
}

void OgreView::createLight()
{
    mSceneManager->setAmbientLight(Ogre::ColourValue(0.5, 0.5, 0.5));
    mainLight = mSceneManager->createLight("MainLight");
    mainLight->setPosition(20,80,50);
}

void OgreView::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event);
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
    
    mPanelView->show();

}

void OgreView::mousePressEvent(QMouseEvent* event)
{
    qDebug("mousePressEvent");
    if(event->button() == Qt::LeftButton)
    {
        mouseLeftPosOriginal = Ogre::Vector2(event->x(), event->y());
    }
    if(event->button() == Qt::RightButton){
        mouseRightPosOriginal = Ogre::Vector2(event->x(), event->y());
    }
    robot_->updateRobot();//chenrui add for test
}

void OgreView::mouseMoveEvent(QMouseEvent *event){
    qDebug()<<"mouseMoveEvent:";
    if(event->buttons() & Qt::RightButton){
        mouseRightPosNew = Ogre::Vector2(event->x(), event->y());
        mZoom = mouseRightPosNew.x - mouseRightPosOriginal.x;
        mSceneNode->setPosition(0,0,mZoom);
    }
    if(event->buttons() & Qt::LeftButton){
        mouseLeftPosNew = Ogre::Vector2(event->x(), event->y());
        Ogre::Real yawValue = mouseLeftPosNew.x-mouseLeftPosOriginal.x;
        mSceneNode->yaw(Ogre::Degree(yawValue));
        mouseLeftPosOriginal = mouseLeftPosNew;
    }
    update();
}

void OgreView::wheelEvent(QWheelEvent* event)
{
    if(event->delta()>0){
        mSceneNode->setPosition(0,0,mZoom++);
    }
    else{
        mSceneNode->setPosition(0,0,mZoom--);
    }
    update();
}
