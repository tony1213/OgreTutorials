/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include "robot.h"
#include "robot_link.h"
#include "robot_joint.h"

#include <urdf_model/model.h>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreEntity.h>
#include <OgreMaterialManager.h>
#include <OgreMaterial.h>
#include <OgreResourceGroupManager.h>
#include <OgreMatrix4.h>
#include <tinyxml.h>

//#include "frame_manager.h"
#include "tf_link_updater.h"

/*
void linkUpdaterStatusFunction( StatusProperty::Level level,
                                const std::string& link_name,
                                const std::string& text,
                                Robot* display )
{
  display->setStatus( level, QString::fromStdString( link_name ), QString::fromStdString( text ));
}

*/

Robot::Robot( Ogre::SceneNode* root_node, Ogre::SceneManager* sceneManger, const std::string& name)
  : visible_( true )
  , visual_visible_( true )
  , name_( name )
{

    root_visual_node_ = root_node->createChildSceneNode();
    root_other_node_ = root_node->createChildSceneNode();
    scene_manager_ = sceneManger;
    link_factory_ = new LinkFactory();

     setVisualVisible( visual_visible_ );
     setAlpha(1.0f);


}


Robot::~Robot()
{
  clear();
  scene_manager_->destroySceneNode(root_visual_node_);
  scene_manager_->destroySceneNode( root_other_node_->getName() );
  delete link_factory_;
  delete frame_manager_;
  frame_manager_ = NULL; 
  link_factory_ = NULL; 
}

void Robot::clear()
{
  // unparent all link and joint properties so they can be deleted in arbitrary
  // order without being delete by their parent propeties (which vary based on
  // style)

  M_NameToLink::iterator link_it = links_.begin();
  M_NameToLink::iterator link_end = links_.end();
  for ( ; link_it != link_end; ++link_it )
  {
    RobotLink* link = link_it->second;
    delete link;
  }

  M_NameToJoint::iterator joint_it = joints_.begin();
  M_NameToJoint::iterator joint_end = joints_.end();
  for ( ; joint_it != joint_end; ++joint_it )
  {
    RobotJoint* joint = joint_it->second;
    delete joint;
  }

  links_.clear();
  joints_.clear();
  root_visual_node_->removeAndDestroyAllChildren();
  root_other_node_->removeAndDestroyAllChildren();
}


void Robot::initFrameManager(){
    // sleep(10);
     qDebug(">>>>initFrameManager 1");
     frame_manager_ = new FrameManager(NULL);
     pointtf_ = new CoordinateTransform();
     qDebug(">>>>initFrameManager 2");
     frame_manager_->setFixedFrame("/base_link");
     pointtf_->setFixedFrame("/base_link");

     qDebug(">>>>initFrameManager 3");

}
void  Robot::updateRobot(){

    M_NameToLink::iterator link_it = links_.begin();
    M_NameToLink::iterator link_end = links_.end();
    for ( ; link_it != link_end; ++link_it )
    {
        RobotLink* link = link_it->second;
        Ogre::Vector3 visual_position, collision_position;
        Ogre::Quaternion visual_orientation, collision_orientation;
        visual_position = link->getPosition();
        visual_orientation = link->getOrientation();

        {
//            if(link->getName() == "Head"){
//            }else if (link->getName() == "RAnklePitch") {
//                link->setTransforms(Ogre::Vector3(-3.5,-31,-1),visual_orientation, collision_position, collision_orientation);
//            }else {
                link->setTransforms( visual_position, Ogre::Quaternion(1,-1,0,0), collision_position, collision_orientation );
                link->setOriginalPosition(visual_position);
                link->setOriginalOrientation(Ogre::Quaternion(1,-1,0,0));  //Quaternion (Real fW, Real fX, Real fY, Real fZ)
//            }

           // Ogre::Quaternion visual_orientation2 = link->getOrientation();
           // qDebug("updateRobot: link name: %s, link orientation is:: (%f,%f,%f)",link->getName().c_str(),visual_orientation2.x,visual_orientation2.y,visual_orientation2.z);

            std::vector<std::string>::const_iterator joint_it = link->getChildJointNames().begin();
            std::vector<std::string>::const_iterator joint_end = link->getChildJointNames().end();

            for ( ; joint_it != joint_end ; ++joint_it )
            {
            RobotJoint *joint = getJoint(*joint_it);
            if (joint)
            {
                joint->setTransforms(visual_position, visual_orientation);
            }
            }
        }



    }

}


/**compute the quaternion....*/
Ogre::Quaternion Robot::quaternion_from_euler(float roll, float pitch, float yaw){

    Ogre::Quaternion orientation ;

    float cx = cos(roll*0.5f);  
    float sx = sin(roll*0.5f);  
    float cy = cos(pitch*0.5f);  
    float sy = sin(pitch*0.5f);  
    float cz = cos(yaw*0.5f);  
    float sz = sin(yaw*0.5f);  
   
    orientation.w = cx*cy*cz + sx*sy*sz;  
    orientation.x = sx*cy*cz - cx*sy*sz;  
    orientation.y = cx*sy*cz + sx*cy*sz;  
    orientation.z = cx*cy*sz - sx*sy*cz; 


    return orientation; 

}


void Robot::local2World(Ogre::Vector3 locP, Ogre::Vector3 &worldP, Ogre::SceneNode * node){
/*
    Ogre::Matrix4 worldMat;  
    node->getWorldTransforms(&worldMat);
    worldP = worldMat * locP;  
*/
}
/*
bool Robot::world2Screen(Vector3 objPos, Vector2& screenPos){

    Matrix4 viewMat = mCamera->getViewMatrix();  
    Matrix4 projMat = mCamera->getProjectionMatrix();  
  
  
    Vector4 inP = Vector4(objPos.x, objPos.y, objPos.z ,1.0);  
    Vector4 outP = viewMat * inP;  
    outP = projMat * outP;  
  
    if(outP.w <= EPSILON)  
        return false;  
  
    outP.x /= outP.w;  
    outP.y /= outP.w;  
    outP.z /= outP.w;  
  
       //[-1,1]->[0,1]  
    outP.x = outP.x*0.5 + 0.5;  
    outP.y = outP.y*0.5 + 0.5;  
    outP.z = outP.z*0.5 + 0.5;  
  
    outP.x = outP.x * mWindow->getWidth();  
    outP.y = (1-outP.y) * mWindow->getHeight();  
  
    screenPos.x = outP.x;  
    screenPos.y = outP.y;  
    return true;  

}

*/

void Robot::update(const LinkUpdater& updater, const std::string& jonitname, int value){


    RobotJoint *joint = getJoint(jonitname);
    std::string linkname = joint->getChildLinkName(); 

    

    M_NameToLink::iterator link_it = links_.begin();
    M_NameToLink::iterator link_end = links_.end();
    for ( ; link_it != link_end; ++link_it )
    {
        RobotLink* link = link_it->second;
        Ogre::Vector3 visual_position, collision_position, curPos;
        Ogre::Quaternion visual_orientation, collision_orientation, curOrientation;


         visual_position = link->getOriginalPosition();
         visual_orientation = link->getOriginalOrientation();


        if(link != NULL){
            

                if(link->getName() == linkname){
                

                if("LShoulderRoll" == linkname){

                 // qDebug("************************************chenrui****************************");
                  curPos = link->getPosition();
                  curOrientation = link->getOrientation(); 
                  qDebug("******current relative position***********link name: %s, link pos is:: (%f,%f,%f)",linkname.c_str(),curPos.x,curPos.y,curPos.z);
                  curPos = link->getWorldPosition();
                  curOrientation = link->getWorldOrientation();

                  qDebug("*****world position************link name: %s, link pos is:: (%f,%f,%f)",linkname.c_str(),curPos.x,curPos.y,curPos.z); 

                  float roll  = -1.57;
                  float pitch = 0;
                  float yaw   = 1.57;

                  visual_position.x = 30;
                  visual_position.y = 18;
                  visual_position.z = 0;

                //yaw is 1.57, then x,y z is: 5, 18, -20; 
                //roll 1,57, pitch 0, yaw -1.57;


                 // float roll  = -1.57;
                 // float pitch = 0;
                 // float yaw   = 1.57;

                  visual_orientation = quaternion_from_euler(roll,pitch, yaw);



                 // qDebug(link->getName().c_str()) ;
                 //Ogre::Quaternion(1,-1,0,0)
                 // Ogre::Quaternion rQuaternion = joint->getOrientation();
                 // qDebug("link name: %s, link pos is:: (%f,%f,%f)",linkname.c_str(),visual_position.x,visual_position.y,visual_position.z);   

                  double percent = (double)value/130;

                  RobotLink * parentLink = joint->getParentLink();
                  if(parentLink != NULL){

                     // qDebug(">>>>>>>>>>>>>>*******************will modify parameter>>>>chenrui");
                    //  Ogre::Vector3 parposition = parentLink->getPosition();
                    //  Ogre::Quaternion parorientation = parentLink->getOrientation();  

                     // qDebug("parentLink name: %s, parentLink pos is:: (%f,%f,%f)",parentLink->getName().c_str(),parposition.x,parposition.y,parposition.z);      
                     // joint->setOrientation(visual_orientation);
                     // joint->setPosition(parposition);
                     // joint->setTransforms(parposition, parorientation);
                    //  visual_position = joint->getPosition();
                    //  visual_orientation = joint->getOrientation();

                  } 

                                
               
               // qDebug("LShoulderRoll link orientation x y z w is:: (%f,%f,%f,%f)",visual_orientation.x,visual_orientation.y,visual_orientation.z, visual_orientation.w);      

               }else if("LShoulderPitch" == linkname){

                  double percent = (double)value/130;

                  visual_position.x = visual_position.x + percent ;
                  visual_position.y = visual_position.y + percent;
                  visual_position.z = visual_position.z + percent;

               }else if("LElbow" == linkname){
                  double percent = (double)value/130;

                  visual_position.x = visual_position.x + percent ;
                  visual_position.y = visual_position.y + percent;
                  visual_position.z = visual_position.z + percent;

               }else if("LHipRoll" == linkname){
                  double percent = (double)value/130;

                  visual_position.x = visual_position.x + percent ;
                  visual_position.y = visual_position.y + percent;
                  visual_position.z = visual_position.z + percent;

               }else if("LHipPitch" == linkname){

                 double percent = (double)value/130;

                  visual_position.x = visual_position.x + percent ;
                  visual_position.y = visual_position.y + percent;
                  visual_position.z = visual_position.z + percent;
               }else if("LKnee" == linkname){
                  double percent = (double)value/130;

                  visual_position.x = visual_position.x + percent ;
                  visual_position.y = visual_position.y + percent;
                  visual_position.z = visual_position.z + percent;

              }else {

                  double percent = (double)value/130;

                  visual_position.x = visual_position.x + percent ;
                  visual_position.y = visual_position.y + percent;
                  visual_position.z = visual_position.z + percent;


              }

          }
            
        }
        if(link != NULL /*   && updater.getLinkTransforms( link->getName(),
                                   visual_position, visual_orientation,
                                   collision_position, collision_orientation
                                   )  */ )

        {

   
              
            link->setTransforms( visual_position, visual_orientation, collision_position, collision_orientation );

            if(link->getName() == "LShoulderRoll"){
                qDebug("-----link setTransforms ");
                visual_position = link->getPosition();//only for test ...
                qDebug("after edit, LShoulderRoll link pos is:: (%f,%f,%f)",visual_position.x,visual_position.y,visual_position.z);
               // Ogre::Quaternion qTest( Ogre::Degree( -90 ), Ogre::Vector3::UNIT_Y );
               // link->rotate(qTest);
            }

            std::vector<std::string>::const_iterator joint_it = link->getChildJointNames().begin();
            std::vector<std::string>::const_iterator joint_end = link->getChildJointNames().end();

            for ( ; joint_it != joint_end ; ++joint_it )
            {
            RobotJoint *joint = getJoint(*joint_it);
            if (joint)
            {
                joint->setTransforms(visual_position, visual_orientation);
                if(joint->getName() == "LShoulderPitch_joint"){ 
                    
                    qDebug(">>>>>current joint is: LShoulderPitch_joint");
                    
                    Ogre::Vector3 position = joint->getPosition();
                    Ogre::Quaternion orientation = joint->getOrientation(); 
                    std::string curlinkname = joint->getChildLinkName();
                    RobotLink *childLink = getLink(curlinkname);
                    childLink->setOrientation(orientation);
                    childLink->setPosition(position);
                    qDebug("LShoulderPitch_joint link pos is:: (%f,%f,%f)",position.x,position.y,position.z);
                    RobotJoint *joint2 = getJoint("LElbow_joint");
                    joint2->setTransforms(position, orientation);

                    Ogre::Vector3 position2 = joint2->getPosition();
                    Ogre::Quaternion orientation2 = joint2->getOrientation();
                    RobotLink *childLink2 = getLink("LElbow");
                    childLink2->setOrientation(orientation2);
                    childLink2->setPosition(position2);
                                       
                    
                } 
            }
            }
        }


        }
       mRoot->renderOneFrame();

}

/** update the link position according to panel view*/
void Robot::updateRobot(const std::string& linkname, int value){

   // update( TFLinkUpdater(frame_manager_, NULL, "" ), linkname, value);  //chenrui
    update( TFLinkUpdater(pointtf_, NULL, "" ), linkname, value);  //chenrui
   

}
RobotLink* Robot::LinkFactory::createLink(
    Robot* robot,
    Ogre::SceneManager* sceneManger ,
    const urdf::LinkConstSharedPtr& link,
    const std::string& parent_joint_name,
    bool visual,
    bool collision)
{
  return new RobotLink(robot, sceneManger, link, parent_joint_name, visual, collision);
}

RobotJoint* Robot::LinkFactory::createJoint(
    Robot* robot,
    const urdf::JointConstSharedPtr& joint)
{
  return new RobotJoint(robot, joint);
}


RobotLink* Robot::getLink( const std::string& name )
{
  M_NameToLink::iterator it = links_.find( name );
  if ( it == links_.end() )
  {
    qDebug(">>>>>Robot::getLink failed");
    return NULL;
  }

  return it->second;
}

RobotJoint* Robot::getJoint( const std::string& name )
{
  M_NameToJoint::iterator it = joints_.find( name );
  if ( it == joints_.end() )
  {
    return NULL;
  }

  return it->second;
}


bool Robot::styleShowLink(LinkTreeStyle style)
{
  return
    style == STYLE_LINK_LIST ||
    style == STYLE_LINK_TREE ||
    style == STYLE_JOINT_LINK_TREE;
}

bool Robot::styleShowJoint(LinkTreeStyle style)
{
  return
    style == STYLE_JOINT_LIST ||
    style == STYLE_JOINT_LINK_TREE;
}

bool Robot::styleIsTree(LinkTreeStyle style)
{
  return
    style == STYLE_LINK_TREE ||
    style == STYLE_JOINT_LINK_TREE;
}


// recursive helper for setLinkTreeStyle() when style is *_TREE
void Robot::addLinkToLinkTree(LinkTreeStyle style, Property *parent,  RobotLink *link)
{

  if (styleShowLink(style))
  {
    link->setParentProperty(parent);
    parent = link->getLinkProperty();
  }

  std::vector<std::string>::const_iterator child_joint_it = link->getChildJointNames().begin();
  std::vector<std::string>::const_iterator child_joint_end = link->getChildJointNames().end();
  for ( ; child_joint_it != child_joint_end ; ++child_joint_it )
  {
    RobotJoint* child_joint = getJoint( *child_joint_it );
    if (child_joint)
    {
      addJointToLinkTree(style, parent,  child_joint);
    }
  }
}




// recursive helper for setLinkTreeStyle() when style is *_TREE
void Robot::addJointToLinkTree(LinkTreeStyle style,  Property *parent,  RobotJoint *joint)
{

  if (styleShowJoint(style))
  {
    joint->setParentProperty(parent);
    parent = joint->getJointProperty();
    //joint->setJointPropertyDescription();
  }

  RobotLink *link = getLink( joint->getChildLinkName() );
  if (link)
  {
    addLinkToLinkTree(style, parent,  link);
  }
}

void Robot::setOgreRoot(Ogre::Root* root)
{
    mRoot = root; 
}

void Robot::setAlpha(float a)
{
  alpha_ = a;

  M_NameToLink::iterator it = links_.begin();
  M_NameToLink::iterator end = links_.end();
  for ( ; it != end; ++it )
  {
    RobotLink* link = it->second;

    link->setRobotAlpha(alpha_);
  }
}



/** void RobotModelDisplay::load() **
* robot_description_ is: urdf file name...
***/
void Robot::load( std::string robot_description_ ,/* const urdf::ModelInterface &urdf, */ bool visual, bool collision ){

   TiXmlDocument doc;
   doc.LoadFile(robot_description_);

   if( !doc.RootElement() )
   {
    qDebug("URDF failed XML parse");
    return;
  }
  

  urdf::Model urdf;
  if( !urdf.initXml( doc.RootElement() ))
  {
    //setStatus( StatusProperty::Error, "URDF", "URDF failed Model parse" );
    qDebug("URDF failed Model parse");
    return;
  }
  qDebug("URDF parsed OK" );
  clear();
  //using descr ; the descr is 
  typedef std::map<std::string, urdf::LinkSharedPtr > M_NameToUrdfLink;
  M_NameToUrdfLink::const_iterator link_it = urdf.links_.begin();
  M_NameToUrdfLink::const_iterator link_end = urdf.links_.end();

  initFrameManager();

  for( ; link_it != link_end; ++link_it )
  {
      const urdf::LinkConstSharedPtr& urdf_link = link_it->second;
      std::string parent_joint_name;

      if (urdf_link != urdf.getRoot() && urdf_link->parent_joint)
      {
        parent_joint_name = urdf_link->parent_joint->name;
      }
      RobotLink* link = link_factory_->createLink( this,
                                                   scene_manager_,
                                                   urdf_link,
                                                   parent_joint_name,
                                                   visual,
                                                   collision );
      if (urdf_link == urdf.getRoot())
      {
        root_link_ = link;
      }
      links_[urdf_link->name] = link;
}

  // Create properties for each joint.
  // Properties are not added to display until changedLinkTreeStyle() is called (below).
  {
    typedef std::map<std::string, urdf::JointSharedPtr > M_NameToUrdfJoint;
    M_NameToUrdfJoint::const_iterator joint_it = urdf.joints_.begin();
    M_NameToUrdfJoint::const_iterator joint_end = urdf.joints_.end();
    for( ; joint_it != joint_end; ++joint_it )
    {
      const urdf::JointConstSharedPtr& urdf_joint = joint_it->second;
      RobotJoint* joint = link_factory_->createJoint( this, urdf_joint );

      joints_[urdf_joint->name] = joint;

      joint->setRobotAlpha( alpha_ );
    }
  }
}



void Robot::setVisible( bool visible )
{
  visible_ = visible;
  if ( visible )
  {
    root_visual_node_->setVisible( visual_visible_ );
   // root_collision_node_->setVisible( collision_visible_ );
    updateLinkVisibilities();
  }
  else
  {
    root_visual_node_->setVisible( false );
   // root_collision_node_->setVisible( false );
    updateLinkVisibilities();
  }
}


void Robot::setVisualVisible( bool visible )
{
  visual_visible_ = visible;
  updateLinkVisibilities();
}

void Robot::updateLinkVisibilities()
{
  M_NameToLink::iterator it = links_.begin();
  M_NameToLink::iterator end = links_.end();
  for ( ; it != end; ++it )
  {
    RobotLink* link = it->second;
    //link->updateVisibility();
  }
}

bool Robot::isVisible()
{
  return visible_;
}

bool Robot::isVisualVisible()
{
  return visual_visible_;
}


void Robot::setPosition( const Ogre::Vector3& position )
{
  root_visual_node_->setPosition( position );
}

void Robot::setOrientation( const Ogre::Quaternion& orientation )
{
  root_visual_node_->setOrientation( orientation );
}

void Robot::setScale( const Ogre::Vector3& scale )
{
  root_visual_node_->setScale( scale );
}

const Ogre::Vector3& Robot::getPosition()
{
  return root_visual_node_->getPosition();
}

const Ogre::Quaternion& Robot::getOrientation()
{
  return root_visual_node_->getOrientation();
}





