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
#include <tinyxml.h>

//#include "frame_manager.h"
#include "tf_link_updater.h"


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
//                link->setTransforms(Ogre::Vector3(0,10.16,-0.6),Ogre::Quaternion(1,0,0,0), collision_position, collision_orientation);
//            }else if (link->getName() == "LElbow") {
//                link->setTransforms(Ogre::Vector3(3.0,0,-0.65),visual_orientation, collision_position, collision_orientation);
//            }else if (link->getName() == "RElbow") {
//                link->setTransforms(Ogre::Vector3(-3.0,0,-0.65),visual_orientation, collision_position, collision_orientation);
//            }else if (link->getName() == "LHipYaw") {
//                link->setTransforms(Ogre::Vector3(3.6,-3.5,0),Ogre::Quaternion(1,0,0,0), collision_position, collision_orientation);
//            }else if (link->getName() == "RHipYaw") {
//                link->setTransforms(Ogre::Vector3(-3.6,-3.5,0),visual_orientation, collision_position, collision_orientation);
//            }else if (link->getName() == "LAnklePitch") {
//                link->setTransforms(Ogre::Vector3(3.5,-31,-1),visual_orientation, collision_position, collision_orientation);
//            }else if (link->getName() == "RAnklePitch") {
//                link->setTransforms(Ogre::Vector3(-3.5,-31,-1),visual_orientation, collision_position, collision_orientation);
//            }else {
                link->setTransforms( visual_position, Ogre::Quaternion(1,-1,0,0), collision_position, collision_orientation );
//            }

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

void Robot::update(const LinkUpdater& updater, const std::string& linkname, int value){

    M_NameToLink::iterator link_it = links_.begin();
    M_NameToLink::iterator link_end = links_.end();
    for ( ; link_it != link_end; ++link_it )
    {
        RobotLink* link = link_it->second;
        Ogre::Vector3 visual_position, collision_position;
        Ogre::Quaternion visual_orientation, collision_orientation;

        visual_position = link->getPosition();
        visual_orientation = link->getOrientation(); 
        if(link != NULL){
           // qDebug("will >>>>>>update one link");
          //  qDebug(link->getName().c_str()) ; //chenrui
            
            if(link->getName() == linkname){
                qDebug("************************************chenrui****************************");
                qDebug(link->getName().c_str()) ;

            //    visual_position.x = visual_position.x + 200;
            //    visual_position.y = visual_position.y + 100;
            //    visual_position.z = visual_position.z + 0;
            }
            
        }
        if(link != NULL  && updater.getLinkTransforms( link->getName(),
                                   visual_position, visual_orientation,
                                   collision_position, collision_orientation
                                   ))

        {

         
            qDebug("-----link setTransforms ");
            visual_position.x = visual_position.x + 20;
            visual_position.y = visual_position.y + 20;
            visual_position.z = visual_position.z + 20;
   
            link->setTransforms( visual_position, visual_orientation, collision_position, collision_orientation );

            std::vector<std::string>::const_iterator joint_it = link->getChildJointNames().begin();
            std::vector<std::string>::const_iterator joint_end = link->getChildJointNames().end();

            for ( ; joint_it != joint_end ; ++joint_it )
            {
            RobotJoint *joint = getJoint(*joint_it);
            //qDebug("jonit name is: " + joint->getName()) ; //chenrui
            if (joint)
            {
                qDebug("-----jonit name is: ");
                qDebug(joint->getName().c_str()) ; //chenrui
                joint->setTransforms(visual_position, visual_orientation);
            }
            }
        }


        }
       mRoot->renderOneFrame();

       //need to call: ogre_root_->renderOneFrame();
}

/** update the link position according to panel view*/
void Robot::updateRobot(const std::string& linkname, int value){

    update( TFLinkUpdater(frame_manager_, NULL, "" ), linkname, value);

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

     frame_manager_ = new FrameManager(NULL);
    // frame_manager_->setFixedFrame("/Torso"); 
     frame_manager_->setFixedFrame("/base_link");
     //frame_manager_->setFixedFrame("/world");
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





