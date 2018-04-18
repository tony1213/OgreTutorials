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


Robot::Robot( Ogre::SceneNode* root_node, Ogre::SceneManager* sceneManger, const std::string& name)
{

    root_visual_node_ = root_node->createChildSceneNode();
    scene_manager_ = sceneManger;
    link_factory_ = new LinkFactory();


}


Robot::~Robot()
{
  scene_manager_->destroySceneNode(root_visual_node_);
  delete link_factory_;
  link_factory_ = NULL; 
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

// recursive helper for setLinkTreeStyle() when style is *_TREE
void Robot::addLinkToLinkTree(LinkTreeStyle style,/* Property *parent, */ RobotLink *link)
{
/*
  if (styleShowLink(style))
  {
    link->setParentProperty(parent);
    parent = link->getLinkProperty();
  }
*/
  std::vector<std::string>::const_iterator child_joint_it = link->getChildJointNames().begin();
  std::vector<std::string>::const_iterator child_joint_end = link->getChildJointNames().end();
  for ( ; child_joint_it != child_joint_end ; ++child_joint_it )
  {
    RobotJoint* child_joint = getJoint( *child_joint_it );
    if (child_joint)
    {
      addJointToLinkTree(style,/* parent, */ child_joint);
    }
  }
}




// recursive helper for setLinkTreeStyle() when style is *_TREE
void Robot::addJointToLinkTree(LinkTreeStyle style, /* Property *parent, */ RobotJoint *joint)
{
/*
  if (styleShowJoint(style))
  {
    joint->setParentProperty(parent);
    parent = joint->getJointProperty();
    joint->setJointPropertyDescription();
  }
*/
  RobotLink *link = getLink( joint->getChildLinkName() );
  if (link)
  {
    addLinkToLinkTree(style,/* parent, */ link);
  }
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
  //using descr ; the descr is 
  typedef std::map<std::string, urdf::LinkSharedPtr > M_NameToUrdfLink;
  M_NameToUrdfLink::const_iterator link_it = urdf.links_.begin();
  M_NameToUrdfLink::const_iterator link_end = urdf.links_.end();

  for( ; link_it != link_end; ++link_it )
  {
      qDebug("chenrui will create link ....");
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







