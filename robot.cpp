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
#include <OgreVector4.h>
#include <pthread.h>
#include <QTimer>

//#include "frame_manager.h"
#include "tf_link_updater.h"



#define EPSILON 0.000001


void linkUpdaterStatusFunction( StatusProperty::Level level,
                                const std::string& link_name,
                                const std::string& text,
                                Robot* display )
{
    qDebug(">>>>>link_name and text is: %s, %s", link_name.c_str(), text.c_str());  
}



Robot::Robot( Ogre::SceneNode* root_node, Ogre::SceneManager* sceneManger, const std::string& name)
  : visible_( true )
  , visual_visible_( true )
  , name_( name )
{

    ros::NodeHandle n_tilde("~");

    updateTf = false; 
    root_visual_node_ = root_node->createChildSceneNode();
    root_other_node_ = root_node->createChildSceneNode();
    scene_manager_ = sceneManger;
    link_factory_ = new LinkFactory();

    setVisualVisible( visual_visible_ );
    setAlpha(1.0f);


    update_timer_ = new QTimer;
    connect( update_timer_, SIGNAL( timeout() ), this, SLOT( onUpdate() ));
    last_update_ros_time_ = ros::Time::now();
    last_update_wall_time_ = ros::WallTime::now();
    time_update_timer_ = 0.0f;
    frame_update_timer_ = 0.0f; 
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
  
  update_timer_->stop();
  delete update_timer_; 
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

void Robot::resetTime()
{
 // root_display_group_->reset();  //???chenrui
  frame_manager_->getTFClient()->clear();

  ros_time_begin_ = ros::Time();
  wall_clock_begin_ = ros::WallTime();

 // queueRender();
}

void Robot::updateTime(){

    if( ros_time_begin_.isZero() )
  {
    ros_time_begin_ = ros::Time::now();
  }

  ros_time_elapsed_ = ros::Time::now() - ros_time_begin_;

  if( wall_clock_begin_.isZero() )
  {
    wall_clock_begin_ = ros::WallTime::now();
  }

  wall_clock_elapsed_ = ros::WallTime::now() - wall_clock_begin_;


}


void Robot::updateFrames(){

    typedef std::vector<std::string> V_string;
    V_string frames;
    frame_manager_->getTFClient()->getFrameStrings( frames );

    // Check the fixed frame to see if it's ok
    std::string error;
    if( frame_manager_->frameHasProblems(frame_manager_->getFixedFrame(), ros::Time(), error ))
    {
        if( frames.empty() )
        {
        // fixed_prop->setToWarn();
        }
        else
        {
        }
    }
    else
    {
        // fixed_prop->setToOK();
    }


}
void Robot::onUpdate(){


    ros::WallDuration wall_diff = ros::WallTime::now() - last_update_wall_time_;
    ros::Duration ros_diff = ros::Time::now() - last_update_ros_time_;
    float wall_dt = wall_diff.toSec();
    float ros_dt = ros_diff.toSec();
    last_update_ros_time_ = ros::Time::now();
    last_update_wall_time_ = ros::WallTime::now();
      
    if(ros_dt < 0.0)
    {
        resetTime();
    }

    ros::spinOnce(); 
    frame_manager_->update();
    update(TFLinkUpdater(frame_manager_, boost::bind(linkUpdaterStatusFunction, _1, _2, _3, this), "" ));

    time_update_timer_ += wall_dt;

    if( time_update_timer_ > 0.1f )
    {
        time_update_timer_ = 0.0f;
    
        updateTime();
    }
    frame_update_timer_ += wall_dt;

    if(frame_update_timer_ > 1.0f)
    {
       frame_update_timer_ = 0.0f;    
       updateFrames();
    }



}

void Robot::initFrameManager(){



    qDebug(">>>>>initFrameManager begin");
    boost::shared_ptr<tf::TransformListener> tf ;
    tf.reset(new tf::TransformListener(ros::NodeHandle(), ros::Duration(10*60), true));

    qDebug(">>>>>>initFrameManager after create tf::TransformListener:");
     frame_manager_ = new FrameManager(tf);
     pointtf_ = new CoordinateTransform();
     frame_manager_->setFixedFrame("/map");
   //  frame_manager_->setFixedFrame("/base_link");
    // pointtf_->setFixedFrame("/base_link");


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


void Robot::setCameraAndWindow(Ogre::Camera* camera, Ogre::RenderWindow* window){
    pCamera = camera;
    pWindow = window; 
}
void Robot::local2World(Ogre::Vector3 locP, Ogre::Vector3 &worldP, Ogre::SceneNode * node){
/*
    Ogre::Matrix4 worldMat;  
    node->getWorldTransforms(&worldMat);
    worldP = worldMat * locP;  
*/
}

bool Robot::world2Screen(Ogre::Vector3 objPos, Ogre::Vector2& screenPos){

    Ogre::Matrix4 viewMat = pCamera->getViewMatrix();  
    Ogre::Matrix4 projMat = pCamera->getProjectionMatrix();  
  
  
    Ogre::Vector4 inP = Ogre::Vector4(objPos.x, objPos.y, objPos.z ,1.0);  
    Ogre::Vector4 outP = viewMat * inP;  
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
  
    outP.x = outP.x * pWindow->getWidth();  
    outP.y = (1-outP.y) * pWindow->getHeight();  
  
    screenPos.x = outP.x;  
    screenPos.y = outP.y;  
    return true;  

}


/**timer to cycle updating the robot*/
void Robot::update(const LinkUpdater& updater){


    M_NameToLink::iterator link_it = links_.begin();
    M_NameToLink::iterator link_end = links_.end();

    for ( ; link_it != link_end; ++link_it )
    {
        RobotLink* link = link_it->second;
        Ogre::Vector3 visual_position, collision_position ;
        Ogre::Quaternion visual_orientation, collision_orientation;


         if(link != NULL    && updater.getLinkTransforms( link->getName(),
                                   visual_position, visual_orientation,
                                   collision_position, collision_orientation
                                   )  ) 

         {
            if("LShoulderRoll" == link->getName()){ // 0.503312, -0.503264, -0.496714, 0.496666
                qDebug("current visual_orientation is: %f, %f, %f, %f", visual_orientation.w, visual_orientation.x, visual_orientation.y, visual_orientation.z);
                qDebug("current position is: %f, %f, %f", visual_position.x, visual_position.y, visual_position.z);
                visual_orientation.w = 0.707141;
                visual_orientation.x = -0.707073;
                visual_orientation.y = 0.000000;
                visual_orientation.z = 0.000000; 
            }
            link->setTransforms( visual_position, visual_orientation, collision_position, collision_orientation );

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
     mRoot->renderOneFrame(); 
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
void Robot::load( std::string robot_file ,/* const urdf::ModelInterface &urdf, */ bool visual, bool collision ){

   update_timer_->stop();

   TiXmlDocument doc;
   doc.LoadFile(robot_file);

   urdfpath = robot_file; 

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

  pUrdf = urdf; 

  qDebug("URDF parsed OK" );
  clear();
  //using descr ; the descr is 
  typedef std::map<std::string, urdf::LinkSharedPtr > M_NameToUrdfLink;
  qDebug(">>>>>step 1");
  M_NameToUrdfLink::const_iterator link_it = urdf.links_.begin();
  qDebug(">>>>>step 2");
  M_NameToUrdfLink::const_iterator link_end = urdf.links_.end();
  qDebug(">>>>>step 3");
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


    frame_manager_->setFixedFrame("/base_link");
    update_timer_->start( 33.333332 ); 
    
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





