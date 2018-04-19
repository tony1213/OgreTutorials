
#include "robot_joint.h"
#include "robot_link.h"
#include "robot.h"

#include <OgreSceneNode.h>

#include "vector_property.h"
#include "quaternion_property.h"

RobotJoint::RobotJoint( Robot* robot, const urdf::JointConstSharedPtr& joint )
  : robot_( robot )
  , name_( joint->name )
  , child_link_name_( joint->child_link_name )
  , parent_link_name_( joint->parent_link_name )
{

  std::string type = "";
  if (joint->type == urdf::Joint::UNKNOWN)
    type = "unknown";
  else if (joint->type == urdf::Joint::REVOLUTE)
    type = "revolute";
  else if (joint->type == urdf::Joint::CONTINUOUS)
    type = "continuous";
  else if (joint->type == urdf::Joint::PRISMATIC)
    type = "prismatic";
  else if (joint->type == urdf::Joint::FLOATING)
    type = "floating";
  else if (joint->type == urdf::Joint::PLANAR)
    type = "planar";
  else if (joint->type == urdf::Joint::FIXED)
    type = "fixed";


  const urdf::Vector3& pos = joint->parent_to_joint_origin_transform.position;
  const urdf::Rotation& rot = joint->parent_to_joint_origin_transform.rotation;
  joint_origin_pos_ = Ogre::Vector3(pos.x, pos.y, pos.z);
  joint_origin_rot_ = Ogre::Quaternion(rot.w, rot.x, rot.y, rot.z);


}

RobotJoint::~RobotJoint()
{
}

Ogre::Vector3 RobotJoint::getPosition()
{
  return position_property_->getVector();
}

Ogre::Quaternion RobotJoint::getOrientation()
{
  return orientation_property_->getQuaternion();
}



RobotJoint* RobotJoint::getParentJoint()
{
  RobotLink* parent_link = robot_->getLink(parent_link_name_);
  if (!parent_link)
    return NULL;

  const std::string& parent_joint_name = parent_link->getParentJointName();
  if (parent_joint_name.empty())
    return NULL;

  return robot_->getJoint(parent_joint_name);
}


void RobotJoint::setTransforms( const Ogre::Vector3& parent_link_position,
                                const Ogre::Quaternion& parent_link_orientation )
{
  Ogre::Vector3 position = parent_link_position + parent_link_orientation * joint_origin_pos_;
  Ogre::Quaternion orientation = parent_link_orientation * joint_origin_rot_;

  position_property_->setVector( position );
  orientation_property_->setQuaternion( orientation );
/*
  if ( axes_ )
  {
    axes_->setPosition( position );
    axes_->setOrientation( orientation );
  }
  if ( axis_ )
  {
    axis_->setPosition( position );
    axis_->setOrientation( orientation );
    axis_->setDirection( parent_link_orientation * axis_property_->getVector() );
  }
  */
}



