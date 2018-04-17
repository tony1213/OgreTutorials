
#include "robot_joint.h"
#include "robot_link.h"
#include "robot.h"

#include <OgreSceneNode.h>


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


