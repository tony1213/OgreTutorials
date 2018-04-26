
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
  , has_decendent_links_with_geometry_( true )
{

  joint_property_ = new Property(
                              name_.c_str(),
                              true,
                              "",
                              NULL,
                              SLOT( updateChildVisibility() ),
                              this);

  details_ = new Property( "Details", QVariant(), "", NULL);
  position_property_ = new VectorProperty(
                              "Position",
                              Ogre::Vector3::ZERO,
                              "Position of this joint, in the current Fixed Frame.  (Not editable)",
                              joint_property_ );
  position_property_->setReadOnly( true );

  orientation_property_ = new QuaternionProperty(
                              "Orientation",
                              Ogre::Quaternion::IDENTITY,
                              "Orientation of this joint, in the current Fixed Frame.  (Not editable)",
                              joint_property_ );
  orientation_property_->setReadOnly( true );


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
    delete joint_property_;
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

  qDebug(">>>>>>>>RobotJoint::setTransforms");
  Ogre::Vector3 position = parent_link_position + parent_link_orientation * joint_origin_pos_;
  Ogre::Quaternion orientation = parent_link_orientation * joint_origin_rot_;

  position_property_->setVector( position );
  orientation_property_->setQuaternion( orientation );
  qDebug(">>>>>>>>RobotJoint::setTransforms  OK>>>>");
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

bool RobotJoint::getEnabled() const
{
  if (!hasDescendentLinksWithGeometry())
    return true;
  return joint_property_->getValue().toBool();
}

bool RobotJoint::styleIsTree() const
{
  return details_->getParent() != NULL;
}

void RobotJoint::useDetailProperty(bool use_detail)
{
  Property* old_parent = details_->getParent();
  if (old_parent)
    old_parent->takeChild(details_);

  if (use_detail)
  {
    while (joint_property_->numChildren() > 0)
    {
      Property* child = joint_property_->childAt(0);
      joint_property_->takeChild(child);
      details_->addChild(child);
    }

    joint_property_->addChild(details_);
  }
  else
  {
    while (details_->numChildren() > 0)
    {
      Property* child = details_->childAt(0);
      details_->takeChild(child);
      joint_property_->addChild(child);
    }
  }
}



void RobotJoint::updateChildVisibility()
{

    if (!hasDescendentLinksWithGeometry())
    return;

  bool visible = getEnabled();

  RobotLink *link = robot_->getLink(child_link_name_);

  if (link)
  {
    if (link->hasGeometry())
    {
      link->getLinkProperty()->setValue(visible);
    }

    if (styleIsTree())
    {
      std::vector<std::string>::const_iterator child_joint_it = link->getChildJointNames().begin();
      std::vector<std::string>::const_iterator child_joint_end = link->getChildJointNames().end();
      for ( ; child_joint_it != child_joint_end ; ++child_joint_it )
      {
        RobotJoint* child_joint = robot_->getJoint( *child_joint_it );
        if (child_joint)
        {
          child_joint->getJointProperty()->setValue(visible);
        }
      }
    }
  }








}

