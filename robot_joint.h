#ifndef RVIZ_ROBOT_JOINT_H
#define RVIZ_ROBOT_JOINT_H

#include <string>
#include <map>

#include <QObject>

#ifndef Q_MOC_RUN
#include <OgreVector3.h>
#include <OgreQuaternion.h>
#include <OgreAny.h>
#include <OgreMaterial.h>
#endif

#include <urdf/model.h>
#include <urdf_model/pose.h>

namespace Ogre
{
class SceneManager;
class Entity;
class SubEntity;
class SceneNode;
class Vector3;
class Quaternion;
class Any;
class RibbonTrail;
}

class Robot;
class RobotJoint;
class RobotLink;
class Property;
class QuaternionProperty;
class VectorProperty;
//class Axes;

/**
 * \struct RobotJoint
 * \brief Contains any data we need from a joint in the robot.
 */
class RobotJoint: public QObject
{
Q_OBJECT
public:

  RobotJoint( Robot* robot, const urdf::JointConstSharedPtr& joint );
  virtual ~RobotJoint();


  void setTransforms(const Ogre::Vector3& parent_link_position,
                     const Ogre::Quaternion& parent_link_orientation);



  const std::string& getName() const { return name_; }
  const std::string& getParentLinkName() const { return parent_link_name_; }
  const std::string& getChildLinkName() const { return child_link_name_; }
  const Property* getJointProperty() const { return joint_property_; }
  Property* getJointProperty() { return joint_property_; }
  RobotJoint* getParentJoint();
  RobotLink*  getParentLink();
  Ogre::Vector3 getPosition();
  Ogre::Quaternion getOrientation();

  void setPosition(Ogre::Vector3 pos);
  void setOrientation(Ogre::Quaternion orientation);


  void setRobotAlpha(float a) {}

    // Remove joint_property_ from its old parent and add to new_parent.  If new_parent==NULL then leav unparented.
  void setParentProperty(Property* new_parent);

  bool hasDescendentLinksWithGeometry() const { return has_decendent_links_with_geometry_; }
   // place subproperties as children of details_ or joint_property_
  void useDetailProperty(bool use_detail);

private Q_SLOTS:
 // void updateAxes();
 // void updateAxis();
  void updateChildVisibility();

private:

    bool getEnabled() const;

  // true if displaying in a tree style.  False if list style.
    bool styleIsTree() const;


protected:
  Robot* robot_;
  std::string name_;                          ///< Name of this joint
  std::string parent_link_name_;
  std::string child_link_name_;

  // properties
  Property* joint_property_;
  Property* details_;
  VectorProperty* position_property_;
  QuaternionProperty* orientation_property_;
  Property* axes_property_;

private:
  Ogre::Vector3 joint_origin_pos_;
  Ogre::Quaternion joint_origin_rot_;
  bool has_decendent_links_with_geometry_;

  bool doing_set_checkbox_;   // prevents updateChildVisibility() from  touching children

//  Axes* axes_;

};


#endif // RVIZ_ROBOT_LINK_H

