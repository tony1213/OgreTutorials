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


#ifndef ROBOT_LINK_H
#define ROBOT_LINK_H

#include <string>
#include <map>

#include <QObject>

#ifndef Q_MOC_RUN
#include <OgreVector3.h>
#include <OgreQuaternion.h>
#include <OgreAny.h>
#include <OgreMaterial.h>
#include <OgreSharedPtr.h>
#endif

#include <urdf/model.h> // can be replaced later by urdf_model/types.h
#include <urdf_model/pose.h>
#include <OgreMesh.h>

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



class Property;
class QuaternionProperty;
class Robot;
class VectorProperty;
class RobotJoint;

/**
 * \struct RobotLink
 * \brief Contains any data we need from a link in the robot.
 */
class RobotLink: public QObject
{
Q_OBJECT
public:
  RobotLink(Robot* robot, Ogre::SceneManager* scenemanager, const urdf::LinkConstSharedPtr& link,
             const std::string& parent_joint_name,
             bool visual,
             bool collision);
  virtual ~RobotLink();

  virtual void setRobotAlpha(float a);
  virtual void setTransforms(const Ogre::Vector3& visual_position, const Ogre::Quaternion& visual_orientation,
                     const Ogre::Vector3& collision_position, const Ogre::Quaternion& collision_orientation);

  void setNewTransforms(const Ogre::Vector3& visual_position, const Ogre::Quaternion& visual_orientation);

  Ogre::Vector3 getPosition();
  Ogre::Quaternion getOrientation();

  Ogre::Vector3 getOriginalPosition(){return originPos;}
  Ogre::Quaternion getOriginalOrientation(){return originOrientation;}

  void setPosition(Ogre::Vector3  position);
  void setOrientation(Ogre::Quaternion  quaternion);

  void setOriginalPosition(Ogre::Vector3  position){ originPos = position; }
  void setOriginalOrientation(Ogre::Quaternion  quaternion){originOrientation = quaternion; }

  Ogre::Vector3 getWorldPosition();
  Ogre::Quaternion getWorldOrientation();

  bool hasGeometry() const;
  
  const std::string& getName() const { return name_; }
  const std::string& getParentJointName() const { return parent_joint_name_; }
  const std::vector<std::string>& getChildJointNames() const { return child_joint_names_; }
  Property* getLinkProperty() const { return link_property_; }
  Ogre::SceneNode* getVisualNode() const { return visual_node_; }
  Robot* getRobot() const { return robot_; }

  // Remove link_property_ from its old parent and add to new_parent.  If new_parent==NULL then leav unparented.
  void setParentProperty(Property* new_parent);

  void rotate( Ogre::Quaternion q);
public Q_SLOTS:
  /** @brief Update the visibility of the link elements: visual mesh, collision mesh, trail, and axes.
   *
   * Called by Robot when changing visual and collision visibilities,
   * since each link may be enabled or disabled. */
  void updateVisibility();



private:

  void setRenderQueueGroup( Ogre::uint8 group );
  bool getEnabled() const;
  void createEntityForGeometryElement( const urdf::LinkConstSharedPtr& link, const urdf::Geometry& geom, const urdf::Pose& origin, const std::string material_name, Ogre::SceneNode* scene_node, Ogre::Entity*& entity );

  void createVisual( const urdf::LinkConstSharedPtr& link);
  Ogre::MaterialPtr getMaterialForLink( const urdf::LinkConstSharedPtr& link, const std::string material_name = "" );

  Ogre::MeshPtr loadMeshFromResource(const std::string& resource_path);

  void updateTrail();

protected:
  Robot* robot_;
  Ogre::SceneManager* scene_manager_;

  std::string name_;                          ///< Name of this link

  std::string parent_joint_name_;
  std::vector<std::string> child_joint_names_;

  Property* link_property_;
  Property* details_;
  VectorProperty* position_property_;
  QuaternionProperty* orientation_property_;

private:

  typedef std::map<Ogre::SubEntity*, Ogre::MaterialPtr> M_SubEntityToMaterial;
  M_SubEntityToMaterial materials_;
  Ogre::MaterialPtr default_material_;
  std::string default_material_name_;
  std::vector<Ogre::Entity*> visual_meshes_;    ///< The entities representing the visual mesh of this link (if they exist)

  Ogre::SceneNode* visual_node_;              ///< The scene node the visual meshes are attached to
  Ogre::RibbonTrail* trail_;
  Ogre::MaterialPtr color_material_;
  bool using_color_;

  Ogre::Vector3  originPos; 
  Ogre::Quaternion  originOrientation;
  urdf::Pose rOrigin;  //recorded original pose got from urdf...  

  float material_alpha_; ///< If material is not a texture, this saves the alpha value set in the URDF, otherwise is 1.0.
  float robot_alpha_; ///< Alpha value from top-level robot alpha Property (set via setRobotAlpha())

};


#endif // ROBOT_LINK_H
