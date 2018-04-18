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


class Robot;

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

  Ogre::Vector3 getPosition();
  Ogre::Quaternion getOrientation();
  
  const std::string& getName() const { return name_; }
  const std::string& getParentJointName() const { return parent_joint_name_; }
  const std::vector<std::string>& getChildJointNames() const { return child_joint_names_; }
  Ogre::SceneNode* getVisualNode() const { return visual_node_; }


private:

  void createEntityForGeometryElement( const urdf::LinkConstSharedPtr& link, const urdf::Geometry& geom, const urdf::Pose& origin, const std::string material_name, Ogre::SceneNode* scene_node, Ogre::Entity*& entity );

  void createVisual( const urdf::LinkConstSharedPtr& link);

  Ogre::MeshPtr loadMeshFromResource(const std::string& resource_path);

  void updateTrail();

protected:
  Robot* robot_;
  Ogre::SceneManager* scene_manager_;

  std::string name_;                          ///< Name of this link

  std::string parent_joint_name_;
  std::vector<std::string> child_joint_names_;
private:

  std::vector<Ogre::Entity*> visual_meshes_;    ///< The entities representing the visual mesh of this link (if they exist)

  Ogre::SceneNode* visual_node_;              ///< The scene node the visual meshes are attached to
  Ogre::RibbonTrail* trail_;

  float robot_alpha_; ///< Alpha value from top-level robot alpha Property (set via setRobotAlpha())

};


#endif // ROBOT_LINK_H
