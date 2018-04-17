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

#ifndef ROBOT_H_
#define ROBOT_H_

#include <string>
#include <map>

#include <OgreVector3.h>
#include <OgreQuaternion.h>
#include <OgreAny.h>

#include <urdf/model.h> // can be replaced later by urdf_model/types.h
#include "robot_link.h"

namespace Ogre
{
class SceneManager;
class Entity;
class SceneNode;
class Vector3;
class Quaternion;
class Any;
class RibbonTrail;
class SceneNode;
}


class Robot : public QObject
{
Q_OBJECT
public:
  Robot( Ogre::SceneNode* root_node, Ogre::SceneManager* sceneManger, const std::string& name);
  virtual ~Robot();

  /**
   * \brief Loads meshes/primitives from a robot description.  Calls clear() before loading.
   *
   * @param urdf The robot description to read from
   * @param visual Whether or not to load the visual representation
   * @param collision Whether or not to load the collision representation
   */
  virtual void load(std::string robot_description_, /* const urdf::ModelInterface &urdf, */ bool visual = true, bool collision = true );

  typedef std::map< std::string, RobotLink* > M_NameToLink;

  const M_NameToLink& getLinks() const { return links_; }
  RobotLink* getRootLink() { return root_link_; }
  RobotLink* getLink( const std::string& name );
  const std::string& getName() { return name_; }
  Ogre::SceneNode* getVisualNode() { return root_visual_node_; }
  Ogre::SceneManager* getSceneManager() { return scene_manager_; }

  class LinkFactory
  {
  public:
    virtual RobotLink* createLink( Robot* robot,
                                   Ogre::SceneManager* sceneManger ,
                                   const urdf::LinkConstSharedPtr& link,
                                   const std::string& parent_joint_name,
                                   bool visual,
                                   bool collision);
   // virtual RobotJoint* createJoint( Robot* robot, const urdf::JointConstSharedPtr& joint);
  };


  protected:
      LinkFactory *link_factory_;
      RobotLink *root_link_;
      M_NameToLink links_;                      ///< Map of name to link info, stores all loaded links.
      Ogre::SceneNode* root_visual_node_;           ///< Node all our visual nodes are children of
      Ogre::SceneManager* scene_manager_;

      std::string name_;


};

#endif /*ROBOT_H_ */
