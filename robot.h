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

#include "link_updater.h"

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

class Robot;
class RobotLink;
class RobotJoint;
class Property;

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

   /**
   * \brief Clears all data loaded from a URDF
   */
  virtual void clear();
  virtual void update(const LinkUpdater& updater);

    /**
   * \brief Set the robot as a whole to be visible or not
   * @param visible Should we be visible?
   */
  virtual void setVisible( bool visible );

  void updateRobot(); //add by chenrui

  /**
   * \brief Set whether the visual meshes of the robot should be visible
   * @param visible Whether the visual meshes of the robot should be visible
   */
  void setVisualVisible( bool visible );
    /**
   * \brief Returns whether anything is visible
   */
  bool isVisible();
    /**
   * \brief Returns whether or not the visual representation is set to be visible
   * To be visible this and isVisible() must both be true.
   */
  bool isVisualVisible();

  enum LinkTreeStyle {
    STYLE_LINK_LIST,         // list of all links sorted by link name
    STYLE_DEFAULT = STYLE_LINK_LIST,
    STYLE_JOINT_LIST,        // list of joints sorted by joint name
    STYLE_LINK_TREE,         // tree of links
    STYLE_JOINT_LINK_TREE    // tree of joints with links
  };

  
  void setAlpha(float a);
  float getAlpha() { return alpha_; }
  RobotLink* getRootLink() { return root_link_; }
  RobotLink* getLink( const std::string& name );
  RobotJoint* getJoint( const std::string& name );
  

  typedef std::map< std::string, RobotLink* > M_NameToLink;
  typedef std::map< std::string, RobotJoint* > M_NameToJoint;
  const M_NameToLink& getLinks() const { return links_; }
  const M_NameToJoint& getJoints() const { return joints_; }

  const std::string& getName() { return name_; }
  Ogre::SceneNode* getVisualNode() { return root_visual_node_; }
  Ogre::SceneManager* getSceneManager() { return scene_manager_; }
  Ogre::SceneNode* getOtherNode() { return root_other_node_; }


  virtual void setPosition( const Ogre::Vector3& position );
  virtual void setOrientation( const Ogre::Quaternion& orientation );
  virtual void setScale( const Ogre::Vector3& scale );
  virtual const Ogre::Vector3& getPosition();
  virtual const Ogre::Quaternion& getOrientation();

  class LinkFactory
  {
  public:
    virtual RobotLink* createLink( Robot* robot,
                                   Ogre::SceneManager* sceneManger ,
                                   const urdf::LinkConstSharedPtr& link,
                                   const std::string& parent_joint_name,
                                   bool visual,
                                   bool collision);
    virtual RobotJoint* createJoint( Robot* robot, const urdf::JointConstSharedPtr& joint);
  };

protected:

     /** @brief Call RobotLink::updateVisibility() on each link. */
  void updateLinkVisibilities();
     /** used by setLinkTreeStyle() to recursively build link & joint tree. */
  void addLinkToLinkTree(LinkTreeStyle style, Property *parent,  RobotLink *link);
  void addJointToLinkTree(LinkTreeStyle style, Property *parent, RobotJoint *joint);
 
  void initLinkTreeStyle(); 
  static bool styleShowLink(LinkTreeStyle style);
  static bool styleShowJoint(LinkTreeStyle style);
  static bool styleIsTree(LinkTreeStyle style);    

      LinkFactory *link_factory_;
      M_NameToLink links_;                      ///< Map of name to link info, stores all loaded links.
      M_NameToJoint joints_;                    ///< Map of name to joint info, stores all loaded joints
      RobotLink *root_link_;
      Ogre::SceneNode* root_visual_node_;           ///< Node all our visual nodes are children of
      Ogre::SceneNode* root_other_node_;
      Ogre::SceneManager* scene_manager_;

      std::string name_;

      float alpha_;

      bool visible_;                                ///< Should we show anything at all? (affects visual, collision, axes, and trails)
      bool visual_visible_;                         ///< Should we show the visual representation?


};

#endif /*ROBOT_H_ */
