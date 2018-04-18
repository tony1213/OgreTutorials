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

#include <boost/filesystem.hpp>

#include <OgreEntity.h>
#include <OgreMaterial.h>
#include <OgreMaterialManager.h>
#include <OgreRibbonTrail.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreSubEntity.h>
#include <OgreTextureManager.h>
#include <OgreSharedPtr.h>
#include <OgreTechnique.h>

#include <OgreMeshManager.h>
#include <OgreTexture.h>
#include <OgrePass.h>
#include <OgreTextureUnitState.h>
#include <OgreMeshSerializer.h>
#include <OgreSubMesh.h>
#include <OgreHardwareBufferManager.h>

#include <urdf_model/model.h>
#include <urdf_model/link.h>


#include "robot.h"
#include "robot_link.h"
#include "retriever.h"

#include "stl_loader.h"
#include <QDebug>

namespace fs=boost::filesystem;



RobotLink::RobotLink(Robot* robot, Ogre::SceneManager* scenemanager, const urdf::LinkConstSharedPtr& link,
                      const std::string& parent_joint_name,
                      bool visual,
                      bool collision)
:robot_(robot),
 robot_alpha_(1.0),
 name_( link->name ),
 parent_joint_name_( parent_joint_name ),
 visual_node_( NULL ),
 trail_( NULL )
{

    scene_manager_ = scenemanager; 
    visual_node_ = robot_->getVisualNode()->createChildSceneNode();
    createVisual( link );

}

RobotLink::~RobotLink()
{

    for( size_t i = 0; i < visual_meshes_.size(); i++ )
    {
        scene_manager_->destroyEntity( visual_meshes_[ i ]);
    }
    if ( trail_ )
    {
        scene_manager_->destroyRibbonTrail( trail_ );
    }
 
}

void RobotLink::updateTrail()
{
    if( !trail_ )
    {
      if( visual_node_ )
      {
        static int count = 0;
        std::stringstream ss;
        ss << "Trail for link " << name_ << count++;
        trail_ = scene_manager_->createRibbonTrail( ss.str() );
        trail_->setMaxChainElements( 100 );
        trail_->setInitialWidth( 0, 0.01f );
        trail_->setInitialColour( 0, 0.0f, 0.5f, 0.5f );
        trail_->addNode( visual_node_ );
        trail_->setTrailLength( 2.0f );
        trail_->setVisible(true);
        robot_->getOtherNode()->attachObject( trail_ );
      }
      else
      {
      }
    }
}



void RobotLink::setRobotAlpha( float a )
{
  robot_alpha_ = a;
 // updateAlpha();
}


void RobotLink::createEntityForGeometryElement(const urdf::LinkConstSharedPtr& link, const urdf::Geometry& geom, const urdf::Pose& origin, const std::string material_name, Ogre::SceneNode* scene_node, Ogre::Entity*& entity)
{

   Ogre::SceneNode* offset_node = scene_node->createChildSceneNode();

  static int count = 0;
  std::stringstream ss;
  ss << "Robot Link" << count++;
  std::string entity_name = ss.str();

  Ogre::Vector3 scale(Ogre::Vector3::UNIT_SCALE);

  Ogre::Vector3 offset_position(Ogre::Vector3::ZERO);
  Ogre::Quaternion offset_orientation(Ogre::Quaternion::IDENTITY);


  {
    Ogre::Vector3 position( origin.position.x, origin.position.y, origin.position.z );
    Ogre::Quaternion orientation( Ogre::Quaternion::IDENTITY );
    orientation = orientation * Ogre::Quaternion( origin.rotation.w, origin.rotation.x, origin.rotation.y, origin.rotation.z  );

    offset_position = position;
    offset_orientation = orientation;
  }
  if(geom.type == urdf::Geometry::MESH){
      const urdf::Mesh& mesh = static_cast<const urdf::Mesh&>(geom);

      if ( mesh.filename.empty() )
          return;
      scale = Ogre::Vector3(mesh.scale.x, mesh.scale.y, mesh.scale.z);

      std::string model_name = mesh.filename;
      //will load mesh file and create entity
      try
     {
       loadMeshFromResource(model_name);
       entity = scene_manager_->createEntity( ss.str(), model_name );
    }
    catch( Ogre::InvalidParametersException& e )
    {
    }
    catch( Ogre::Exception& e )
    {
    }

  }
  qDebug("createEntityForGeometryElement step 1");

  if ( entity )
  {
    qDebug("createEntityForGeometryElement step 2");
    offset_node->attachObject(entity);
    offset_node->setScale(scale);
    offset_node->setPosition(offset_position);
    offset_node->setOrientation(offset_orientation);
    //perhaps set material 

  }



}

/** ogre load mesh file */
Ogre::MeshPtr RobotLink::loadMeshFromResource(const std::string& resource_path){

  if (Ogre::MeshManager::getSingleton().resourceExists(resource_path))
  {
      return Ogre::MeshManager::getSingleton().getByName(resource_path);
  }
  else
  {
    fs::path model_path(resource_path);
#if BOOST_FILESYSTEM_VERSION == 3
    std::string ext = model_path.extension().string();
#else
    std::string ext = model_path.extension();
#endif
    if (ext == ".stl" || ext == ".STL" || ext == ".stlb" || ext == ".STLB")
    {
      qDebug("loadMeshFromResource step 1");
/*
      resource_retriever::Retriever retriever;
      resource_retriever::MemoryResource res;
      try
      {
        res = retriever.get(resource_path); //there is error here...chenrui
      }
      catch (resource_retriever::Exception& e)
      {
        qDebug("loadMeshFromResource exception 1");
        return Ogre::MeshPtr();
      }

      if (res.size == 0)
      {
        qDebug("loadMeshFromResource exception 2");
        return Ogre::MeshPtr();
      }

      ogre_tools::STLLoader loader;
      loader.load(resource_path);
  
      if (!loader.load(res.data.get(), res.size, resource_path))
      {
        qDebug("loadMeshFromResource exception 3");
        return Ogre::MeshPtr();
      }
*/
      qDebug()<<"will load stl file"<< resource_path.c_str();
      ogre_tools::STLLoader loader;
 
      if(loader.load(resource_path) == false){
          qDebug("load stl file failed");
          return Ogre::MeshPtr();
      }
      qDebug("load stl file ok");
      return loader.toMesh(resource_path);
    }
    
 
   }

   return Ogre::MeshPtr();

}

void RobotLink::createVisual(const urdf::LinkConstSharedPtr& link ){

    bool valid_visual_found = false;


    std::vector<urdf::VisualSharedPtr >::const_iterator vi;
  for( vi = link->visual_array.begin(); vi != link->visual_array.end(); vi++ )
  {
    urdf::VisualSharedPtr visual = *vi;
    if( visual && visual->geometry )
    {
      Ogre::Entity* visual_mesh = NULL;
      qDebug("createVisual step 1");
      createEntityForGeometryElement( link, *visual->geometry, visual->origin, visual->material_name, visual_node_, visual_mesh );
      if( visual_mesh )
      {
        visual_meshes_.push_back( visual_mesh );
        valid_visual_found = true;
      }
    }
  }


  if( !valid_visual_found && link->visual && link->visual->geometry )
  {
    Ogre::Entity* visual_mesh = NULL;
    qDebug("createVisual step 2");
    createEntityForGeometryElement( link, *link->visual->geometry, link->visual->origin, link->visual->material_name, visual_node_, visual_mesh );
    if( visual_mesh )
    {
      visual_meshes_.push_back( visual_mesh );
    }
  }

  visual_node_->setVisible(true);



}


