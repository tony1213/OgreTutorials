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

#ifndef ROS_PACKAGE_NAME
# define ROS_PACKAGE_NAME "rviz"
#endif


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

void RobotLink::setTransforms( const Ogre::Vector3& visual_position, const Ogre::Quaternion& visual_orientation,
                               const Ogre::Vector3& collision_position, const Ogre::Quaternion& collision_orientation )
{
  if ( visual_node_ )
  {
    visual_node_->setPosition( visual_position );
    visual_node_->setOrientation( visual_orientation );
  }


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
    static int count = 0;
    if (default_material_name_.empty())
    {
      default_material_ = getMaterialForLink(link);

      qDebug("createEntityForGeometryElement step 2X");
      if(default_material_ == NULL)
         qDebug(">>>>>>>>default_material_ == NULL");

      std::stringstream ss;
      ss << default_material_->getName() << count++ << "Robot";
      std::string cloned_name = ss.str();
      qDebug("createEntityForGeometryElement step 2Y");

      default_material_ = default_material_->clone(cloned_name);
      default_material_name_ = default_material_->getName();
    }
    qDebug("createEntityForGeometryElement step 3");
    for (uint32_t i = 0; i < entity->getNumSubEntities(); ++i)
    {
      default_material_ = getMaterialForLink(link, material_name);
      std::stringstream ss;
      ss << default_material_->getName() << count++ << "Robot";
      std::string cloned_name = ss.str();

      default_material_ = default_material_->clone(cloned_name);
      default_material_name_ = default_material_->getName();

      // Assign materials only if the submesh does not have one already

      Ogre::SubEntity* sub = entity->getSubEntity(i);
      const std::string& material_name = sub->getMaterialName();
      qDebug("createEntityForGeometryElement step 4");

      if (material_name == "BaseWhite" || material_name == "BaseWhiteNoLighting")
      {
        //sub->setMaterialName(default_material_name_);
          sub->setMaterialName("Sinbad/Body");
      }
      else
      {
        // Need to clone here due to how selection works.  Once selection id is done per object and not per material,
        // this can go away
        std::stringstream ss;
        ss << material_name << count++ << "Robot";
        std::string cloned_name = ss.str();
        sub->getMaterial()->clone(cloned_name);
        sub->setMaterialName(cloned_name);
      }
      qDebug("createEntityForGeometryElement step 5");

      materials_[sub] = sub->getMaterial();
  
  }

}

}

Ogre::MaterialPtr RobotLink::getMaterialForLink( const urdf::LinkConstSharedPtr& link, const std::string material_name){
   
    if (!link->visual || !link->visual->material) //error ...chenrui
    {
        qDebug("getMaterialForLink will return default material...");
        return Ogre::MaterialManager::getSingleton().getByName("Sinbad/Body");
    }

    static int count = 0;
    std::stringstream ss;
    ss << "Robot Link Material" << count++;

    Ogre::MaterialPtr mat = Ogre::MaterialManager::getSingleton().create(ss.str(), ROS_PACKAGE_NAME);
    mat->getTechnique(0)->setLightingEnabled(true);

    urdf::VisualSharedPtr visual = link->visual;
  std::vector<urdf::VisualSharedPtr>::const_iterator vi;
  for( vi = link->visual_array.begin(); vi != link->visual_array.end(); vi++ )
  {
    if( (*vi) && material_name != "" && (*vi)->material_name  == material_name) {
      visual = *vi;
      break;
    }
  }
  if ( vi == link->visual_array.end() ) {
    visual = link->visual; // if link does not have material, use default oneee
  }

  if (visual->material->texture_filename.empty())
  {
    const urdf::Color& col = visual->material->color;
    mat->getTechnique(0)->setAmbient(col.r * 0.5, col.g * 0.5, col.b * 0.5);
    mat->getTechnique(0)->setDiffuse(col.r, col.g, col.b, col.a);

    material_alpha_ = col.a;
  }
  else
  {
    std::string filename = visual->material->texture_filename;
    if (!Ogre::TextureManager::getSingleton().resourceExists(filename))
    {
      resource_retriever::Retriever retriever;
      resource_retriever::MemoryResource res;
      try
      {
        res = retriever.get(filename);
      }
      catch (resource_retriever::Exception& e)
      {
      }

      if (res.size != 0)
      {
        Ogre::DataStreamPtr stream(new Ogre::MemoryDataStream(res.data.get(), res.size));
        Ogre::Image image;
        std::string extension = fs::extension(fs::path(filename));

        if (extension[0] == '.')
        {
          extension = extension.substr(1, extension.size() - 1);
        }

        try
        {
          image.load(stream, extension);
          Ogre::TextureManager::getSingleton().loadImage(filename, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, image);
        }
        catch (Ogre::Exception& e)
        {
        }
      }
    }

    Ogre::Pass* pass = mat->getTechnique(0)->getPass(0);
    Ogre::TextureUnitState* tex_unit = pass->createTextureUnitState();;
    tex_unit->setTextureName(filename);
  }

  return mat;


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

      char buf[80];   
      getcwd(buf,sizeof(buf));
      std::string packagePath = buf; 
      qDebug()<<"path is:"<< packagePath.c_str();  
      int index = packagePath.find("build");
      packagePath.erase(index, strlen("build"));
      
      std::string completepath = packagePath + resource_path; 
      qDebug()<<"complete path string is:"<< completepath.c_str();


          

      qDebug()<<"will load stl file"<< resource_path.c_str();
      ogre_tools::STLLoader loader;
 
      if(loader.load(completepath) == false){ //resource_path to completepath
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


