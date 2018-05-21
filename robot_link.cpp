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

#include "property.h"
#include "quaternion_property.h"
#include "vector_property.h"


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

   link_property_ = new Property( link->name.c_str(), true, "", NULL, SLOT( updateVisibility() ), this );
   details_ = new Property( "Details", QVariant(), "", NULL);

    position_property_ = new VectorProperty( "Position", Ogre::Vector3::ZERO,
                                           "Position of this link, in the current Fixed Frame.  (Not editable)",
                                           link_property_ );
 // position_property_->setReadOnly( true );

  orientation_property_ = new QuaternionProperty( "Orientation", Ogre::Quaternion::IDENTITY,
                                                  "Orientation of this link, in the current Fixed Frame.  (Not editable)",
                                                  link_property_ );
 // orientation_property_->setReadOnly( true );

  link_property_->collapse();






    scene_manager_ = scenemanager; 
    visual_node_ = robot_->getVisualNode()->createChildSceneNode();
    createVisual( link );


    if (link->child_joints.empty())
  {
  //  qDebug() << " has no children.";
  }
  else
  {

    if (link->child_joints.size() > 1)
    {
    }
    else
    {
    }

    std::vector<urdf::JointSharedPtr >::const_iterator child_it = link->child_joints.begin();
    std::vector<urdf::JointSharedPtr >::const_iterator child_end = link->child_joints.end();
    for ( ; child_it != child_end ; ++child_it )
    {
      urdf::Joint *child_joint = child_it->get();
      if (child_joint && !child_joint->name.empty())
      {
        child_joint_names_.push_back(child_joint->name);
      }
    }
  }



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
    delete link_property_;
 
}

bool RobotLink::hasGeometry() const
{
  // return true; 
 return visual_meshes_.size() /*  + collision_meshes_.size() */ > 0;
}

bool RobotLink::getEnabled() const
{
   //return true; 

  if (!hasGeometry())
    return true;
  return link_property_->getValue().toBool();

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

void RobotLink::updateVisibility()
{
  bool enabled = getEnabled();

 // robot_->calculateJointCheckboxes();

  if( visual_node_ )
  {
   // visual_node_->setVisible( enabled && robot_->isVisible() && robot_->isVisualVisible() );
      visual_node_->setVisible(true);
  }
  /*
  if( collision_node_ )
  {
    collision_node_->setVisible( enabled && robot_->isVisible() && robot_->isCollisionVisible() );
  }
  */
  if( trail_ )
  {
   // trail_->setVisible( enabled && robot_->isVisible() );
      trail_->setVisible(true);
  }
  /*
  if( axes_ )
  {
    axes_->getSceneNode()->setVisible( enabled && robot_->isVisible() );
  }
  */
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

  position_property_->setVector( visual_position );
  orientation_property_->setQuaternion( visual_orientation );


}

void RobotLink::setParentProperty(Property* new_parent)
{
  Property* old_parent = link_property_->getParent();
  if (old_parent)
    old_parent->takeChild(link_property_);

  if (new_parent)
    new_parent->addChild(link_property_);
}


Ogre::Vector3 RobotLink::getPosition()
{
  return position_property_->getVector();
}

Ogre::Quaternion RobotLink::getOrientation()
{
  return orientation_property_->getQuaternion();
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
      scale = Ogre::Vector3(mesh.scale.x*90, mesh.scale.y*90, mesh.scale.z*90);

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

  if ( entity )
  {
    offset_node->attachObject(entity);
    offset_node->setScale(scale);
    offset_node->setPosition(offset_position);
    //set position and orientation  to link, chenrui

    position_property_->setVector(offset_position );
    orientation_property_->setQuaternion(offset_orientation );


    offset_node->setOrientation(offset_orientation);
    //perhaps set material
    static int count = 0;
    if (default_material_name_.empty())
    {
      default_material_ = getMaterialForLink(link);


      std::stringstream ss;
      ss << default_material_->getName() << count++ << "Robot";
      std::string cloned_name = ss.str();

      default_material_ = default_material_->clone(cloned_name);
      default_material_name_ = default_material_->getName();
    }
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

      materials_[sub] = sub->getMaterial();
  
  }

}

}

Ogre::MaterialPtr RobotLink::getMaterialForLink( const urdf::LinkConstSharedPtr& link, const std::string material_name){
   
    if (!link->visual || !link->visual->material) //error ...chenrui
    {
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
    //  qDebug()<<"path is:"<< packagePath.c_str();  
      int index = packagePath.find("debug");
      packagePath.erase(index, strlen("debug"));
      
      std::string completepath = packagePath +"OgreTutorials/"+ resource_path;
    //  qDebug()<<"complete path string is:"<< completepath.c_str();


          

   //   qDebug()<<"will load stl file"<< resource_path.c_str();
      ogre_tools::STLLoader loader;
 
      if(loader.load(completepath) == false){ //resource_path to completepath
        //  qDebug("load stl file failed");
          return Ogre::MeshPtr();
      }
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
      createEntityForGeometryElement( link, *visual->geometry, visual->origin, visual->material_name, visual_node_, visual_mesh );
      if( visual_mesh )
      {
        visual_meshes_.push_back( visual_mesh );
        valid_visual_found = true;
      }
    }
  }

/*
  if( !valid_visual_found && link->visual && link->visual->geometry )
  {
    qDebug(">>>>>RobotLink::createVisual>>>>>the second condistion");
    Ogre::Entity* visual_mesh = NULL;
    createEntityForGeometryElement( link, *link->visual->geometry, link->visual->origin, link->visual->material_name, visual_node_, visual_mesh );
    if( visual_mesh )
    {
      visual_meshes_.push_back( visual_mesh );
    }
  }
*/
  visual_node_->setVisible(true);



}


