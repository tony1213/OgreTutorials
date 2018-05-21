
#include "property.h"

#include "coordinate_transform.h"

CoordinateTransform::CoordinateTransform()
{
}

CoordinateTransform::~CoordinateTransform()
{
}


void CoordinateTransform::setFixedFrame(const std::string& frame)
{
  bool send = false;
  boost::mutex::scoped_lock lock(cache_mutex_);
  if( fixed_frame_ != frame )
  {
      fixed_frame_ = frame;
      cache_.clear();
      send = true;
  }
  if(send == true)
  {
    // This emission must be kept outside of the mutex lock to avoid deadlocks.
    Q_EMIT fixedFrameChanged();
  }
}


bool CoordinateTransform::getTransform(const std::string& frame, QDateTime time, Ogre::Vector3& position, Ogre::Quaternion& orientation){




    if ( !adjustTime(frame, time) )
    {
        qDebug(">>>>CoordinateTransform::getTransform 1");
        return false;
    }
    boost::mutex::scoped_lock lock(cache_mutex_);

    position = Ogre::Vector3(9999999, 9999999, 9999999);
    orientation = Ogre::Quaternion::IDENTITY;

    if (fixed_frame_.empty())
    {
        qDebug(">>>>CoordinateTransform::getTransform 2");
        return false;
    }

    M_Cache::iterator it = cache_.find(CacheKey(frame, time));
    if (it != cache_.end())
    {
        position = it->second.position;
        orientation = it->second.orientation;
        return true;
    }
    Ogre::Vector3 positionX; 
    Ogre::Quaternion orientationY; 
    orientationY.w = 1.0f;


    if (!transform(frame, time, positionX, orientationY, position, orientation))
    {
        qDebug(">>>>CoordinateTransform::getTransform 3");
        return false;
    }

    cache_.insert(std::make_pair(CacheKey(frame, time), CacheEntry(position, orientation)));


    return true;
}



bool CoordinateTransform::transform(const std::string& frame, QDateTime time, Ogre::Vector3& positionT,Ogre::Quaternion& orientationT,  Ogre::Vector3& position, Ogre::Quaternion& orientation){


    position = Ogre::Vector3::ZERO;
    orientation = Ogre::Quaternion::IDENTITY;

    Ogre::Vector3  inputPos; 
    Ogre::Quaternion  inputOrientation; 
  
    Ogre::Vector3  outputPos;
    Ogre::Quaternion  outputOrientation;

    inputPos.x = positionT.x;
    inputPos.y = positionT.y;
    inputPos.z = positionT.z;
    inputOrientation.x = orientationT.x;
    inputOrientation.y = orientationT.y;
    inputOrientation.z = orientationT.z;
    if(inputOrientation.x == 0 && inputOrientation.y == 0 && inputOrientation.z == 0 && inputOrientation.w == 0){
        inputOrientation.w = 1.0;
    }
     



 




    position.x = outputPos.x;
    position.y = outputPos.y;
    position.z = outputPos.z;
    orientation.x = outputOrientation.x;
    orientation.y = outputOrientation.y;
    orientation.z = outputOrientation.z;
    orientation.w = outputOrientation.w;




    return true;
}


bool CoordinateTransform::adjustTime( const std::string &frame, QDateTime &time ){





    return true;
}



