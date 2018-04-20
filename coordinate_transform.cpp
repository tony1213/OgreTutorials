
#include "property.h"

#include "coordinate_transform.h"

CoordinateTransform::CoordinateTransform()
{
}

CoordinateTransform::~CoordinateTransform()
{
}



bool CoordinateTransform::getTransform(const std::string& frame, QDateTime time, Ogre::Vector3& position, Ogre::Quaternion& orientation){

    if ( !adjustTime(frame, time) )
    {
        return false;
    }
    boost::mutex::scoped_lock lock(cache_mutex_);

    position = Ogre::Vector3(9999999, 9999999, 9999999);
    orientation = Ogre::Quaternion::IDENTITY;

    if (fixed_frame_.empty())
    {
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
        return false;
    }

    cache_.insert(std::make_pair(CacheKey(frame, time), CacheEntry(position, orientation)));


    return true;
}



bool CoordinateTransform::transform(const std::string& frame, QDateTime time, Ogre::Vector3& positionT,Ogre::Quaternion& orientationT,  Ogre::Vector3& position, Ogre::Quaternion& orientation){













    return true;
}


bool CoordinateTransform::adjustTime( const std::string &frame, QDateTime &time ){





    return true;
}



