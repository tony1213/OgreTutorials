
#ifndef RVIZ_FRAME_MANAGER_H
#define RVIZ_FRAME_MANAGER_H

#include <map>

#include <QObject>

#include <OgreVector3.h>
#include <OgreQuaternion.h>

#include <boost/thread/mutex.hpp>


#include <QDateTime>

class CoordinateTransform: public QObject
{
Q_OBJECT
public:

   CoordinateTransform();

  /** @brief Destructor.
   *
   * FrameManager should not need to be destroyed by hand, just
   * destroy the boost::shared_ptr returned by instance(), and it will
   * be deleted when the last reference is removed. */
  ~CoordinateTransform();



  /** @brief Return the pose for a frame relative to the fixed frame, in Ogre classes, at a given time.
   * @param[in] frame The frame to find the pose of.
   * @param[in] time The time at which to get the pose.
   * @param[out] position The position of the frame relative to the fixed frame.
   * @param[out] orientation The orientation of the frame relative to the fixed frame.
   * @return true on success, false on failure. */
  bool getTransform(const std::string& frame, QDateTime time, Ogre::Vector3& position, Ogre::Quaternion& orientation);

   /** @brief Transform a pose from a frame into the fixed frame.
   * @param[in] frame The input frame.
   * @param[in] time The time at which to get the pose.
   * @param[in] pose The input pose, relative to the input frame.
   * @param[in] Quaternion, the input Quaternion, releative to the input frame
   * @param[out] position Position part of pose relative to the fixed frame.
   * @param[out] orientation: Orientation part of pose relative to the fixed frame.
   * @return true on success, false on failure. */
  bool transform(const std::string& frame, QDateTime time, Ogre::Vector3& positionT,Ogre::Quaternion& orientationT,  Ogre::Vector3& position, Ogre::Quaternion& orientation);


Q_SIGNALS:
  /** @brief Emitted whenever the fixed frame changes. */
  void fixedFrameChanged();


private:

  bool adjustTime( const std::string &frame, QDateTime &time );



  struct CacheKey
  {
    CacheKey(const std::string& f, QDateTime t)
    : frame(f)
    , time(t)
    {}

    bool operator<(const CacheKey& rhs) const
    {
      if (frame != rhs.frame)
      {
        return frame < rhs.frame;
      }

      return time < rhs.time;
    }

    std::string frame;
    QDateTime time;
  };


struct CacheEntry
  {
    CacheEntry(const Ogre::Vector3& p, const Ogre::Quaternion& o)
    : position(p)
    , orientation(o)
    {}

    Ogre::Vector3 position;
    Ogre::Quaternion orientation;
  };
  typedef std::map<CacheKey, CacheEntry > M_Cache;


   boost::mutex cache_mutex_;
   M_Cache cache_;


};

#endif // RVIZ_FRAME_MANAGER_H
                                
