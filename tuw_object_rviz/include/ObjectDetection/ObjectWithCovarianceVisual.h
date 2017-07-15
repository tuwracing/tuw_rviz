/***************************************************************************
 *   Software License Agreement (BSD License)                              *
 *   Copyright (C) 2017 by Florian Beck <florian.beck@tuwien.ac.at>        *
 *                                                                         *
 *   Redistribution and use in source and binary forms, with or without    *
 *   modification, are permitted provided that the following conditions    *
 *   are met:                                                              *
 *                                                                         *
 *   1. Redistributions of source code must retain the above copyright     *
 *      notice, this list of conditions and the following disclaimer.      *
 *   2. Redistributions in binary form must reproduce the above copyright  *
 *      notice, this list of conditions and the following disclaimer in    *
 *      the documentation and/or other materials provided with the         *
 *      distribution.                                                      *
 *   3. Neither the name of the copyright holder nor the names of its      *
 *      contributors may be used to endorse or promote products derived    *
 *      from this software without specific prior written permission.      *
 *                                                                         *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS   *
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT     *
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS     *
 *   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE        *
 *   COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,  *
 *   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,  *
 *   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;      *
 *   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER      *
 *   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT    *
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY *
 *   WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           *
 *   POSSIBILITY OF SUCH DAMAGE.                                           *
 ***************************************************************************/

#ifndef OBJECT_WITH_COVARIANCE_VISUAL_H
#define OBJECT_WITH_COVARIANCE_VISUAL_H

#include <ObjectDetection/CovarianceVisual.h>
#include <ObjectDetection/TextVisual.h>
#include <ObjectDetection/common.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/shape.h>
#include <tuw_object_msgs/ObjectWithCovariance.h>

namespace Ogre
{
class Vector3;
class Quaternion;
class ColorValue;
}

namespace rviz
{
class Arrow;
class Shape;
class Shape;
class MovableText;
}

namespace tuw_object_rviz
{
// Declare the visual class for this display.
class ObjectWithCovarianceVisual
{
public:
  // Constructor.  Creates the visual stuff and puts it into the
  // scene, but in an unconfigured state.
  ObjectWithCovarianceVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node);

  // Destructor.  Removes the visual stuff from the scene.
  virtual ~ObjectWithCovarianceVisual();

  // Configure the visual to show the data in the message.
  virtual void setMessage(const tuw_object_msgs::ObjectWithCovariance::ConstPtr& msg);

  // Set the pose of the coordinate frame the message refers to.
  // These could be done inside setMessage(), but that would require
  // calls to FrameManager and error handling inside setMessage(),
  // which doesn't seem as clean.  This way ObjectWithCovarianceVisual is
  // only responsible for visualization.
  void setFramePosition(const Ogre::Vector3& position);
  void setFrameOrientation(const Ogre::Quaternion& orientation);

  // void setTransformPosition ( const Ogre::Vector3& position );
  // void setTransformOrientation ( const Ogre::Quaternion& orientation );

  void setTransform(const Ogre::Vector3& position, const Ogre::Quaternion& orientation);

  // Set the scale of the visual, which is an user-editable
  // parameter and therefore don't come from the message.
  virtual void setScale(float scale);

  // Set the color of the visual's Pose, which is an user-editable
  // parameter and therefore don't come from the message.
  virtual void setColor(Ogre::ColourValue color);

  void setVisiblities(bool render_covariance, bool render_id, bool render_sensor_type);

  virtual void setStyle(Styles style);

private:
  // The object implementing the actual pose shape
  boost::shared_ptr< rviz::Arrow > pose_;

  // the object implementing the center of the detection
  boost::shared_ptr< rviz::Shape > mean_;

  boost::shared_ptr< CovarianceVisual > covariance_;

  // the object that can be used to show category color even though color is part of the detection
  boost::shared_ptr< rviz::Shape > category_;

  boost::shared_ptr< TextVisual > detection_id_;

protected:
  // A SceneNode whose pose is set to match the coordinate frame of
  // the Imu message header.
  Ogre::SceneNode* frame_node_;

  // The SceneManager, kept here only so the destructor can ask it to
  // destroy the ``frame_node_``.
  Ogre::SceneManager* scene_manager_;

  // The pose Shape object's scale
  float scale_;

  // The pose Shape object's color
  Ogre::ColourValue color_;

  // transform
  Ogre::Matrix4 transform_;
};

}  // end namespace tuw_object_rviz

#endif  // OBJECT_DETECTION_VISUAL_H
