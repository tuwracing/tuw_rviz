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

#ifndef OBJECT_WITH_COVARIANCE_ARRAY_DISPLAY_H
#define OBJECT_WITH_COVARIANCE_ARRAY_DISPLAY_H

#ifndef Q_MOC_RUN
#include <rviz/message_filter_display.h>
#include <tuw_object_msgs/ObjectWithCovarianceArray.h>
#include <boost/circular_buffer.hpp>
#include <map>
#include "ObjectDetection/common.h"
#endif

namespace Ogre
{
class SceneNode;
}

namespace rviz
{
class ColorProperty;
class FloatProperty;
class BoolProperty;
class EnumProperty;
}

// All the source in this plugin is in its own namespace.  This is not
// required but is good practice.
namespace tuw_object_rviz
{
class ObjectWithCovarianceVisual;

// Here we declare our new subclass of rviz::Display.  Every display
// which can be listed in the "Displays" panel is a subclass of
// rviz::Display.
class ObjectWithCovarianceArrayDisplay : public rviz::MessageFilterDisplay< tuw_object_msgs::ObjectWithCovarianceArray >
{
  Q_OBJECT
public:
  // Constructor.  pluginlib::ClassLoader creates instances by calling
  // the default constructor, so make sure you have one.
  ObjectWithCovarianceArrayDisplay();
  virtual ~ObjectWithCovarianceArrayDisplay();

  // Overrides of protected virtual functions from Display.  As much
  // as possible, when Displays are not enabled, they should not be
  // subscribed to incoming data and should not show anything in the
  // 3D view.  These functions are where these connections are made
  // and broken.
protected:
  virtual void onInitialize();

  // A helper to clear this display back to the initial state.
  virtual void reset();

  // These Qt slots get connected to signals indicating changes in the user-editable properties.
private Q_SLOTS:
  virtual void stylesChanged();

  // Function to handle an incoming ROS message.
private:
  void processMessage(const tuw_object_msgs::ObjectWithCovarianceArray::ConstPtr& msg);

  // Storage of the visual
  std::vector< boost::shared_ptr< ObjectWithCovarianceVisual > > visual_;

  // Properties
  rviz::BoolProperty* render_covariances_property_;
  rviz::BoolProperty* render_ids_property_;
  rviz::ColorProperty* color_property_;
  rviz::EnumProperty* style_property_;
};

}  // end namespace tuw_object_rviz

#endif  // OBJECT_DETECTION_DISPLAY_H
// %EndTag(FULL_SOURCE)%
