/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

#ifndef AIRSKIN_DISPLAY_H
#define AIRSKIN_DISPLAY_H

#ifndef Q_MOC_RUN
#include <OgreVector3.h>
#include <OgreQuaternion.h>
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreEntity.h>
#include <tuw_airskin_msgs/AirskinPressures.h>
#include <tuw_airskin_msgs/AirskinColors.h>
#include <ros/ros.h>
#include "rviz/properties/ros_topic_property.h"
#include "rviz/default_plugin/marker_display.h"
#include <visualization_msgs/Marker.h>
#include "rviz/default_plugin/markers/arrow_marker.h"
#include "rviz/default_plugin/markers/shape_marker.h"
#endif

namespace Ogre
{
class SceneNode;
}

namespace rviz
{
class ColorProperty;
class FloatProperty;
class RosTopicProperty;
class MarkerBase;
class MarkerDisplay;
}

// All the source in this plugin is in its own namespace.  This is not
// required but is good practice.
namespace tuw_airskin_rviz
{

class AirSkinVisual;

// Here we declare our new subclass of rviz::Display.  Every display
// which can be listed in the "Displays" panel is a subclass of
// rviz::Display.
class AirSkinDisplay: public rviz::Display
{
    Q_OBJECT
public:
    // Constructor.  pluginlib::ClassLoader creates instances by calling
    // the default constructor, so make sure you have one.
    AirSkinDisplay();
    virtual ~AirSkinDisplay();

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
    void updateTopics();
private:
    ros::NodeHandle nh_;
    ros::Subscriber pressures_sub_;
    ros::Subscriber colors_sub_;
    void pressuresCallback(const tuw_airskin_msgs::AirskinPressures::ConstPtr& _pressures);
    void colorsCallback(const tuw_airskin_msgs::AirskinColors::ConstPtr& _colors);
    //Called periodically by rviz
    void update( float wall_dt, float ros_dt );

    std::unique_ptr<rviz::RosTopicProperty> pressures_topic_;
    std::unique_ptr<rviz::RosTopicProperty> colors_topic_;
    
    // Storage of the visual
    std::vector<std::shared_ptr<rviz::ArrowMarker>> arrows_;
    std::vector<std::shared_ptr<rviz::ShapeMarker>> boxes_;
    visualization_msgs::Marker defaultBoxMsg_;
    visualization_msgs::Marker defaultArrowMsg_;
    std::vector<visualization_msgs::Marker> arrowMsgs_;
    rviz::MarkerDisplay markerDisplay_;
    std::vector<unsigned int> min_press_;
    std::vector<unsigned int> max_press_;
};

} // end namespace tuw_pose_rviz_plugin

#endif // LINES_SEGMENTS_2D_DISPLAY_H
// %EndTag(FULL_SOURCE)%
