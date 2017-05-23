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
#include "AirSkinDisplay.h"
#include <tf/transform_broadcaster.h>

namespace tuw_airskin_rviz
{
// The constructor must have no arguments, so we can't give the
// constructor the parameters it needs to fully initialize.
AirSkinDisplay::AirSkinDisplay()
{
    pressures_topic_ = std::make_unique<rviz::RosTopicProperty>("Airskin Pressures Topic", "",
                                         QString::fromStdString(ros::message_traits::datatype<tuw_airskin_msgs::AirskinPressures>()),
                                         "tuw_airskin_msgs::AirskinPressures topic to subscribe to.", this, SLOT( updateTopics() ));
    colors_topic_ = std::make_unique<rviz::RosTopicProperty>("Airskin Colors Topic", "",
                                         QString::fromStdString(ros::message_traits::datatype<tuw_airskin_msgs::AirskinColors>()),
                                         "tuw_airskin_msgs::AirskinColors topic to subscribe to.", this, SLOT( updateTopics() ));
}

// After the top-level rviz::Display::initialize() does its own setup,
// it calls the subclass's onInitialize() function.  This is where we
// instantiate all the workings of the class.  We make sure to also
// call our immediate super-class's onInitialize() function, since it
// does important stuff setting up the message filter.
//
//  Note that "MFDClass" is a typedef of
// ``MessageFilterDisplay<message type>``, to save typing that long
// templated class name every time you need to refer to the
// superclass.
void AirSkinDisplay::onInitialize() {
    defaultBoxMsg_.pose.orientation.w = 1.0;
    defaultBoxMsg_.scale.x = 0.021;
    defaultBoxMsg_.scale.y = 0.101;
    defaultBoxMsg_.scale.z = 0.051;
    
    defaultArrowMsg_.pose.orientation.w = 1.0;
    defaultArrowMsg_.color.a = 1.0;
    defaultArrowMsg_.color.r = 1.0;
    defaultArrowMsg_.color.g = 1.0;
    defaultArrowMsg_.color.b = 1.0;
    defaultArrowMsg_.scale.y = 0.025;
    defaultArrowMsg_.scale.z = 0.025;
}

AirSkinDisplay::~AirSkinDisplay()
{
}

// Clear the visual by deleting its object.
void AirSkinDisplay::reset()
{
}

// This is our callback to handle an incoming message.
void AirSkinDisplay::update( float wall_dt, float ros_dt )
{
    //resize marker vectors and possibly fill if the message vector is of a different size
    if(arrowMsgs_.size() > arrows_.size()) {
         for(int i=0;i<(arrowMsgs_.size() - arrows_.size());i++) {
            arrows_.emplace_back(std::make_shared<rviz::ArrowMarker>(&markerDisplay_, context_, scene_node_));
            boxes_.emplace_back(std::make_shared<rviz::ShapeMarker>(&markerDisplay_, context_, scene_node_));
         }
    } else if(arrowMsgs_.size() <  arrows_.size()) {
        arrows_.resize(arrowMsgs_.size());
        boxes_.resize(arrowMsgs_.size());
    }
    
    //pass messages to markers
    //there is only one default box msg because the arrow msgs already contain all needed information (frame_id and color)
    for(int i=0;i<arrows_.size();i++) {
        arrows_[i]->setMessage(arrowMsgs_[i]);
        visualization_msgs::Marker boxMsg;
        boxMsg = defaultBoxMsg_;
        boxMsg.color = arrowMsgs_[i].color;
        boxMsg.header = arrowMsgs_[i].header;
        boxes_[i]->setMessage(boxMsg);
    }
}

void AirSkinDisplay::updateTopics() {
    colors_sub_ = nh_.subscribe<tuw_airskin_msgs::AirskinColors>(colors_topic_->getTopicStd(), 1, &AirSkinDisplay::colorsCallback, this );
    pressures_sub_ = nh_.subscribe<tuw_airskin_msgs::AirskinPressures>(pressures_topic_->getTopicStd(), 1, &AirSkinDisplay::pressuresCallback, this );
}

void AirSkinDisplay::colorsCallback(const tuw_airskin_msgs::AirskinColors::ConstPtr& _colors) {
    for(int i=0;i<arrowMsgs_.size() && i<_colors->colors.size();i++) {
        arrowMsgs_[i].color = _colors->colors[i];
    }
}

void AirSkinDisplay::pressuresCallback(const tuw_airskin_msgs::AirskinPressures::ConstPtr& _pressures) {
    //resize message vector and possibly fill if the received message contains a different amount of airskin measurements
    if(arrowMsgs_.size()<_pressures->pressures.size()) {
        for(int i=0;i<(_pressures->pressures.size() - arrowMsgs_.size());i++) {
            arrowMsgs_.emplace_back(defaultArrowMsg_);
            min_press_.emplace_back(UINT_MAX);
            max_press_.emplace_back(0);
        }
    } else if(arrowMsgs_.size()>_pressures->pressures.size()) {
        arrowMsgs_.resize(_pressures->pressures.size());
        min_press_.resize(_pressures->pressures.size());
        max_press_.resize(_pressures->pressures.size());
    }
    
    //Update min and max measurements for scaling
    for(int i=0;i<_pressures->pressures.size();i++) {
        if(_pressures->pressures[i]<min_press_[i]) {
            min_press_[i] = _pressures->pressures[i];
        }
        if(_pressures->pressures[i]>max_press_[i]) {
            max_press_[i] = _pressures->pressures[i];
        }
    }
    
    //scale according to min and max measurements
    //get frame id from pad name
    for(int i=0;i<arrowMsgs_.size();i++) {
        arrowMsgs_[i].scale.x = 0.02 + ((_pressures->pressures[i]-min_press_[i])/float(max_press_[i]+1000-min_press_[i]))/10;
        arrowMsgs_[i].header.frame_id = _pressures->frame_ids[i];
    }
}
    
    

    

}  // end namespace tuw_airskin_rviz_plugin

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(tuw_airskin_rviz::AirSkinDisplay, rviz::Display)
