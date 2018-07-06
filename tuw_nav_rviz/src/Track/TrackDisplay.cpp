/***************************************************************************
 *   Software License Agreement (BSD License)                              *
 *   Copyright (C) 2016 by George Todoran <george.todoran@tuwien.ac.at>        *
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

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/frame_manager.h>

#include <Track/TrackVisual.h>
#include <Track/TrackDisplay.h>

namespace tuw_nav_rviz
{

TrackDisplay::TrackDisplay()
{
    history_length_property_ = new rviz::IntProperty("History Length", 1,
                                                     "Number of prior tracks to display.",
                                                     this, SLOT(updateHistoryLength()));
    history_length_property_->setMin(1);
    history_length_property_->setMax(100);
}

void TrackDisplay::onInitialize()
{
    MFDClass::onInitialize();
    updateHistoryLength();
}

TrackDisplay::~TrackDisplay()
{
}

// Clear the visuals by deleting their objects.
void TrackDisplay::reset()
{
    MFDClass::reset();
    visuals_.clear();
}

// Set the number of past visuals to show.
void TrackDisplay::updateHistoryLength()
{
    visuals_.rset_capacity(history_length_property_->getInt());
}

// This is our callback to handle an incoming message.
void TrackDisplay::processMessage(const tuw_nav_msgs::Track::ConstPtr &msg)
{
    Ogre::Quaternion orientation;
    Ogre::Vector3 position;
    if (!context_->getFrameManager()->getTransform(msg->header.frame_id,
                                                   msg->header.stamp,
                                                   position, orientation))
    {
        ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
                  msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
        return;
    }

    Ogre::Matrix4 transform(orientation);
    transform.setTrans(position);

    // We are keeping a circular buffer of visual pointers.  This gets
    // the next one, or creates and stores it if the buffer is not full
    boost::shared_ptr<TrackVisual> visual;
    if (visuals_.full())
    {
        visual = visuals_.front();
    }
    else
    {
        visual.reset(new TrackVisual(context_->getSceneManager(), scene_node_, transform));
    }

    visual->setMessage(msg);

    // And send it to the end of the circular buffer
    visuals_.push_back(visual);
}

} // end namespace tuw_nav_rviz

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(tuw_nav_rviz::TrackDisplay, rviz::Display)
