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

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/frame_manager.h>

#include "ObjectDetection/ObjectDetectionDisplay.h"
#include "ObjectDetection/ObjectDetectionVisual.h"

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

namespace tuw_object_rviz
{
// The constructor must have no arguments, so we can't give the
// constructor the parameters it needs to fully initialize.
ObjectDetectionDisplay::ObjectDetectionDisplay()
{
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
void ObjectDetectionDisplay::onInitialize()
{
  MFDClass::onInitialize();
  
  visual_.clear();
  /*
  boost::shared_ptr<ObjectDetectionVisual> init_visual;
  init_visual.reset(new ObjectDetectionVisual(context_->getSceneManager(), scene_node_));
  visual_.push_back(init_visual);*/

  render_covariances_property =
      new rviz::BoolProperty("Render covariances", true, "Render covariance ellipses", this, SLOT(stylesChanged()));
  render_ids_property =
      new rviz::BoolProperty("Render detection IDs", true, "Render IDs of the detection", this, SLOT(stylesChanged()));
  render_ids_confidences_property =
      new rviz::BoolProperty("Render confidences", false, "Render detection confidences", this, SLOT(stylesChanged()));

  render_sensor_type_property =
      new rviz::BoolProperty("Render sensor type text", false,
                             "Render detection sensor type as text below detected object", this, SLOT(stylesChanged()));
}

ObjectDetectionDisplay::~ObjectDetectionDisplay()
{
  foreach(boost::shared_ptr<ObjectDetectionVisual>& object_detection_visual, visual_)
  {
    object_detection_visual.reset(new ObjectDetectionVisual(context_->getSceneManager(), scene_node_));
  }
}

// Clear the visual by deleting its object.
void ObjectDetectionDisplay::reset()
{
  MFDClass::reset();
}

void ObjectDetectionDisplay::objectVisualTypeChanged()
{
  //visual_->
  stylesChanged();
}

void ObjectDetectionDisplay::stylesChanged()
{
  /*foreach(boost::shared_ptr<ObjectDetectionVisual>& object_detection_visual, visual_)
  {
    
  }*/
}

// This is our callback to handle an incoming message.
void ObjectDetectionDisplay::processMessage(const tuw_object_msgs::ObjectDetection::ConstPtr& msg)
{
  // clear previous detections
  visual_.clear();
  
  // Now set or update the contents of the visual.
  stylesChanged();
  
  for(std::vector<tuw_object_msgs::ObjectWithCovariance>::const_iterator detected_object_it = msg->objects.begin(); detected_object_it != msg->objects.end(); detected_object_it++)
  {
    // create new visual object
    boost::shared_ptr<ObjectDetectionVisual> detected_object_visual;
    detected_object_visual = boost::shared_ptr<ObjectDetectionVisual>(new ObjectDetectionVisual(context_->getSceneManager(), scene_node_));
    
    tuw_object_msgs::ObjectWithCovarianceConstPtr detected_object = boost::shared_ptr<tuw_object_msgs::ObjectWithCovariance>(new tuw_object_msgs::ObjectWithCovariance(*detected_object_it));
    
    // Here we call the rviz::FrameManager to get the transform from the
    // fixed frame to the frame in the header of this Imu message.  If
    // it fails, we can't do anything else so we return.
    Ogre::Quaternion orientation;
    Ogre::Vector3 position;
    
    //ROS_INFO("msg frame id = %s", msg->header.frame_id.c_str());
    if (!context_->getFrameManager()->getTransform(msg->header, position, orientation))
    {
      ROS_DEBUG("Error transforming from frame '%s' to frame '%s'", msg->header.frame_id.c_str(),
                qPrintable(fixed_frame_));
      return;
    }
    //ROS_INFO("frame = %s", msg->header.frame_id.c_str());
    //ROS_INFO("position = (x=%f, x=%f, x=%f), orientation = (w=%f, x=%f, y=%f, z=%f)", position.x, position.y, position.z, orientation.w, orientation.x, orientation.y, orientation.z);
    
    // set frame position to origin of parent, then explicitly transform points in msg
    // the advantage of this is that one can treat every detected object the same
    // regardless of orientation of the coordinate system
    // e.g. z = 0 means on the ground for every detection
    detected_object_visual->setFramePosition(Ogre::Vector3(0, 0, 0));
    detected_object_visual->setFrameOrientation(Ogre::Quaternion(1, 0, 0, 0));
    detected_object_visual->setTransformPosition(position);
    detected_object_visual->setTransformOrientation(orientation);
    detected_object_visual->setMessage(detected_object);
    
    visual_.push_back(detected_object_visual);
  }
}

}  // end namespace tuw_geometry_rviz

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(tuw_object_rviz::ObjectDetectionDisplay, rviz::Display)
