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

#include <ros/ros.h>

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreMatrix3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include "ObjectDetection/ObjectDetectionVisual.h"
//#include <ObjectDetection/CovarianceVisual.h>

namespace tuw_object_rviz
{
ObjectDetectionVisual::ObjectDetectionVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node)
{
  scene_manager_ = scene_manager;

  // Ogre::SceneNode s form a tree, with each node storing the
  // transform (position and orientation) of itself relative to its
  // parent.  Ogre does the math of combining those transforms when it
  // is time to render.
  //
  // Here we create a node to store the pose of the MarkerDetection's header frame
  // relative to the RViz fixed frame.
  frame_node_ = parent_node->createChildSceneNode();

  // We create the visual objects within the frame node so that we can
  // set thier position and direction relative to their header frame.
  pose_.reset(new rviz::Arrow(scene_manager_, frame_node_));
  covariance_visual_.reset(new ProbabilityEllipseCovarianceVisual(scene_manager_, frame_node_));
}

ObjectDetectionVisual::~ObjectDetectionVisual()
{
  // Destroy the frame node since we don't need it anymore.
  scene_manager_->destroySceneNode(frame_node_);
}

void ObjectDetectionVisual::setMessage(const tuw_object_msgs::ObjectWithCovariance::ConstPtr& msg)
{
  Ogre::Vector3 position = Ogre::Vector3(msg->object.pose.position.x, msg->object.pose.position.y, msg->object.pose.position.z);
  
  Ogre::Vector3 vel = Ogre::Vector3(msg->object.twist.linear.x, msg->object.twist.linear.y, msg->object.twist.linear.z);
  
  Ogre::Quaternion orientation = Ogre::Quaternion(msg->object.pose.orientation.w, msg->object.pose.orientation.x, msg->object.pose.orientation.y, msg->object.pose.orientation.z);
  
  // only use yaw since person is flat on the ground
  //orientation = Ogre::Quaternion(orientation.getRoll(), Ogre::Vector3(0,0,1));

  Ogre::Matrix3 C = Ogre::Matrix3(msg->covariance_pose[0], msg->covariance_pose[1], msg->covariance_pose[2],
                                  msg->covariance_pose[3], msg->covariance_pose[4], msg->covariance_pose[5],
                                  msg->covariance_pose[6], msg->covariance_pose[7], msg->covariance_pose[8]);
  
  //position = Ogre::Vector3(0, 0, 0);
  //orientation = Ogre::Quaternion(1, 0, 0, 0);
  
  // rotate covariance matrix in right coordinates
  // cov(Ax) = A * cov(x) * AT
  Ogre::Matrix3 rotation_mat;
  orientation_.ToRotationMatrix(rotation_mat);
  C = rotation_mat * C * rotation_mat.Transpose();
  
  //ROS_INFO("frame_rotation = (w=%f, x=%f, y=%f, z=%f)", frame_rotation.w, frame_rotation.x, frame_rotation.y, frame_rotation.z);
  //ROS_INFO("frame_translation = (x=%f, y=%f, z=%f)", frame_translation.x, frame_translation.y, frame_translation.z);
  
  //ROS_INFO("person position = (x=%f, y=%f, z=%f)", position.x, position.y, position.z);

  Ogre::Matrix4 transf(orientation_);
  transf.setTrans(position_);
  
  position = transf * position;
  position.z = 0; // fix on ground z=0
  
  covariance_visual_->setOrientation(orientation);
  covariance_visual_->setPosition(position);
  covariance_visual_->setMeanCovariance(Ogre::Vector3(0, 0, 0), C);
  Ogre::ColourValue color = Ogre::ColourValue(0, 1, 0, 1);
  covariance_visual_->setColor(color);
  
  pose_->setPosition(position);
  pose_->setDirection(vel);

}

// Position is passed through to the SceneNode.
void ObjectDetectionVisual::setFramePosition(const Ogre::Vector3& position)
{
  frame_node_->setPosition(position);
}

// Orientation is passed through to the SceneNode.
void ObjectDetectionVisual::setFrameOrientation(const Ogre::Quaternion& orientation)
{
  frame_node_->setOrientation(orientation);
}

void ObjectDetectionVisual::setTransformPosition(const Ogre::Vector3& position)
{
  position_ = position;
}

void ObjectDetectionVisual::setTransformOrientation(const Ogre::Quaternion& orientation)
{
  orientation_ = orientation;
}

// Scale is passed through to the pose Shape object.
void ObjectDetectionVisual::setScale(float scale)
{
  pose_->setScale(Ogre::Vector3(scale, scale, scale));
  scale_ = scale;
}

// Color is passed through to the pose Shape object.
void ObjectDetectionVisual::setColor(Ogre::ColourValue color)
{
  pose_->setColor(color);
  color_ = color;
}

}  // end namespace marker_rviz
