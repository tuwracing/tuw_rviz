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

#include <ros/ros.h>

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreMatrix3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include "ObjectDetection/ObjectDetectionVisual.h"

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
  covariance_.reset(new ProbabilityEllipseCovarianceVisual(scene_manager_, frame_node_));
  mean_.reset(new rviz::Shape(rviz::Shape::Sphere, scene_manager_, frame_node_));
  detection_id_.reset(new TextVisual(scene_manager_, frame_node_, Ogre::Vector3(0, 0, 0)));
}

ObjectDetectionVisual::~ObjectDetectionVisual()
{
  // Destroy the frame node since we don't need it anymore.
  scene_manager_->destroySceneNode(frame_node_);
}

void ObjectDetectionVisual::setMessage(const tuw_object_msgs::ObjectWithCovariance::ConstPtr& msg)
{
  Ogre::Vector3 position =
      Ogre::Vector3(msg->object.pose.position.x, msg->object.pose.position.y, msg->object.pose.position.z);
  position = transform_ * position;
  position.z = 0;  // fix on ground z=0

  Ogre::Vector3 vel = Ogre::Vector3(msg->object.twist.linear.x, msg->object.twist.linear.y, msg->object.twist.linear.z);

  Ogre::Quaternion orientation = Ogre::Quaternion(msg->object.pose.orientation.w, msg->object.pose.orientation.x,
                                                  msg->object.pose.orientation.y, msg->object.pose.orientation.z);

  covariance_->setVisible(true);
  Ogre::Matrix3 C = Ogre::Matrix3(msg->covariance_pose[0], msg->covariance_pose[1], msg->covariance_pose[2],
                                  msg->covariance_pose[3], msg->covariance_pose[4], msg->covariance_pose[5],
                                  msg->covariance_pose[6], msg->covariance_pose[7], msg->covariance_pose[8]);

  // rotate covariance matrix in right coordinates
  // cov(Ax) = A * cov(x) * AT
  Ogre::Matrix3 rotation_mat;
  transform_.extract3x3Matrix(rotation_mat);
  C = rotation_mat * C * rotation_mat.Transpose();

  covariance_->setOrientation(orientation);
  covariance_->setPosition(position);
  covariance_->setMeanCovariance(Ogre::Vector3(0, 0, 0), C);

  pose_->setPosition(position);
  pose_->setDirection(vel);

  // only show arrow if velocity > 0 (in any direction)
  if (vel == Ogre::Vector3::ZERO)
  {
    pose_->getSceneNode()->setVisible(false, true);
  }
  else
  {
    pose_->getSceneNode()->setVisible(true, true);
  }

  mean_->setPosition(position);
  mean_->setScale(Ogre::Vector3(0.1, 0.1, 0.1));

  detection_id_->setPosition(position - Ogre::Vector3(0, 0, 0.2));
  detection_id_->setCharacterHeight(0.2);

  // concatenate ids with confidences as string for display
  // only first two decimal digits are displayed for confidences
  std::string ids = "";
  std::vector<int>::const_iterator it_ids = msg->object.ids.begin();
  std::vector<double>::const_iterator it_conf = msg->object.ids_confidence.begin();

  for (; (it_ids != msg->object.ids.end()) || (it_conf != msg->object.ids_confidence.end()); it_ids++, it_conf++)
  {
    std::string conf = (boost::format("%.2f") % *it_conf).str();

    if (it_ids == (msg->object.ids.end() - 1))
    {
      ids += boost::lexical_cast<std::string>(*it_ids) + " (" + conf + ")";
    }
    else
    {
      ids += boost::lexical_cast<std::string>(*it_ids) + " (" + conf + ")" + ", ";
    }
  }

  detection_id_->setCaption(ids);
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

void ObjectDetectionVisual::setTransform(const Ogre::Vector3& position, const Ogre::Quaternion& orientation)
{
  transform_ = Ogre::Matrix4(orientation);
  transform_.setTrans(position);
}

// Scale is passed through to the pose Shape object.
void ObjectDetectionVisual::setScale(float scale)
{
  pose_->setScale(Ogre::Vector3(scale, scale, scale));
  mean_->setScale(Ogre::Vector3(scale, scale, scale));
  covariance_->setLineWidth(scale);
  scale_ = scale;
}

// Color is passed through to the pose Shape object.
void ObjectDetectionVisual::setColor(Ogre::ColourValue color)
{
  pose_->setColor(color);
  mean_->setColor(color);
  covariance_->setColor(color);
  detection_id_->setColor(color);
  color_ = color;
}

void ObjectDetectionVisual::setVisiblities(bool render_covariance, bool render_id, bool render_sensor_type)
{
  covariance_->setVisible(render_covariance);
  detection_id_->setVisible(render_id);
}

void ObjectDetectionVisual::setStyle(Styles style)
{
  
}

}  // end namespace marker_rviz
