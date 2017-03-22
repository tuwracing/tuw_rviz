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

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreMatrix3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <ObjectDetection/ObjectDetectionPersonVisual.h>

namespace tuw_object_rviz
{
ObjectDetectionPersonVisual::ObjectDetectionPersonVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node) : ObjectDetectionVisual(scene_manager, parent_node)
{
  person_visual_.reset(new CylinderPersonVisual(PersonVisualDefaultArgs(scene_manager_, frame_node_)));
}

ObjectDetectionPersonVisual::~ObjectDetectionPersonVisual()
{
  // empty destructor since parent destructor destroys frame node anyway
}

void ObjectDetectionPersonVisual::setMessage(const tuw_object_msgs::ObjectWithCovariance::ConstPtr& msg)
{
  // call parent class set Message to also display covariance, center, ids
  ObjectDetectionVisual::setMessage(msg);
  
  Ogre::Vector3 position = Ogre::Vector3(msg->object.pose.position.x, msg->object.pose.position.y, msg->object.pose.position.z);
  position = transform_ * position;
  position.z = 0;  // fix on ground z=0

  Ogre::Quaternion orientation = Ogre::Quaternion(msg->object.pose.orientation.w, msg->object.pose.orientation.x,
                                                  msg->object.pose.orientation.y, msg->object.pose.orientation.z);

  person_visual_->setPosition(position + Ogre::Vector3(0, 0, person_visual_->getHeight() / 2));
  person_visual_->setOrientation(orientation);
}

// Color is passed through to the pose Shape object.
void ObjectDetectionPersonVisual::setColor(Ogre::ColourValue color)
{
  ObjectDetectionVisual::setColor(color);
  person_visual_->setColor(color);
}

void ObjectDetectionPersonVisual::setStyle(Styles style)
{
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  Ogre::ColourValue color;
  PersonVisualDefaultArgs default_args(scene_manager_, frame_node_);
  
  position = person_visual_->getPosition();
  orientation = person_visual_->getOrientation();
  color = person_visual_->getColor();

  switch (style)
  {
    case STYLE_SIMPLE:
      person_visual_->setVisible(false);
      break;
    case STYLE_CYLINDER:
      person_visual_.reset(new CylinderPersonVisual(default_args));
      break;
    case STYLE_PERSON_MESHES:
      person_visual_.reset(new MeshPersonVisual(default_args));
      break;
    case STYLE_BOUNDING_BOXES:
      person_visual_.reset(new BoundingBoxPersonVisual(default_args));
      break;
  }
  
  person_visual_->setOrientation(orientation);
  person_visual_->setPosition(position);
  person_visual_->setColor(color);
  
}

}  // end namespace tuw_object_rviz
