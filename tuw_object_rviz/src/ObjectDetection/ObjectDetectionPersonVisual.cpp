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
