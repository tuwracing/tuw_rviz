#include <ros/ros.h>

#include <OGRE/OgreMatrix3.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreVector3.h>

#include "WeightedPoseWithCovarianceArray/WeightedPoseWithCovarianceArrayVisual.h"

namespace tuw_geometry_rviz
{
WeightedPoseWithCovarianceArrayVisual::WeightedPoseWithCovarianceArrayVisual(Ogre::SceneManager *scene_manager,
                                                                             Ogre::SceneNode *parent_node)
{
  scene_manager_ = scene_manager;

  // Ogre::SceneNode s form a tree, with each node storing the
  // transform (position and orientation) of itself relative to its
  // parent.  Ogre does the math of combining those transforms when it
  // is time to render.
  //
  // Here we create a node to store the pose of the MarkerDetection's header
  // frame
  // relative to the RViz fixed frame.
  frame_node_ = parent_node->createChildSceneNode();

  count_ = 0;
}

WeightedPoseWithCovarianceArrayVisual::~WeightedPoseWithCovarianceArrayVisual()
{
  // Destroy the frame node since we don't need it anymore.
  scene_manager_->destroySceneNode(frame_node_);
}

Ogre::ColourValue WeightedPoseWithCovarianceArrayVisual::colorFromWeight(Ogre::ColourValue low, Ogre::ColourValue high,
                                                                         double weight)
{
  Ogre::ColourValue result;
  result[0] = weight * high[0] + (1 - weight) * low[0];
  result[1] = weight * high[1] + (1 - weight) * low[1];
  result[2] = weight * high[2] + (1 - weight) * low[2];
  result[3] = 0.2 + 0.8 * weight;
  return result;
}

void WeightedPoseWithCovarianceArrayVisual::setMessage(
    const tuw_geometry_msgs::WeightedPoseWithCovarianceArray::ConstPtr &msg)
{
  unsigned int i;

  if (count_ != msg->poses.size())
  {
    poses_.resize(msg->poses.size());
    variances_.resize(msg->poses.size());

    for (i = 0; i < msg->poses.size(); i++)
    {
      // We create the visual objects within the frame node so that we can
      // set thier position and direction relative to their header frame.
      poses_[i].reset(new rviz::Arrow(scene_manager_, frame_node_));
      variances_[i].reset(new rviz::Shape(rviz::Shape::Sphere, scene_manager_, frame_node_));
    }
  }
  count_ = msg->poses.size();

  double maxWeight = -DBL_MAX;
  for (i = 0; i < msg->poses.size(); i++)
  {
    double pw = msg->poses[i].weight;
    if (pw > 1)
    {
      ROS_WARN("[WeightedPoseWithCovarianceArrayVisual setMessage] Invalid "
               "weight: %f > 1",
               pw);
      pw = 0;
    }
    if (pw < 0)
    {
      ROS_WARN("[WeightedPoseWithCovarianceArrayVisual setMessage] Invalid "
               "weight: %f < 0",
               pw);
      pw = 0;
    }
    if (pw > maxWeight)
    {
      maxWeight = pw;
    }
  }

  for (i = 0; i < msg->poses.size(); i++)
  {
    tuw_geometry_msgs::WeightedPoseWithCovariance pose = msg->poses[i];

    double weight = pose.weight;
    weight = maxWeight > 0 ? weight / maxWeight : 0;

    Ogre::Vector3 position = Ogre::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
    Ogre::Quaternion orientation = Ogre::Quaternion(pose.pose.orientation.x, pose.pose.orientation.y,
                                                    pose.pose.orientation.z, pose.pose.orientation.w);

    // Arrow points in -Z direction, so rotate the orientation before display.
    // TODO: is it safe to change Arrow to point in +X direction?
    Ogre::Quaternion rotation1 = Ogre::Quaternion(Ogre::Degree(-90), Ogre::Vector3::UNIT_Y);
    Ogre::Quaternion rotation2 = Ogre::Quaternion(Ogre::Degree(-180), Ogre::Vector3::UNIT_X);
    orientation = rotation2 * rotation1 * orientation;

    poses_[i]->setPosition(position);
    poses_[i]->setOrientation(orientation);

    poses_[i]->setColor(colorFromWeight(color_pose_low_, color_pose_high_, weight));

    Ogre::Matrix3 C = Ogre::Matrix3(pose.covariance[6 * 0 + 0], pose.covariance[6 * 0 + 1], pose.covariance[6 * 0 + 5],
                                    pose.covariance[6 * 1 + 0], pose.covariance[6 * 1 + 1], pose.covariance[6 * 1 + 5],
                                    pose.covariance[6 * 5 + 0], pose.covariance[6 * 5 + 1], pose.covariance[6 * 5 + 5]);
    Ogre::Real eigenvalues[3];
    Ogre::Vector3 eigenvectors[3];
    C.EigenSolveSymmetric(eigenvalues, eigenvectors);
    if (eigenvalues[0] < 0)
    {
      ROS_WARN("[WeightedPoseWithCovarianceArrayVisual setMessage] "
               "eigenvalue[0]: %f < 0 ",
               eigenvalues[0]);
      eigenvalues[0] = 0;
    }
    if (eigenvalues[1] < 0)
    {
      ROS_WARN("[WeightedPoseWithCovarianceArrayVisual setMessage] "
               "eigenvalue[1]: %f < 0 ",
               eigenvalues[1]);
      eigenvalues[1] = 0;
    }
    if (eigenvalues[2] < 0)
    {
      ROS_WARN("[WeightedPoseWithCovarianceArrayVisual setMessage] "
               "eigenvalue[2]: %f < 0 ",
               eigenvalues[2]);
      eigenvalues[2] = 0;
    }

    variances_[i]->setColor(colorFromWeight(color_variance_, color_variance_, weight));
    variances_[i]->setPosition(position);
    variances_[i]->setOrientation(Ogre::Quaternion(eigenvectors[0], eigenvectors[1], eigenvectors[2]));
    variances_[i]->setScale(
        Ogre::Vector3(2 * sqrt(eigenvalues[0]), 2 * sqrt(eigenvalues[1]), 2 * sqrt(eigenvalues[2])));
  }
}

// Position is passed through to the SceneNode.
void WeightedPoseWithCovarianceArrayVisual::setFramePosition(const Ogre::Vector3 &position)
{
  frame_node_->setPosition(position);
}

// Orientation is passed through to the SceneNode.
void WeightedPoseWithCovarianceArrayVisual::setFrameOrientation(const Ogre::Quaternion &orientation)
{
  frame_node_->setOrientation(orientation);
}

// Scale is passed through to the pose Shape object.
void WeightedPoseWithCovarianceArrayVisual::setScalePose(float scale)
{
  scale_pose_ = scale;
}

// Color is passed through to the pose Shape object.
void WeightedPoseWithCovarianceArrayVisual::setColorPose(Ogre::ColourValue low, Ogre::ColourValue high)
{
  color_pose_low_ = low;
  color_pose_high_ = high;
}

// Color is passed through to the variance Shape object.
void WeightedPoseWithCovarianceArrayVisual::setColorVariance(Ogre::ColourValue color)
{
  color_variance_ = color;
}

}  // end namespace marker_rviz
