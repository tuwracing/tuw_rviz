#ifndef WEIGHTED_POSE_WITH_COVARIANCE_ARRAY_DISPLAY_H
#define WEIGHTED_POSE_WITH_COVARIANCE_ARRAY_DISPLAY_H

#ifndef Q_MOC_RUN
#include <rviz/message_filter_display.h>
#include <tuw_geometry_msgs/WeightedPoseWithCovarianceArray.h>
#endif

namespace Ogre
{
class SceneNode;
}

namespace rviz
{
class ColorProperty;
class FloatProperty;
}

// All the source in this plugin is in its own namespace.  This is not
// required but is good practice.
namespace tuw_geometry_rviz
{
class WeightedPoseWithCovarianceArrayVisual;

// Here we declare our new subclass of rviz::Display.  Every display
// which can be listed in the "Displays" panel is a subclass of
// rviz::Display.
class WeightedPoseWithCovarianceArrayDisplay
    : public rviz::MessageFilterDisplay< tuw_geometry_msgs::WeightedPoseWithCovarianceArray >
{
  Q_OBJECT
public:
  // Constructor.  pluginlib::ClassLoader creates instances by calling
  // the default constructor, so make sure you have one.
  WeightedPoseWithCovarianceArrayDisplay();
  virtual ~WeightedPoseWithCovarianceArrayDisplay();

  // Overrides of protected virtual functions from Display.  As much
  // as possible, when Displays are not enabled, they should not be
  // subscribed to incoming data and should not show anything in the
  // 3D view.  These functions are where these connections are made
  // and broken.
protected:
  virtual void onInitialize();

  // A helper to clear this display back to the initial state.
  virtual void reset();

  // These Qt slots get connected to signals indicating changes in the
  // user-editable properties.
private Q_SLOTS:
  void updateScalePose();
  void updateColorPose();
  void updateColorVariance();

  // Function to handle an incoming ROS message.
private:
  void processMessage(const tuw_geometry_msgs::WeightedPoseWithCovarianceArray::ConstPtr &msg);

  // Storage of the visual
  boost::shared_ptr< WeightedPoseWithCovarianceArrayVisual > visual_;

  // User-editable property variables.
  rviz::FloatProperty *property_scale_pose_;
  rviz::ColorProperty *property_color_pose_high_;
  rviz::ColorProperty *property_color_pose_low_;
  rviz::ColorProperty *property_color_variance_;
};

}  // end namespace tuw_pose_rviz

#endif  // POSE_WITH_COVARIANCE_DISPLAY_H
// %EndTag(FULL_SOURCE)%
