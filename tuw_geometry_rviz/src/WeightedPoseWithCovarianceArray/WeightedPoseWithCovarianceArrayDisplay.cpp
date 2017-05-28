#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/frame_manager.h>

#include "WeightedPoseWithCovarianceArray/WeightedPoseWithCovarianceArrayDisplay.h"
#include "WeightedPoseWithCovarianceArray/WeightedPoseWithCovarianceArrayVisual.h"

namespace tuw_geometry_rviz {

// The constructor must have no arguments, so we can't give the
// constructor the parameters it needs to fully initialize.
WeightedPoseWithCovarianceArrayDisplay::WeightedPoseWithCovarianceArrayDisplay() {
    property_scale_pose_ = new rviz::FloatProperty ( "Scale Pose", 0.4,
            "Scale of the pose's pose.",
            this, SLOT ( updateScalePose() ) );
    property_scale_pose_->setMin ( 0 );
    property_scale_pose_->setMax ( 1 );

    property_color_pose_ = new rviz::ColorProperty ( "Color Pose", QColor ( 204, 51, 0 ),
            "Color to draw the pose's pose.",
            this, SLOT ( updateColorPose() ) );

    property_color_variance_ = new rviz::ColorProperty ( "Color Variance", QColor ( 204, 51, 204 ),
            "Color to draw the pose's variance.",
            this, SLOT ( updateColorVariance() ) );
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
void WeightedPoseWithCovarianceArrayDisplay::onInitialize() {
    MFDClass::onInitialize();
    visual_.reset ( new WeightedPoseWithCovarianceArrayVisual ( context_->getSceneManager(), scene_node_ ) );
}

WeightedPoseWithCovarianceArrayDisplay::~WeightedPoseWithCovarianceArrayDisplay() {
}

// Clear the visual by deleting its object.
void WeightedPoseWithCovarianceArrayDisplay::reset() {
    MFDClass::reset();
}

// Set the current scale for the visual's pose.
void WeightedPoseWithCovarianceArrayDisplay::updateScalePose() {
    float scale = property_scale_pose_->getFloat();
    visual_->setScalePose ( scale );
}

// Set the current color for the visual's pose.
void WeightedPoseWithCovarianceArrayDisplay::updateColorPose() {
    Ogre::ColourValue color = property_color_pose_->getOgreColor();
    visual_->setColorPose ( color );
}

// Set the current color for the visual's variance.
void WeightedPoseWithCovarianceArrayDisplay::updateColorVariance() {
    Ogre::ColourValue color = property_color_variance_->getOgreColor();
    visual_->setColorVariance ( color );
}

// This is our callback to handle an incoming message.
void WeightedPoseWithCovarianceArrayDisplay::processMessage ( const tuw_geometry_msgs::WeightedPoseWithCovarianceArray::ConstPtr& msg ) {
    // Here we call the rviz::FrameManager to get the transform from the
    // fixed frame to the frame in the header of this Imu message.  If
    // it fails, we can't do anything else so we return.
    Ogre::Quaternion orientation;
    Ogre::Vector3 position;

    if ( !context_->getFrameManager()->getTransform ( msg->header.frame_id, msg->header.stamp, position, orientation ) ) {
        ROS_DEBUG ( "Error transforming from frame '%s' to frame '%s'",
                    msg->header.frame_id.c_str(), qPrintable ( fixed_frame_ ) );
        return;
    }

    // Now set or update the contents of the visual.
    visual_->setMessage ( msg );
    visual_->setFramePosition ( position );
    visual_->setFrameOrientation ( orientation );
    visual_->setScalePose ( property_scale_pose_->getFloat() );
    visual_->setColorPose ( property_color_pose_->getOgreColor() );
    visual_->setColorVariance ( property_color_variance_->getOgreColor() );
}

} // end namespace tuw_geometry_rviz

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (tuw_geometry_rviz::WeightedPoseWithCovarianceArrayDisplay,rviz::Display )
