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

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreMatrix4.h>

#include <Track/TrackVisual.h>
#include <tuw_nav_msgs/TrackMarking.h>

namespace tuw_nav_rviz
{

TrackVisual::TrackVisual(Ogre::SceneManager *scene_manager,
                         Ogre::SceneNode *parent_node,
                         Ogre::Matrix4 &transform) : transform_(transform)
{
    scene_manager_ = scene_manager;
    frame_node_ = parent_node->createChildSceneNode();

    leftBorder_line_ = new rviz::BillboardLine(scene_manager_, frame_node_);
    rightBorder_line_ = new rviz::BillboardLine(scene_manager_, frame_node_);
    waypoints_line_ = new rviz::BillboardLine(scene_manager_, frame_node_);

    // initialize global variables
    colorLeft_ = Ogre::ColourValue(0, 0, 255);
    colorRight_ = Ogre::ColourValue(255, 255, 0);
    colorCenter_ = Ogre::ColourValue(255, 255, 255);
    shape_type_ = rviz::Shape::Sphere;
}

TrackVisual::~TrackVisual()
{
    delete leftBorder_line_;
    scene_manager_->destroySceneNode(frame_node_);
}

void TrackVisual::setMessage(const tuw_nav_msgs::Track::ConstPtr &msg)
{
    // static double timeOld_;
    // if (timeOld_ == msg->header.stamp.toSec())
    // {
    //     return;
    // }
    // timeOld_ = msg->header.stamp.toSec();

    setMarkingLine(msg->leftBorder, leftBorder_line_, trackPtsXY_left_, colorLeft_);
    setMarkingLine(msg->rightBorder, rightBorder_line_, trackPtsXY_right_, colorRight_);
    setWaypoints(msg->waypoints);
}

void TrackVisual::setMarkingLine(
    const std::vector<tuw_nav_msgs::TrackMarking> &markings,
    rviz::BillboardLine *line,
    std::vector<boost::shared_ptr<rviz::Shape>> &visuals,
    Ogre::ColourValue color)
{
    size_t nMarkings = markings.size();

    line->clear();
    line->setNumLines(1);
    line->setMaxPointsPerLine(nMarkings);
    line->setLineWidth(0.03);

    // Draw markers and line between
    visuals.resize(nMarkings);
    for (size_t i = 0; i < visuals.size(); ++i)
    {
        double p_x = markings[i].pose.x;
        double p_y = markings[i].pose.y;
        double p_z = 0;

        Ogre::Quaternion rotation;
        rotation.x = 0;
        rotation.y = 0;
        rotation.z = 0;
        rotation.w = 1;

        visuals[i].reset(new rviz::Shape(shape_type_, scene_manager_, frame_node_));
        visuals[i]->setColor(color);
        visuals[i]->setPosition(Ogre::Vector3(p_x, p_y, p_z));
        visuals[i]->setOrientation(rotation);
        visuals[i]->setScale(Ogre::Vector3(0.1, 0.1, 0.1));

        Ogre::Vector3 xpos = transform_ * Ogre::Vector3(p_x, p_y, p_z);
        line->addPoint(xpos, color);
    }
}

void TrackVisual::setWaypoints(const std::vector<geometry_msgs::Pose2D> &waypoints)
{
    size_t n = waypoints.size();

    waypoints_line_->clear();
    waypoints_line_->setNumLines(1);
    waypoints_line_->setMaxPointsPerLine(n);
    waypoints_line_->setLineWidth(0.03);

    // Draw markers and line between
    trackPtsXY_center_.resize(n);
    for (size_t i = 0; i < trackPtsXY_center_.size(); ++i)
    {
        double p_x = waypoints[i].x;
        double p_y = waypoints[i].y;
        double p_z = 0;

        Ogre::Quaternion rotation;
        rotation.x = 0;
        rotation.y = 0;
        rotation.z = 0;
        rotation.w = 1;

        trackPtsXY_center_[i].reset(new rviz::Shape(shape_type_, scene_manager_, frame_node_));
        trackPtsXY_center_[i]->setColor(colorCenter_);
        trackPtsXY_center_[i]->setPosition(Ogre::Vector3(p_x, p_y, p_z));
        trackPtsXY_center_[i]->setOrientation(rotation);
        trackPtsXY_center_[i]->setScale(Ogre::Vector3(0.1, 0.1, 0.1));

        Ogre::Vector3 xpos = transform_ * Ogre::Vector3(p_x, p_y, p_z);
        waypoints_line_->addPoint(xpos, colorCenter_);
    }
}

} // end namespace tuw_nav_rviz
