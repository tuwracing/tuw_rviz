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
#pragma once

#include <tuw_nav_msgs/Track.h>
#include <rviz/ogre_helpers/shape.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/billboard_line.h>

namespace Ogre
{
class Vector3;
class Quaternion;
} // namespace Ogre

namespace rviz
{
class Shape;
}

namespace tuw_nav_rviz
{

class TrackVisual
{
public:
  TrackVisual(Ogre::SceneManager *scene_manager, Ogre::SceneNode *parent_node,
              Ogre::Matrix4 &transform);

  virtual ~TrackVisual();

  void setMessage(const tuw_nav_msgs::Track::ConstPtr &msg);

private:
  std::vector<boost::shared_ptr<rviz::Shape>> trackPtsXY_left_;
  std::vector<boost::shared_ptr<rviz::Shape>> trackPtsXY_right_;
  std::vector<boost::shared_ptr<rviz::Shape>> trackPtsXY_center_;

  Ogre::SceneNode *frame_node_;
  Ogre::SceneManager *scene_manager_;

  rviz::BillboardLine *leftBorder_line_;
  rviz::BillboardLine *rightBorder_line_;
  rviz::BillboardLine *waypoints_line_;

  Ogre::ColourValue colorLeft_;  // = blue
  Ogre::ColourValue colorRight_; // = yellow
  Ogre::ColourValue colorCenter_;

  // shape type of track markings
  rviz::Shape::Type shape_type_;

  Ogre::Matrix4 transform_;

  void setMarkingLine(
    const std::vector<tuw_nav_msgs::TrackMarking> &markings,
    rviz::BillboardLine *line,
    std::vector<boost::shared_ptr<rviz::Shape>> &visuals,
    Ogre::ColourValue color);

  void setWaypoints(const std::vector<geometry_msgs::Pose2D> &waypoints);

};

} // end namespace tuw_nav_rviz
