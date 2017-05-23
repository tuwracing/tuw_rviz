/*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013-2015, Timm Linder, Social Robotics Lab, University of Freiburg
*  Copyright (c) 2006, Kai O. Arras, Autonomous Intelligent Systems Lab, University of Freiburg
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions are met:
*
*  * Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.
*  * Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*  * Neither the name of the copyright holder nor the names of its contributors
*    may be used to endorse or promote products derived from this software
*    without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
*  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
*  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
*  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
*  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
*  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
*  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef COVARIANCE_VISUAL_H
#define COVARIANCE_VISUAL_H

#include <rviz/ogre_helpers/shape.h>
#include <rviz/ogre_helpers/billboard_line.h>
#include <OGRE/OgreSceneManager.h>
#include <cmath>


namespace tuw_object_rviz {
    // Visualization of a covariance matrix
    class CovarianceVisual {
    public:
        CovarianceVisual(Ogre::SceneManager* sceneManager, Ogre::SceneNode* parentNode);

        virtual ~CovarianceVisual();

        void setPosition(const Ogre::Vector3& position);

        void setOrientation(const Ogre::Quaternion& orientation);

        void setVisible(bool visible);

        virtual void setColor(const Ogre::ColourValue& c) = 0;

        virtual void setLineWidth(float lineWidth) = 0;

        /// NOTE: It is assumed that the covariance matrix is already rotated into the target frame of the sceneNode!
        virtual void setMeanCovariance(const Ogre::Vector3& mean, const Ogre::Matrix3& cov) = 0;

    protected:
        Ogre::SceneManager* m_sceneManager;
        Ogre::SceneNode* m_sceneNode;
    };


    // 2D ellipse visualization of a covariance matrix
    class ProbabilityEllipseCovarianceVisual : public CovarianceVisual {
    public:
        ProbabilityEllipseCovarianceVisual(Ogre::SceneManager* sceneManager, Ogre::SceneNode* parentNode);

        virtual ~ProbabilityEllipseCovarianceVisual();

        virtual void setLineWidth(float lineWidth);

        virtual void setColor(const Ogre::ColourValue& c);

        virtual void setMeanCovariance(const Ogre::Vector3& mean, const Ogre::Matrix3& cov);

    private:
        rviz::BillboardLine* m_line;

        // Puts angle alpha into the interval [min..min+2*pi[
        double set_angle_to_range(double alpha, double min);

        // Calculates the points on a rotated ellipse given by center xc, yc, half axes a, b and angle phi.
        // Returns number of points np and points in Cart. coordinates
        void calc_ellipse(double xc, double yc, double a, double b, double phi, int& np, double*& xvec, double*& yvec);

        // Calculates the points on a 95%-iso-probability ellipse given by the bivarate RV with mean xc, yc
        // and covariance matrix sxx, syy, sxy. Returns number of points np and points in Cart. coordinates
        void calc_prob_elli_95(double xc, double yc, double sxx, double syy, double sxy, int& np, double*& x, double*& y);

        // Calculates the points on a 99%-iso-probability ellipse given by the bivarate RV with mean xc, yc
        // and covariance matrix sxx, syy, sxy. Returns number of points np and points in Cart. coordinates
        void calc_prob_elli_99(double xc, double yc, double sxx, double syy, double sxy, int& np, double*& x, double*& y);
    };

}

#endif // COVARIANCE_VISUAL_H

