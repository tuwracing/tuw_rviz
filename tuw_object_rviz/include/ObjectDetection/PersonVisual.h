/*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013-2015, Timm Linder, Social Robotics Lab, University of Freiburg
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

#ifndef PERSON_VISUAL_H
#define PERSON_VISUAL_H

#include <rviz/ogre_helpers/shape.h>
#include <rviz/ogre_helpers/billboard_line.h>

#include <OgreSceneNode.h>
#include <OgreAnimation.h>
#include <OgreSharedPtr.h>
#include <OgreEntity.h>

#include <resource_retriever/retriever.h>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

namespace tuw_object_rviz {
    // Abstract class for visuals which have got an adjustable line width
    class HasLineWidth {
    public:
        virtual void setLineWidth(double lineWidth) = 0;
    };

    // Default arguments that need to be supplied to all types of PersonVisual
    struct PersonVisualDefaultArgs {
        PersonVisualDefaultArgs(Ogre::SceneManager* sceneManager, Ogre::SceneNode* parentNode) : sceneManager(sceneManager), parentNode(parentNode) {}
        Ogre::SceneManager* sceneManager;
        Ogre::SceneNode* parentNode;
    };

    /// Base class for all person visualization types
    class PersonVisual {
    public:
        PersonVisual(const PersonVisualDefaultArgs& args);

        virtual ~PersonVisual();

        void setPosition(const Ogre::Vector3& position);

        const Ogre::Vector3& getPosition() const;

        void setOrientation(const Ogre::Quaternion& orientation);

        const Ogre::Quaternion& getOrientation() const;

        virtual void setScalingFactor(double scalingFactor);

        void setVisible(bool visible);

        Ogre::SceneNode* getParentSceneNode();

        virtual void update(float deltaTime);
        
        virtual void setColor(const Ogre::ColourValue& c) = 0;
        
        virtual Ogre::ColourValue& getColor() = 0;
        
        virtual double getHeight() = 0;

    protected:
        Ogre::SceneManager* m_sceneManager;
        Ogre::SceneNode *m_sceneNode, *m_parentSceneNode;
        Ogre::ColourValue m_color;
    };
    
    /** This helper class ensures that skeletons can be loaded from a package:// path **/
    class RosPackagePathResourceLoadingListener : public Ogre::ResourceLoadingListener
    {
    public:
        RosPackagePathResourceLoadingListener(const fs::path& parentPath);

        /** This event is called when a resource beings loading. */
        virtual Ogre::DataStreamPtr resourceLoading(const Ogre::String &name, const Ogre::String &group, Ogre::Resource *resource);

        virtual void resourceStreamOpened(const Ogre::String &name, const Ogre::String &group, Ogre::Resource *resource, Ogre::DataStreamPtr& dataStream);

        virtual bool resourceCollision(Ogre::Resource *resource, Ogre::ResourceManager *resourceManager);

    private:
        const fs::path& _parentPath;
        resource_retriever::MemoryResource _lastResource;
    };
    
    /// Visualization of a person as cylinder (body) + sphere (head)
    class CylinderPersonVisual : public PersonVisual {
    public:
        CylinderPersonVisual(const PersonVisualDefaultArgs& args);

        virtual ~CylinderPersonVisual();

        virtual void setColor(const Ogre::ColourValue& c);
        
        virtual Ogre::ColourValue& getColor();

        virtual double getHeight();

    private:
        rviz::Shape *m_bodyShape, *m_headShape;
    };

    /// Visualization of a person as a wireframe bounding box
    class BoundingBoxPersonVisual : public PersonVisual, public HasLineWidth {
    public:
        BoundingBoxPersonVisual ( const PersonVisualDefaultArgs& args, double height = 1.75, double width = 0.6, double scalingFactor = 1.0 );

        virtual ~BoundingBoxPersonVisual();

        virtual void setColor(const Ogre::ColourValue& c);
        
        virtual Ogre::ColourValue& getColor();

        virtual double getHeight();

        virtual void setLineWidth(double lineWidth);

        /*
        virtual void setScalingFactor(double scalingFactor);
        */

    protected:
        virtual void generateWireframe();

    private:
        rviz::BillboardLine *m_wireframe;
        double m_width, m_height, m_scalingFactor, m_lineWidth;
    };
    

    /// Visualization of a person as a mesh (walking human)
    class MeshPersonVisual : public PersonVisual {
    
    private:
        Ogre::SceneNode *m_childSceneNode;
        Ogre::Entity* entity_;
        Ogre::AnimationState* m_animationState;
        std::set<Ogre::MaterialPtr> materials_;
        float m_walkingSpeed;
    public:
        MeshPersonVisual ( const PersonVisualDefaultArgs& args );

        virtual ~MeshPersonVisual();

        virtual void update(float deltaTime);

        virtual void setColor(const Ogre::ColourValue& c);
        
        virtual Ogre::ColourValue& getColor();

        void setAnimationState(const std::string& nameOfAnimationState);

        void setWalkingSpeed(float walkingSpeed);

        virtual double getHeight() {
            return 1.75;
        }

        virtual void setScalingFactor(double scalingFactor) {
            // Not supported (for some reason causes the mesh to be mirrored vertically).
        }
    };

}

#endif // PERSON_VISUAL_H
