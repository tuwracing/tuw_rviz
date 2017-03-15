#ifndef OBJECT_DETECTION_PERSON_VISUAL_H
#define OBJECT_DETECTION_PERSON_VISUAL_H

#include <ObjectDetection/ObjectDetectionVisual.h>
#include <ObjectDetection/PersonVisual.h>

namespace tuw_object_rviz
{

class ObjectDetectionPersonVisual : public ObjectDetectionVisual
{
public:
     ObjectDetectionPersonVisual ( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node );
     ~ObjectDetectionPersonVisual ();
     void setMessage ( const tuw_object_msgs::ObjectWithCovariance::ConstPtr& msg );
     void setColor ( Ogre::ColourValue color );
     void setStyle ( Styles style );
private:
    boost::shared_ptr<PersonVisual> person_visual_;
};

} // end namespace tuw_object_rviz

#endif // OBJECT_DETECTION_PERSON_VISUAL_H
