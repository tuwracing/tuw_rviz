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

#include <ObjectDetection/TextVisual.h>

namespace tuw_object_rviz
{
TextVisual::TextVisual(Ogre::SceneManager* sceneManager, Ogre::SceneNode* parentNode,
                       Ogre::Vector3 position)
  : m_sceneManager(sceneManager)
{
  m_sceneNode = parentNode->createChildSceneNode();

  m_text = new rviz::MovableText("text");
  m_text->setTextAlignment(rviz::MovableText::H_CENTER, rviz::MovableText::V_BELOW);
  m_sceneNode->attachObject(m_text);

  setCharacterHeight(1.0);
  setPosition(position);
  setVisible(true);
}

TextVisual::~TextVisual()
{
  m_sceneManager->destroySceneNode(m_sceneNode->getName());
  delete m_text;
};

void TextVisual::setCharacterHeight(double characterHeight)
{
  m_text->setCharacterHeight(characterHeight);
  m_text->setSpaceWidth(0.3 * characterHeight);
}

double TextVisual::getCharacterHeight()
{
  return m_text->getCharacterHeight();
}

void TextVisual::setCaption(const std::string& caption)
{
  m_text->setCaption(caption);
}

void TextVisual::setPosition(const Ogre::Vector3& position)
{
  m_sceneNode->setPosition(position);
}

void TextVisual::setVisible(bool visible)
{
  m_sceneNode->setVisible(visible, true);
}

void TextVisual::setColor(const Ogre::ColourValue& c)
{
  m_text->setColor(c);
}

void TextVisual::showOnTop(bool onTop)
{
  m_text->showOnTop(onTop);
}

}
