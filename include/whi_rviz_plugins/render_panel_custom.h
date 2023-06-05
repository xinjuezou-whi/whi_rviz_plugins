/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013-2015, Team ViGIR ( TORC Robotics LLC, TU Darmstadt, Virginia Tech, Oregon State University, Cornell University, and Leibniz University Hanover )
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Team ViGIR, TORC Robotics, nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
//@TODO_ADD_AUTHOR_INFO
#ifndef RVIZ_RENDER_PANEL_CUSTOM_H
#define RVIZ_RENDER_PANEL_CUSTOM_H

#include <ros/ros.h>
#include "rviz/render_panel.h"

namespace rviz
{

/**
 * A widget which shows an OGRE-rendered scene in RViz.
 *
 * RenderPanelCustom displays a scene and forwards mouse and key events to
 * the DisplayContext (which further forwards them to the active
 * Tool, etc.)
 */
class RenderPanelCustom: public RenderPanel
{
Q_OBJECT
public:
  /** Constructor.  Ogre::Root::createRenderWindow() is called within. */
  RenderPanelCustom( QWidget* parent = 0 );
  virtual ~RenderPanelCustom();

  //Ogre::Camera* getCamera() {return customCamera;};

  void setEventFilters(int function, bool block, Qt::KeyboardModifiers block_key_mod=Qt::NoModifier, Qt::MouseButtons block_mouse_buttons=Qt::NoButton);

  static enum
  {
      MOUSE_MOVE_EVENT=0,
      MOUSE_PRESS_EVENT=1,
      MOUSE_RELEASE_EVENT=2,
      MOUSE_DOUBLE_CLICK_EVENT=3,
      MOUSE_WHEEL_EVENT=4,
      MOUSE_LEAVE_EVENT=5,
      KEY_PRESS_EVENT=6
  } Events;

Q_SIGNALS:
  void signalMouseMoveEvent( QMouseEvent* event );
  void signalMousePressEvent( QMouseEvent* event );
  void signalMouseReleaseEvent( QMouseEvent* event );
  void signalMouseDoubleClickEvent( QMouseEvent* event );
  void signalMouseWheelEvent( QWheelEvent* event );
  void signalMouseLeaveEvent ( QEvent * event );
  void signalMouseEnterEvent ( QEvent * event );
  void signalKeyPressEvent( QKeyEvent* event );

protected:
  // Override from QWidget
  virtual void mouseMoveEvent( QMouseEvent* event );
  virtual void mousePressEvent( QMouseEvent* event );
  virtual void mouseReleaseEvent( QMouseEvent* event );
  virtual void mouseDoubleClickEvent( QMouseEvent* event );
  virtual void wheelEvent( QWheelEvent* event );
  virtual void leaveEvent ( QEvent * event );
  virtual void enterEvent( QEvent * event );
  virtual void keyPressEvent( QKeyEvent* event );

private:
  //Ogre::Camera customCamera;

  // define what will be sent over to rviz, gives total flexibility
  typedef struct {
      bool block;
      Qt::KeyboardModifiers block_key_modifiers;
      Qt::MouseButtons block_mouse_buttons;
  } BlockConfig;
  std::map<int,BlockConfig> block_config_;
};

} // namespace rviz

#endif

