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
#ifndef RVIZ_MOUSE_EVENT_HANDLER_H
#define RVIZ_MOUSE_EVENT_HANDLER_H

#include <QApplication>
#include <QMouseEvent>
#include <QWidget>
#include <ros/ros.h>

namespace whi_rviz_plugins
{
/**
 * a handler for mouse events in general
 * is pa
 */
class MouseEventHandler : public QObject
{
Q_OBJECT
public:
  MouseEventHandler( QObject* parent = 0 );
  virtual ~MouseEventHandler();

Q_SIGNALS:
  void mouseLeftButton( bool, int, int );
  void mouseLeftButtonCtrl( bool, int, int );
  void mouseLeftButtonShift( bool, int, int );
  void mouseRightButton( bool, int, int );
  void signalMouseLeftDoubleClick(int,int);

public Q_SLOTS:
  void mousePressEvent( QMouseEvent* event );
  void mouseReleaseEvent( QMouseEvent* event );
  void mouseDoubleClick(QMouseEvent * event);

};

} // namespace whi_rviz_plugins

#endif
