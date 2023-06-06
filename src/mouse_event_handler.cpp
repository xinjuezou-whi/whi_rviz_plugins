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
#include "whi_rviz_plugins/mouse_event_handler.h"

#include <iostream>

namespace whi_rviz_plugins
{
MouseEventHandler::MouseEventHandler(QObject* Parent)
    : QObject(Parent)
{
}

MouseEventHandler::~MouseEventHandler()
{
}

void MouseEventHandler::mousePressEvent(QMouseEvent* Event) 
{ 
    if (Event->button() == Qt::LeftButton)
    {
        Qt::KeyboardModifiers keyMod = Event->modifiers();
        bool shift = keyMod.testFlag(Qt::ShiftModifier);
        bool ctrl = keyMod.testFlag(Qt::ControlModifier);

        if (shift && !ctrl) // as long as shift is pressed
        {
            Q_EMIT mouseLeftButtonShift(true, Event->x(), Event->y());
        }
        if (ctrl && !shift) // if only ctrl is pressed
        {
            Q_EMIT mouseLeftButtonCtrl(true, Event->x(), Event->y());
        }

        if (!ctrl && !shift)
        {
            Q_EMIT mouseLeftButton(true, Event->x(), Event->y());
        }
    }
    else if (Event->button() == Qt::RightButton)
    {
        Q_EMIT mouseRightButton(true, Event->x(), Event->y());
    }
}

void MouseEventHandler::mouseReleaseEvent(QMouseEvent* Event)
{
    if (Event->button() == Qt::LeftButton)
    {
        if (Event->modifiers() & Qt::ShiftModifier) // as long as shift is pressed
        {
            // need to emit selection signal here
            Q_EMIT mouseLeftButtonShift(false, Event->x(), Event->y());
        }
        else if (Event->modifiers() == Qt::NoModifier)
        {
            Q_EMIT mouseLeftButton(false, Event->x(), Event->y());
        }
    }
}

void MouseEventHandler::mouseDoubleClick(QMouseEvent* Event)
{
    //should only handle left double click
    if (Event->button() == Qt::LeftButton)
    {
        Q_EMIT signalMouseLeftDoubleClick(Event->x(), Event->y());
    }
}

} // namespace whi_rviz_plugins
