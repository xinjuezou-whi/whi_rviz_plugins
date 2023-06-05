/******************************************************************
rviz plugin for robot model viewer

Features:
- OGRE view
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2023-06-05: Initial version
2022-xx-xx: xxx
******************************************************************/
#include "whi_rviz_plugins/display_robot_model_viewer.h"

#include <rviz/window_manager_interface.h>
#include <rviz/display_context.h>
#include <pluginlib/class_list_macros.h>

namespace whi_rviz_plugins
{
    DisplayRobotModelViewer::DisplayRobotModelViewer()
        : Display()
        , node_handle_(std::make_unique<ros::NodeHandle>())
    {
        std::cout << "\nWHI RViz plugin for viewing robot model VERSION 00.01" << std::endl;
        std::cout << "Copyright @ 2023-2024 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n" << std::endl;

        // topic_odom_property_ = new rviz::StringProperty("Odom topic", "odom",
        //     "Topic of odometry",
        //     this, SLOT(updateOdomTopic()));
        // topic_goal_property_ = new rviz::StringProperty("Goal topic", "goal",
        //     "Topic of navigation goal",
        //     this, SLOT(updateGoalTopic()));
        // frame_baselink_property_ = new rviz::StringProperty("baselink frame", "base_link",
        //     "Frame of base_link",
        //     this, SLOT(updateGoalTopic()));
    }

    DisplayRobotModelViewer::~DisplayRobotModelViewer()
    {
        delete frame_dock_;
    }
    
    void DisplayRobotModelViewer::onInitialize()
    {
        Display::onInitialize();

        panel_ = new RobotModelViewerPanel();
        rviz::WindowManagerInterface* windowContext = context_->getWindowManager();
        if (windowContext)
        {
            frame_dock_ = windowContext->addPane("Navi_robot_model_viewer", panel_); // getName() return "" ???
            frame_dock_->setIcon(getIcon()); // set the image name as same as the name of plugin
        }

        // updateOdomTopic();
        // updateGoalTopic();
        // updateBaselinkFrame();
    }

    PLUGINLIB_EXPORT_CLASS(whi_rviz_plugins::DisplayRobotModelViewer, rviz::Display)
} // end namespace whi_rviz_plugins
