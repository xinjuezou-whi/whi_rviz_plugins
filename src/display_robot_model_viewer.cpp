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

#include <rviz_common/window_manager_interface.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/string_property.hpp>

namespace whi_rviz_plugins
{
    DisplayRobotModelViewer::DisplayRobotModelViewer()
        : Display()
    {
        std::cout << "\nWHI RViz plugin for viewing robot model VERSION 02.05.1" << std::endl;
        std::cout << "Copyright @ 2023-2026 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n" << std::endl;

        color_property_ = new rviz_common::properties::ColorProperty("Background Color", QColor(48, 48, 48),
            "Background color for the 3D view.", this, SLOT(updateBackgroundColor()));
        fixed_frame_property_ = new rviz_common::properties::StringProperty("Fixed Frame", "base_link",
            "Frame into which all data is transformed before being displayed.",
            this, SLOT(updateFixedFrame()));
        robot_description_property_ = new rviz_common::properties::StringProperty("Robot Description", "robot_description",
            "Name of the parameter to search for to load the robot description.",
            this, SLOT(updateRobotDescription()));
        tf_prefix_property_ = new rviz_common::properties::StringProperty("TF Prefix", "",
            "Robot Model normally assumes the link name is the same as the tf frame name.\
            This option allows you to set a prefix. Mainly useful for multi-robot situations.",
            this, SLOT(updateTfPrefix()));
    }

    DisplayRobotModelViewer::~DisplayRobotModelViewer()
    {
        delete frame_dock_;
    }
    
    void DisplayRobotModelViewer::onInitialize()
    {
        Display::onInitialize();

        // Access the abstract ROS Node and
        // in the process lock it for exclusive use until the method is done.
        // Get a pointer to the familiar rclcpp::Node for making subscriptions/publishers
        // (as per normal rclcpp code)
        node_handle_ = context_->getRosNodeAbstraction().lock()->get_raw_node();

        panel_ = new RobotModelViewerPanel(context_, scene_node_);
        rviz_common::WindowManagerInterface* windowContext = context_->getWindowManager();
        if (windowContext)
        {
            frame_dock_ = windowContext->addPane("Navi_robot_model_viewer", panel_); // getName() return "" ???
            frame_dock_->setIcon(getIcon()); // set the image name as same as the name of plugin
        }

        updateBackgroundColor();
        updateFixedFrame();
        updateRobotDescription();
    }

    void DisplayRobotModelViewer::update(float WallDt, float RosDt)
    {
        // panel_->updateCameraParams();
    }

    void DisplayRobotModelViewer::load(const rviz_common::Config& Config)
    {
        rviz_common::Display::load(Config);
        panel_->load(Config);
    }

    void DisplayRobotModelViewer::save(rviz_common::Config Config) const
    {
        rviz_common::Display::save(Config);
        panel_->save(Config);
    }

    void DisplayRobotModelViewer::updateBackgroundColor()
    {
        panel_->setBackgroundColor(color_property_->getColor());
    }

    void DisplayRobotModelViewer::updateFixedFrame()
    {
        panel_->setFixedFrame(fixed_frame_property_->getString());
    }

    void DisplayRobotModelViewer::updateRobotDescription()
    {
        panel_->setRobotDescription(robot_description_property_->getString());
    }

    void DisplayRobotModelViewer::updateTfPrefix()
    {
        panel_->setTfPrefix(tf_prefix_property_->getString());
    }
} // end namespace whi_rviz_plugins
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(whi_rviz_plugins::DisplayRobotModelViewer, rviz_common::Display)
