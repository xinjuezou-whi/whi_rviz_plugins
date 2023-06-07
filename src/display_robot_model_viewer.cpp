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
#include <rviz/properties/color_property.h>
#include <rviz/properties/string_property.h>
#include <pluginlib/class_list_macros.h>

namespace whi_rviz_plugins
{
    DisplayRobotModelViewer::DisplayRobotModelViewer()
        : Display()
        , node_handle_(std::make_unique<ros::NodeHandle>())
    {
        std::cout << "\nWHI RViz plugin for viewing robot model VERSION 00.04" << std::endl;
        std::cout << "Copyright @ 2023-2024 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n" << std::endl;

        color_property_ = new rviz::ColorProperty("Background Color", QColor(48, 48, 48),
            "Background color for the 3D view.", this, SLOT(updateBackgroundColor()));
        fixed_frame_property_ = new rviz::StringProperty("Fixed Frame", "base_link",
            "Frame into which all data is transformed before being displayed.",
            this, SLOT(updateFixedFrame()));
        robot_description_property_ = new rviz::StringProperty("Robot Description", "robot_description",
            "Name of the parameter to search for to load the robot description.",
            this, SLOT(updateRobotDescription()));
    }

    DisplayRobotModelViewer::~DisplayRobotModelViewer()
    {
        delete frame_dock_;
    }
    
    void DisplayRobotModelViewer::onInitialize()
    {
        Display::onInitialize();

        panel_ = new RobotModelViewerPanel(scene_node_);
        rviz::WindowManagerInterface* windowContext = context_->getWindowManager();
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

    void DisplayRobotModelViewer::load(const rviz::Config& Config)
    {
        rviz::Display::load(Config);
        panel_->load(Config);
    }

    void DisplayRobotModelViewer::save(rviz::Config Config) const
    {
        rviz::Display::save(Config);
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

    PLUGINLIB_EXPORT_CLASS(whi_rviz_plugins::DisplayRobotModelViewer, rviz::Display)
} // end namespace whi_rviz_plugins
