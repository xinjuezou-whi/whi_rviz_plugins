/******************************************************************
rviz plugin for 2D line chart

Features:
- 2D line
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_rviz_plugins/display_2D_line_chart.h"

#include <pluginlib/class_list_macros.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/window_manager_interface.h>
#include <rviz/display_context.h>

namespace whi_rviz_plugins
{
    Display2DLineChart::Display2DLineChart()
        : Display()
    {
        std::cout << "\nWHI RViz plugin for 2D line chart VERSION 00.01.0" << std::endl;
        std::cout << "Copyright @ 2024-2025 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n" << std::endl;

        // enable_property_ = new rviz::BoolProperty("Enable teleop", true, "Toggle the functionality of teleop",
        //     this, SLOT(updateEnable()));
        // pub_frequency_property_ = new rviz::FloatProperty("Publish frequency(Hz)", 5.0, "Frequency of publishing the twist message",
        //     this, SLOT(updatePubFrequency()));
        // pub_frequency_property_->setMin(0.1);
        // pub_topic_property_ = new rviz::StringProperty("Twist message topic", "cmd_vel", "Topic of twist message",
        //     this, SLOT(updatePubTopic()));
        // linear_min_ = new rviz::FloatProperty("Min linear", 0.08, "Min limit of linear velocity",
        //     this, SLOT(updateLinearMin()));
        // linear_min_->setMin(0.0);
        // linear_max_ = new rviz::FloatProperty("Max linear", 1.5, "Max limit of linear velocity",
        //     this, SLOT(updateLinearMax()));
        // linear_max_->setMin(0.0);
        // linear_step_ = new rviz::FloatProperty("Linear step", 0.01, "Delta of linear per jog",
        //     this, SLOT(updateLinearStep()));
        // linear_step_->setMin(0.01);
        // angular_min_ = new rviz::FloatProperty("Min angular", 0.01, "Min limit of angular velocity",
        //     this, SLOT(updateAngularMin()));
        // angular_min_->setMin(0.0);
        // angular_max_ = new rviz::FloatProperty("Max angular", 1.57, "Max limit of angular velocity",
        //     this, SLOT(updateAngularMax()));
        // angular_max_->setMin(0.0);
        // angular_step_ = new rviz::FloatProperty("Angular step", 0.1, "Delta of angular per jog",
        //     this, SLOT(updateAngularStep()));
        // angular_step_->setMin(0.01);
        // motion_state_topic_property_ = new rviz::RosTopicProperty("Motion state topic", "motion_state",
        //     "whi_interfaces/WhiMotionState", "Topic of motion state",
        //     this, SLOT(updateMotionStateTopic()));
        data_topic_property_ = new rviz::RosTopicProperty("Data topic", "line_data_2D",
            "whi_interfaces/WhiLineChart2D", "Topic of 2D data",
            this, SLOT(updateDataTopic()));
    }

    Display2DLineChart::~Display2DLineChart()
    {
        delete frame_dock_;
    }
    
    void Display2DLineChart::onInitialize()
    {
        Display::onInitialize();

        panel_ = new LineChart2DPanel();
        rviz::WindowManagerInterface* windowContext = context_->getWindowManager();
        if (windowContext)
        {
            frame_dock_ = windowContext->addPane("2DLineChart", panel_); // getName() return "" ???
            frame_dock_->setIcon(getIcon()); // set the image name as same as the name of plugin
        }

        updateDataTopic();
    }

    // void DisplayTeleop::updateEnable()
    // {
    //     panel_->setPubFunctionality(enable_property_->getBool());
    // }

	// void DisplayTeleop::updatePubFrequency()
    // {
    //     panel_->setPubFrequency(pub_frequency_property_->getFloat());
    // }

	// void DisplayTeleop::updatePubTopic()
    // {
    //     panel_->setPubTopic(pub_topic_property_->getString().toStdString());
    // }

    // void DisplayTeleop::updateLinearMin()
    // {
    //     panel_->setLinearMin(linear_min_->getFloat());
    // }

    // void DisplayTeleop::updateLinearMax()
    // {
    //     panel_->setLinearMax(linear_max_->getFloat());
    // }

	// void DisplayTeleop::updateLinearStep()
    // {
    //     panel_->setLinearStep(linear_step_->getFloat());
    // }

    // void DisplayTeleop::updateAngularMin()
    // {
    //     panel_->setAngularMin(angular_min_->getFloat());
    // }

	// void DisplayTeleop::updateAngularMax()
    // {
    //     panel_->setAngularMax(angular_max_->getFloat());
    // }

	// void DisplayTeleop::updateAngularStep()
    // {
    //     panel_->setAngularStep(angular_step_->getFloat());
    // }

    // void DisplayTeleop::updateMotionStateTopic()
    // {
    //     panel_->setMotionStateTopic(motion_state_topic_property_->getTopicStd());
    // }

    void Display2DLineChart::updateDataTopic()
    {
        panel_->setDataTopic(data_topic_property_->getTopicStd());
    }

    PLUGINLIB_EXPORT_CLASS(whi_rviz_plugins::Display2DLineChart, rviz::Display)
} // end namespace whi_rviz_plugins
