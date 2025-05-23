/******************************************************************
rviz plugin for teleop motion

Features:
- teleop motion
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_rviz_plugins/display_teleop.h"

#include <pluginlib/class_list_macros.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/window_manager_interface.h>
#include <rviz/display_context.h>

namespace whi_rviz_plugins
{
    DisplayTeleop::DisplayTeleop()
        : Display()
    {
        std::cout << "\nWHI RViz plugin for teleop VERSION 00.08.1" << std::endl;
        std::cout << "Copyright @ 2022-2025 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n" << std::endl;

        enable_property_ = new rviz::BoolProperty("Enable teleop", true, "Toggle the functionality of teleop",
            this, SLOT(updateEnable()));
        pub_frequency_property_ = new rviz::FloatProperty("Publish frequency(Hz)", 5.0, "Frequency of publishing the twist message",
            this, SLOT(updatePubFrequency()));
        pub_frequency_property_->setMin(0.1);
        pub_topic_property_ = new rviz::StringProperty("Twist message topic", "cmd_vel", "Topic of twist message",
            this, SLOT(updatePubTopic()));
        linear_min_ = new rviz::FloatProperty("Min linear", 0.08, "Min limit of linear velocity",
            this, SLOT(updateLinearMin()));
        linear_min_->setMin(0.0);
        linear_max_ = new rviz::FloatProperty("Max linear", 1.5, "Max limit of linear velocity",
            this, SLOT(updateLinearMax()));
        linear_max_->setMin(0.0);
        linear_step_ = new rviz::FloatProperty("Linear step", 0.01, "Delta of linear per jog",
            this, SLOT(updateLinearStep()));
        linear_step_->setMin(0.01);
        angular_min_ = new rviz::FloatProperty("Min angular", 0.01, "Min limit of angular velocity",
            this, SLOT(updateAngularMin()));
        angular_min_->setMin(0.0);
        angular_max_ = new rviz::FloatProperty("Max angular", 1.57, "Max limit of angular velocity",
            this, SLOT(updateAngularMax()));
        angular_max_->setMin(0.0);
        angular_step_ = new rviz::FloatProperty("Angular step", 0.1, "Delta of angular per jog",
            this, SLOT(updateAngularStep()));
        angular_step_->setMin(0.01);
        motion_state_topic_property_ = new rviz::RosTopicProperty("Motion state topic", "motion_state",
            "whi_interfaces/WhiMotionState", "Topic of motion state",
            this, SLOT(updateMotionStateTopic()));
        sw_estop_topic_property_ = new rviz::RosTopicProperty("Software EStop topic", "estop",
            "std_msgs/Bool", "Topic of software EStop",
            this, SLOT(updateSwEstopTopic()));
        rc_state_topic_property_ = new rviz::RosTopicProperty("Remote controller state topic", "rc_state",
            "whi_interfaces/WhiRcState", "Topic of remote controller state",
            this, SLOT(updateRcStateTopic()));
    }

    DisplayTeleop::~DisplayTeleop()
    {
        delete frame_dock_;
    }
    
    void DisplayTeleop::onInitialize()
    {
        Display::onInitialize();

        panel_ = new TeleopPanel();
        rviz::WindowManagerInterface* windowContext = context_->getWindowManager();
        if (windowContext)
        {
            frame_dock_ = windowContext->addPane("Teleop", panel_); // getName() return "" ???
            frame_dock_->setIcon(getIcon()); // set the image name as same as the name of plugin
        }
        updateLinearMin();
        updateLinearMax();
        updateLinearStep();
        updateAngularMin();
        updateAngularMax();
        updateAngularStep();
        // these three are order critical
        updatePubTopic();
        updatePubFrequency();
        updateEnable();
        updateMotionStateTopic();
        updateSwEstopTopic();
        updateRcStateTopic();
    }

    void DisplayTeleop::updateEnable()
    {
        panel_->setPubFunctionality(enable_property_->getBool());
    }

	void DisplayTeleop::updatePubFrequency()
    {
        panel_->setPubFrequency(pub_frequency_property_->getFloat());
    }

	void DisplayTeleop::updatePubTopic()
    {
        panel_->setPubTopic(pub_topic_property_->getString().toStdString());
    }

    void DisplayTeleop::updateLinearMin()
    {
        panel_->setLinearMin(linear_min_->getFloat());
    }

    void DisplayTeleop::updateLinearMax()
    {
        panel_->setLinearMax(linear_max_->getFloat());
    }

	void DisplayTeleop::updateLinearStep()
    {
        panel_->setLinearStep(linear_step_->getFloat());
    }

    void DisplayTeleop::updateAngularMin()
    {
        panel_->setAngularMin(angular_min_->getFloat());
    }

	void DisplayTeleop::updateAngularMax()
    {
        panel_->setAngularMax(angular_max_->getFloat());
    }

	void DisplayTeleop::updateAngularStep()
    {
        panel_->setAngularStep(angular_step_->getFloat());
    }

    void DisplayTeleop::updateMotionStateTopic()
    {
        panel_->setMotionStateTopic(motion_state_topic_property_->getTopicStd());
    }

    void DisplayTeleop::updateSwEstopTopic()
    {
        panel_->setSwEstopTopic(sw_estop_topic_property_->getTopicStd());
    }

    void DisplayTeleop::updateRcStateTopic()
    {
        panel_->setRcStateTopic(rc_state_topic_property_->getTopicStd());
    }

    PLUGINLIB_EXPORT_CLASS(whi_rviz_plugins::DisplayTeleop, rviz::Display)
} // end namespace whi_rviz_plugins
