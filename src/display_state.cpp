/******************************************************************
rviz plugin for motion status

Features:
- kinematic info
- nave target ETA info
- indicators
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_rviz_plugins/display_state.h"

#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_common/window_manager_interface.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/properties/tf_frame_property.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <QMainWindow>

namespace whi_rviz_plugins
{
    template <typename T>
    std::string toStringWithPrecision(const T Value, const int Digits = 6)
    {
        std::ostringstream out;
        out.precision(Digits);
        out << std::fixed << Value;
        return out.str();
    }

    DisplayState::DisplayState()
        : Display()
    {
        std::cout << "\nWHI RViz plugin for motion state VERSION 02.12.4" << std::endl;
        std::cout << "Copyright @ 2023-2026 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n" << std::endl;

        // feedback_topic_property_ = new rviz_common::properties::RosTopicProperty("Navigation feedback topic", "navigate_to_pose/_action/feedback",
        //     "nav2_msgs/action/NavigateToPose/Impl/FeedbackMessage", "Topic of navigation feedback", this);
        path_topic_property_ = new rviz_common::properties::RosTopicProperty("Planned path topic", "plan",
            "", "Topic of planned path", this, SLOT(updateTopicPath()));
        battery_topic_property_ = new rviz_common::properties::RosTopicProperty("Battery info topic", "battery_data",
            "", "Topic of battery info", this, SLOT(updateTopicBattery()));
        whi_state_topic_property_ = new rviz_common::properties::RosTopicProperty("WHI state topic", "whi_state",
            "", "Topic of WHI state", this, SLOT(updateTopicWhiState()));
        estop_topic_property_ = new rviz_common::properties::RosTopicProperty("Estop topic", "estop",
            "", "Topic of EStop", this, SLOT(updateTopicEstop()));
        rc_state_topic_property_ = new rviz_common::properties::RosTopicProperty("Remote controller state topic", "rc_state",
            "", "Topic of remote controller state", this, SLOT(updateTopicRc()));
        frame_property_ = new rviz_common::properties::TfFrameProperty("base_frame", "base_link", "Base link frame of robot",
            this, nullptr, false, SLOT(updateBaselinkFrame()));
    }

    DisplayState::~DisplayState()
    {
        delete frame_dock_;
    }
    
    void DisplayState::onInitialize()
    {
        // Access the abstract ROS Node and
        // in the process lock it for exclusive use until the method is done.
        // Get a pointer to the familiar rclcpp::Node for making subscriptions/publishers
        // (as per normal rclcpp code)
        node_rviz_weak_ = context_->getRosNodeAbstraction();

        // feedback_topic_property_->initialize(context_->getRosNodeAbstraction());
        path_topic_property_->initialize(context_->getRosNodeAbstraction());
        battery_topic_property_->initialize(context_->getRosNodeAbstraction());
        whi_state_topic_property_->initialize(context_->getRosNodeAbstraction());
        estop_topic_property_->initialize(context_->getRosNodeAbstraction());
        rc_state_topic_property_->initialize(context_->getRosNodeAbstraction());
        
        panel_ = new StatePanel(context_);
        rviz_common::WindowManagerInterface* windowContext = context_->getWindowManager();
        if (windowContext)
        {
            frame_dock_ = windowContext->addPane("Navi_state", panel_); // getName() return "" ???
            auto main_window = dynamic_cast<QMainWindow*>(windowContext->getParentWindow());
            if (main_window)
            {
                main_window->addDockWidget(Qt::BottomDockWidgetArea, frame_dock_);
            }
            else
            {
                RCLCPP_WARN(rclcpp::get_logger("display_state"), "failed to cast parent window to QMainWindow");
            }
            frame_dock_->setIcon(getIcon()); // set the image name as same as the name of plugin
        }

        updateTopicFeedback();
        updateBaselinkFrame();

        frame_property_->setFrameManager(context_->getFrameManager());
    }

    void DisplayState::onEnable()
    {
        if (!isEnabled())
        {
            return;
        }

        updateTopicFeedback();
		updateTopicPath();
        updateTopicBattery();
        updateTopicWhiState();
		updateTopicEstop();
		updateTopicRc();
    }

    void DisplayState::onDisable()
    {
        sub_navi_feedback_.reset();
        sub_path_.reset();
        sub_battery_.reset();
        sub_whi_state_.reset();
    }

    void DisplayState::updateTopicFeedback()
    {
        rclcpp::Node::SharedPtr node = node_rviz_weak_.lock()->get_raw_node();
        if (node)
        {
            sub_navi_feedback_ = node->create_subscription<nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage>(
                "navigate_to_pose/_action/feedback",//feedback_topic_property_->getTopicStd(),
                1,
                [this](const nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage::SharedPtr Msg)
                {
                    std::string etaStr("remaining ");
                    etaStr += toStringWithPrecision(Msg->feedback.distance_remaining, 2) + "m, in " +
                        toStringWithPrecision(rclcpp::Duration(Msg->feedback.estimated_time_remaining).seconds(), 0) + "s";

                    panel_->setEta(etaStr);
                });
            context_->queueRender();
        }
    }

    void DisplayState::updateTopicPath()
    {
        sub_path_.reset();
        rclcpp::Node::SharedPtr node = node_rviz_weak_.lock()->get_raw_node();
        if (node)
        {
            sub_path_ = node->create_subscription<nav_msgs::msg::Path>(
                path_topic_property_->getTopicStd(), 1, std::bind(&DisplayState::subCallbackPath, this, std::placeholders::_1));
            context_->queueRender();
        }
    }

    void DisplayState::updateTopicBattery()
    {
        sub_battery_.reset();
        rclcpp::Node::SharedPtr node = node_rviz_weak_.lock()->get_raw_node();
        if (node)
        {
            sub_battery_ = node->create_subscription<whi_interfaces::msg::WhiBattery>(
                battery_topic_property_->getTopicStd(), 1,
                std::bind(&DisplayState::subCallbackBattery, this, std::placeholders::_1));
            context_->queueRender();
        }
    }

    void DisplayState::updateTopicWhiState()
    {
        sub_whi_state_.reset();
        rclcpp::Node::SharedPtr node = node_rviz_weak_.lock()->get_raw_node();
        if (node)
        {
            sub_whi_state_ = node->create_subscription<whi_interfaces::msg::WhiState>(
                whi_state_topic_property_->getTopicStd(), 1,
                std::bind(&DisplayState::subCallbackWhiState, this, std::placeholders::_1));
            context_->queueRender();
        }
    }

    void DisplayState::updateTopicEstop()
    {
        panel_->setEstopTopic(estop_topic_property_->getTopicStd());
    }

    void DisplayState::updateTopicRc()
    {   
        panel_->setRcStateTopic(rc_state_topic_property_->getTopicStd());
    }

    void DisplayState::subCallbackPath(const nav_msgs::msg::Path::SharedPtr Msg)
    {
        goal_ = Msg->poses.back().pose;
        
        panel_->setGoal(goal_);
    }

    void DisplayState::subCallbackBattery(const whi_interfaces::msg::WhiBattery::SharedPtr Msg)
    {
        panel_->setBatteryInfo(Msg->soc, Msg->soh);
    }

    void DisplayState::subCallbackWhiState(const whi_interfaces::msg::WhiState::SharedPtr Msg)
    {
        static auto lastID = std::string("");
        rclcpp::Clock rosClock(RCL_ROS_TIME);
        auto current = rosClock.now();
        static auto last = current;
        if ((current - last).seconds() > 0.1 || lastID != Msg->hardware_id)
        {
            panel_->setWhiState(Msg);

            lastID = Msg->hardware_id;
            last = current;
        }
    }

    void DisplayState::updateBaselinkFrame()
    {
        // do nothing so far
        panel_->setRobotFrame(frame_property_->getFrameStd());
    }
} // end namespace whi_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(whi_rviz_plugins::DisplayState, rviz_common::Display)
