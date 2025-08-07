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
        , node_handle_(std::make_shared<rclcpp::Node>("DisplayState"))
        , buffer_(std::make_shared<tf2_ros::Buffer>(node_handle_->get_clock()))
        , tf_listener_(std::make_unique<tf2_ros::TransformListener>(*buffer_))
    {
        std::cout << "\nWHI RViz plugin for motion state VERSION 02.10.1" << std::endl;
        std::cout << "Copyright @ 2023-2026 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n" << std::endl;

        odom_topic_property_ = new rviz_common::properties::RosTopicProperty("Odom topic", "odom", "nav_msgs/Odometry",
            "Topic of odometry",
            this, SLOT(updateOdomTopic()));
        goal_topic_property_ = new rviz_common::properties::RosTopicProperty("Goal topic", "goal", "geometry_msgs/PoseStamped",
            "Topic of navigation goal",
            this, SLOT(updateGoalTopic()));
        motion_state_topic_property_ = new rviz_common::properties::RosTopicProperty("Motion state topic", "motion_state",
            "whi_interfaces/WhiMotionState", "Topic of motion state",
            this, SLOT(updateMotionStateTopic()));
        battery_topic_property_ = new rviz_common::properties::RosTopicProperty("Battery info topic", "battery_data",
            "whi_interfaces/WhiBattery", "Topic of battery info",
            this, SLOT(updateBatteryTopic()));
        rc_state_topic_property_ = new rviz_common::properties::RosTopicProperty("Remote controller state topic", "rc_state",
            "whi_interfaces/WhiRcState", "Topic of remote controller state",
            this, SLOT(updateRcStateTopic()));
        arm_state_topic_property_ = new rviz_common::properties::RosTopicProperty("manipulator state topic", "arm_motion_state",
            "whi_interfaces/WhiMotionState", "Topic of manipulator state",
            this, SLOT(updateArmStateTopic()));
        imu_topic_property_ = new rviz_common::properties::RosTopicProperty("IMU topic", "imu_data",
            "sensor_msgs/Imu", "Topic of IMU data",
            this, SLOT(updateImuTopic()));
        estop_topic_property_ = new rviz_common::properties::RosTopicProperty("Estop topic", "estop",
            "std_msgs/Bool", "Topic of EStop",
            this, SLOT(updateEstopTopic()));
        temp_hum_topic_property_ = new rviz_common::properties::RosTopicProperty("temperature and humidity topic", "temperature_humidity",
            "whi_interfaces/WhiTemperatureHumidity", "Topic of environmental temperature and humidity",
            this, SLOT(updateTempHumTopic()));
        frame_property_ = new rviz_common::properties::TfFrameProperty("base_frame", "base_link", "Base link frame of robot",
            this, nullptr, false, SLOT(updateBaselinkFrame()));
    }

    DisplayState::~DisplayState()
    {
        delete frame_dock_;
    }
    
    void DisplayState::onInitialize()
    {
        Display::onInitialize();

        frame_property_->setFrameManager(context_->getFrameManager());

        panel_ = new StatePanel(node_handle_);
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
                RCLCPP_WARN(node_handle_->get_logger(), "failed to cast parent window to QMainWindow");
            }
            frame_dock_->setIcon(getIcon()); // set the image name as same as the name of plugin
        }

        updateOdomTopic();
        updateGoalTopic();
        updateMotionStateTopic();
        updateBatteryTopic();
        updateRcStateTopic();
        updateArmStateTopic();
        updateEstopTopic();
        updateTempHumTopic();
        updateBaselinkFrame();

        auto period = std::chrono::duration<double>(0.2);
        non_realtime_loop_ = node_handle_->create_wall_timer(period,
            std::bind(&DisplayState::update, this));
    }

    void DisplayState::update()
    {
        std::string etaStr("no info");
        if (fabs(velocities_.first) > 9.9e-4 || fabs(velocities_.second) > 9.9e-4)
        {
            auto tfBase2Map = listenTf("map", frame_property_->getFrame().toStdString());
            geometry_msgs::msg::Pose baselink;
            baselink.position.x = tfBase2Map.transform.translation.x;
            baselink.position.y = tfBase2Map.transform.translation.y;
            double dist = distance(baselink, goal_);
            etaStr = dist < 0.1 ? "arrived" : "in " + toStringWithPrecision(dist / fabs(velocities_.first), 2);
        }

        panel_->setEta(etaStr);
    }

    geometry_msgs::msg::TransformStamped DisplayState::listenTf(const std::string& DstFrame, const std::string& SrcFrame) const
    {
        try
        {
            if (buffer_->canTransform(DstFrame, SrcFrame, tf2::TimePointZero, tf2::durationFromSec(1.0)))
            {
                return buffer_->lookupTransform(DstFrame, SrcFrame, tf2::TimePointZero, tf2::durationFromSec(1.0));
            }
            else
            {
                auto pose = geometry_msgs::msg::TransformStamped();
                pose.transform.rotation.w = 1.0;
                return pose;
            }
        }
        catch (tf2::TransformException &e)
        {
            RCLCPP_ERROR_STREAM(node_handle_->get_logger(), "\033[1;31m" << "failed to listen TF: " << e.what() <<
                "\033[0m");

            auto pose = geometry_msgs::msg::TransformStamped();
            pose.transform.rotation.w = 1.0;
            return pose;
        }
    }

    double DisplayState::distance(const geometry_msgs::msg::Pose& Pose1, const geometry_msgs::msg::Pose& Pose2)
    {
	    return sqrt(pow(Pose1.position.x - Pose2.position.x, 2.0) + pow(Pose1.position.y - Pose2.position.y, 2.0));
    }

    void DisplayState::subCallbackOdom(const nav_msgs::msg::Odometry::SharedPtr Msg)
    {
        velocities_.first = Msg->twist.twist.linear.x;
        velocities_.second = Msg->twist.twist.angular.z;

        panel_->setVelocities(velocities_.first, velocities_.second);
    }

    void DisplayState::subCallbackGoal(const geometry_msgs::msg::PoseStamped::SharedPtr Msg)
    {
        goal_ = Msg->pose;

        panel_->setGoal(goal_);
    }

    void DisplayState::subCallbackMotionState(const whi_interfaces::msg::WhiMotionState::SharedPtr Msg)
    {
        panel_->setMotionState(Msg);
    }

    void DisplayState::subCallbackBattery(const whi_interfaces::msg::WhiBattery::SharedPtr Msg)
    {
        panel_->setBatteryInfo(Msg->soc, Msg->soh);
    }

    void DisplayState::subCallbackRcState(const whi_interfaces::msg::WhiRcState::SharedPtr Msg)
    {
        panel_->setRcState(Msg);
    }

    void DisplayState::subCallbackArmState(const whi_interfaces::msg::WhiMotionState::SharedPtr Msg)
    {
        panel_->setArmState(Msg);
    }

    void DisplayState::subCallbackImu(const sensor_msgs::msg::Imu::SharedPtr Msg)
    {
        panel_->setImuState();
    }

    void DisplayState::subCallbackTempHum(const whi_interfaces::msg::WhiTemperatureHumidity::SharedPtr Msg)
    {
        panel_->setTempHum(Msg->temperature, Msg->humidity);
    }

    void DisplayState::updateOdomTopic()
    {
        sub_odom_ = node_handle_->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_property_->getTopicStd(), 10, std::bind(&DisplayState::subCallbackOdom, this, std::placeholders::_1));
    }

	void DisplayState::updateGoalTopic()
    {
        sub_goal_ = node_handle_->create_subscription<geometry_msgs::msg::PoseStamped>(
            goal_topic_property_->getTopicStd(), 10, std::bind(&DisplayState::subCallbackGoal, this, std::placeholders::_1));
    }

    void DisplayState::updateMotionStateTopic()
    {
        sub_motion_state_ = node_handle_->create_subscription<whi_interfaces::msg::WhiMotionState>(
            motion_state_topic_property_->getTopicStd(), 10, std::bind(&DisplayState::subCallbackMotionState, this, std::placeholders::_1));
    }

    void DisplayState::updateBatteryTopic()
    {
        sub_battery_ = node_handle_->create_subscription<whi_interfaces::msg::WhiBattery>(
            battery_topic_property_->getTopicStd(), 10, std::bind(&DisplayState::subCallbackBattery, this, std::placeholders::_1));
    }

    void DisplayState::updateRcStateTopic()
    {
        sub_rc_state_ = node_handle_->create_subscription<whi_interfaces::msg::WhiRcState>(
            rc_state_topic_property_->getTopicStd(), 10, std::bind(&DisplayState::subCallbackRcState, this, std::placeholders::_1));
        
        panel_->setRcStateTopic(rc_state_topic_property_->getTopicStd());
    }

    void DisplayState::updateArmStateTopic()
    {
        if (arm_state_topic_property_->getTopicStd().empty())
        {
            panel_->setArmState(nullptr);
        }
        else
        {
            sub_arm_state_ = node_handle_->create_subscription<whi_interfaces::msg::WhiMotionState>(
                arm_state_topic_property_->getTopicStd(), 10, std::bind(&DisplayState::subCallbackArmState, this, std::placeholders::_1));
        }
    }

    void DisplayState::updateImuTopic()
    {
        sub_imu_ = node_handle_->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_property_->getTopicStd(), 10, std::bind(&DisplayState::subCallbackImu, this, std::placeholders::_1));
    }

    void DisplayState::updateTempHumTopic()
    {
        sub_temp_hum_ = node_handle_->create_subscription<whi_interfaces::msg::WhiTemperatureHumidity>(
            temp_hum_topic_property_->getTopicStd(), 10, std::bind(&DisplayState::subCallbackTempHum, this, std::placeholders::_1));
    }

    void DisplayState::updateEstopTopic()
    {
        panel_->setEstopTopic(estop_topic_property_->getTopicStd());
    }

    void DisplayState::updateBaselinkFrame()
    {
        // do nothing so far
    }
} // end namespace whi_rviz_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(whi_rviz_plugins::DisplayState, rviz_common::Display)
