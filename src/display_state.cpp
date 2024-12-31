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

#include <rviz/window_manager_interface.h>
#include <rviz/display_context.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/tf_frame_property.h>
#include <rviz/frame_manager.h>
#include <pluginlib/class_list_macros.h>

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
        , node_handle_(std::make_unique<ros::NodeHandle>())
    {
        std::cout << "\nWHI RViz plugin for motion state VERSION 00.09.1" << std::endl;
        std::cout << "Copyright @ 2023-2025 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n" << std::endl;

        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(buffer_);

        odom_topic_property_ = new rviz::RosTopicProperty("Odom topic", "odom", "nav_msgs/Odometry",
            "Topic of odometry",
            this, SLOT(updateOdomTopic()));
        goal_topic_property_ = new rviz::RosTopicProperty("Goal topic", "goal", "geometry_msgs/PoseStamped",
            "Topic of navigation goal",
            this, SLOT(updateGoalTopic()));
        motion_state_topic_property_ = new rviz::RosTopicProperty("Motion state topic", "motion_state",
            "whi_interfaces/WhiMotionState", "Topic of motion state",
            this, SLOT(updateMotionStateTopic()));
        battery_topic_property_ = new rviz::RosTopicProperty("Battery info topic", "battery_data",
            "whi_interfaces/WhiBattery", "Topic of battery info",
            this, SLOT(updateBatteryTopic()));
        rc_state_topic_property_ = new rviz::RosTopicProperty("Remote controller state topic", "rc_state",
            "whi_interfaces/WhiRcState", "Topic of remote controller state",
            this, SLOT(updateRcStateTopic()));
        arm_state_topic_property_ = new rviz::RosTopicProperty("manipulator state topic", "arm_motion_state",
            "whi_interfaces/WhiMotionState", "Topic of manipulator state",
            this, SLOT(updateArmStateTopic()));
        imu_topic_property_ = new rviz::RosTopicProperty("IMU topic", "imu_data",
            "sensor_msgs/Imu", "Topic of IMU data",
            this, SLOT(updateImuTopic()));
        estop_topic_property_ = new rviz::RosTopicProperty("Estop topic", "estop",
            "std_msgs/Bool", "Topic of EStop",
            this, SLOT(updateEstopTopic()));
        temp_hum_topic_property_ = new rviz::RosTopicProperty("temperature and humidity topic", "temperature_humidity",
            "whi_interfaces/WhiTemperatureHumidity", "Topic of environmental temperature and humidity",
            this, SLOT(updateTempHumTopic()));
        frame_manager_ = std::make_shared<rviz::FrameManager>();
        frame_property_ = new rviz::TfFrameProperty("base_frame", "base_link", "Base link frame of robot",
            this, frame_manager_.get(), false, SLOT(updateBaselinkFrame()));
    }

    DisplayState::~DisplayState()
    {
        delete frame_dock_;
    }
    
    void DisplayState::onInitialize()
    {
        Display::onInitialize();

        panel_ = new StatePanel();
        rviz::WindowManagerInterface* windowContext = context_->getWindowManager();
        if (windowContext)
        {
            frame_dock_ = windowContext->addPane("Navi_state", panel_); // getName() return "" ???
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

        // tf listener
        ros::Duration updateFreq = ros::Duration(0.2);
        non_realtime_loop_ = std::make_unique<ros::Timer>(
            node_handle_->createTimer(updateFreq, std::bind(&DisplayState::update, this, std::placeholders::_1)));
    }

    void DisplayState::update(const ros::TimerEvent& Event)
    {
        std::string etaStr("no info");
        if (fabs(velocities_.first) > 9.9e-4 || fabs(velocities_.second) > 9.9e-4)
        {
            auto tfBase2Map = listenTf("map", frame_property_->getFrame().toStdString(), ros::Time(0));
            geometry_msgs::Pose baselink;
            baselink.position.x = tfBase2Map.transform.translation.x;
            baselink.position.y = tfBase2Map.transform.translation.y;
            double dist = distance(baselink, goal_);
            etaStr = dist < 0.1 ? "arrived" : "in " + toStringWithPrecision(dist / fabs(velocities_.first), 2);
        }
        panel_->setEta(etaStr);
    }

    geometry_msgs::TransformStamped DisplayState::listenTf(const std::string& DstFrame, const std::string& SrcFrame,
        const ros::Time& Time)
    {
        try
        {
            if (buffer_.canTransform(DstFrame, SrcFrame, Time, ros::Duration(1.0)))
            {
                return buffer_.lookupTransform(DstFrame, SrcFrame, Time, ros::Duration(1.0));
            }
            else
            {
				auto pose = geometry_msgs::TransformStamped();
				pose.transform.rotation.w = 1.0;
                return pose;
            }
        }
        catch (tf2::TransformException &e)
        {
            ROS_ERROR("%s", e.what());

			auto pose = geometry_msgs::TransformStamped();
			pose.transform.rotation.w = 1.0;
            return pose;
        }
    }

    double DisplayState::distance(const geometry_msgs::Pose& Pose1, const geometry_msgs::Pose& Pose2)
    {
	    return sqrt(pow(Pose1.position.x - Pose2.position.x, 2.0) + pow(Pose1.position.y - Pose2.position.y, 2.0));
    }

    void DisplayState::subCallbackOdom(const nav_msgs::Odometry::ConstPtr& Odom)
    {
        velocities_.first = Odom->twist.twist.linear.x;
        velocities_.second = Odom->twist.twist.angular.z;
        panel_->setVelocities(velocities_.first, velocities_.second);
    }

    void DisplayState::subCallbackGoal(const geometry_msgs::PoseStamped::ConstPtr& Goal)
    {
        goal_ = Goal->pose;
        panel_->setGoal(goal_);
    }

    void DisplayState::subCallbackMotionState(const whi_interfaces::WhiMotionState::ConstPtr& MotionState)
    {
        panel_->setMotionState(MotionState);
    }

    void DisplayState::subCallbackBattery(const whi_interfaces::WhiBattery::ConstPtr& Battery)
    {
        panel_->setBatteryInfo(Battery->soc, Battery->soh);
    }

    void DisplayState::subCallbackRcState(const whi_interfaces::WhiRcState::ConstPtr& RcState)
    {
        panel_->setRcState(RcState);
    }

    void DisplayState::subCallbackArmState(const whi_interfaces::WhiMotionState::ConstPtr& ArmState)
    {
        panel_->setArmState(ArmState);
    }

    void DisplayState::subCallbackImu(const sensor_msgs::Imu::ConstPtr& Imu)
    {
        panel_->setImuState();
    }

    void DisplayState::subCallbackTempHum(const whi_interfaces::WhiTemperatureHumidity::ConstPtr& TempHum)
    {
        panel_->setTempHum(TempHum->temperature, TempHum->humidity);
    }

    void DisplayState::updateOdomTopic()
    {
        sub_odom_ = std::make_unique<ros::Subscriber>(node_handle_->subscribe<nav_msgs::Odometry>(
		    odom_topic_property_->getTopicStd(), 10,
            std::bind(&DisplayState::subCallbackOdom, this, std::placeholders::_1)));
    }

	void DisplayState::updateGoalTopic()
    {
        sub_goal_ = std::make_unique<ros::Subscriber>(node_handle_->subscribe<geometry_msgs::PoseStamped>(
		    goal_topic_property_->getTopicStd(), 10,
            std::bind(&DisplayState::subCallbackGoal, this, std::placeholders::_1)));
    }

    void DisplayState::updateMotionStateTopic()
    {
        sub_motion_state_ = std::make_unique<ros::Subscriber>(
            node_handle_->subscribe<whi_interfaces::WhiMotionState>(
		    motion_state_topic_property_->getTopicStd(), 10,
            std::bind(&DisplayState::subCallbackMotionState, this, std::placeholders::_1)));
    }

    void DisplayState::updateBatteryTopic()
    {
        sub_battery_ = std::make_unique<ros::Subscriber>(
            node_handle_->subscribe<whi_interfaces::WhiBattery>(
		    battery_topic_property_->getTopicStd(), 10,
            std::bind(&DisplayState::subCallbackBattery, this, std::placeholders::_1)));
    }

    void DisplayState::updateRcStateTopic()
    {
        sub_rc_state_ = std::make_unique<ros::Subscriber>(
            node_handle_->subscribe<whi_interfaces::WhiRcState>(
		    rc_state_topic_property_->getTopicStd(), 10,
            std::bind(&DisplayState::subCallbackRcState, this, std::placeholders::_1)));

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
            sub_arm_state_ = std::make_unique<ros::Subscriber>(
                node_handle_->subscribe<whi_interfaces::WhiMotionState>(
                arm_state_topic_property_->getTopicStd(), 10,
                std::bind(&DisplayState::subCallbackArmState, this, std::placeholders::_1)));
        }
    }

    void DisplayState::updateImuTopic()
    {
        sub_imu_ = std::make_unique<ros::Subscriber>(
            node_handle_->subscribe<sensor_msgs::Imu>(
		    imu_topic_property_->getTopicStd(), 10,
            std::bind(&DisplayState::subCallbackImu, this, std::placeholders::_1)));
    }

    void DisplayState::updateTempHumTopic()
    {
        sub_temp_hum_ = std::make_unique<ros::Subscriber>(
            node_handle_->subscribe<whi_interfaces::WhiTemperatureHumidity>(
		    temp_hum_topic_property_->getTopicStd(), 10,
            std::bind(&DisplayState::subCallbackTempHum, this, std::placeholders::_1)));
    }

    void DisplayState::updateEstopTopic()
    {
        panel_->setEstopTopic(estop_topic_property_->getTopicStd());
    }

    void DisplayState::updateBaselinkFrame()
    {
        // do nothing so far
    }

    PLUGINLIB_EXPORT_CLASS(whi_rviz_plugins::DisplayState, rviz::Display)
} // end namespace whi_rviz_plugins
