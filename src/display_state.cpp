/******************************************************************
rviz plugin for motion status

Features:
- kinematic info
- nave target ETA info
- indicators
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_rviz_plugins/display_state.h"

#include <rviz/window_manager_interface.h>
#include <rviz/display_context.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/ros_topic_property.h>
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
        std::cout << "\nWHI RViz plugin for motion state VERSION 00.01.2" << std::endl;
        std::cout << "Copyright @ 2023-2024 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n" << std::endl;

        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(buffer_);

        odom_topic_property_ = new rviz::RosTopicProperty("Odom topic", "odom", "nav_msgs/Odometry",
            "Topic of odometry",
            this, SLOT(updateOdomTopic()));
        goal_topic_property_ = new rviz::RosTopicProperty("Goal topic", "goal", "geometry_msgs/PoseStamped",
            "Topic of navigation goal",
            this, SLOT(updateGoalTopic()));
        motion_topic_inteface_property_ = new rviz::RosTopicProperty("Motion interface topic", "motion_interface",
            "whi_interfaces/WhiMotionInterface", "Topic of motion interface",
            this, SLOT(updateMotionInterfaceTopic()));
        frame_baselink_property_ = new rviz::StringProperty("baselink frame", "base_link",
            "Frame of base_link",
            this, SLOT(updateGoalTopic()));
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
        updateBaselinkFrame();

        // tf listener
        ros::Duration updateFreq = ros::Duration(0.2);
        non_realtime_loop_ = std::make_unique<ros::Timer>(
            node_handle_->createTimer(updateFreq, std::bind(&DisplayState::update, this, std::placeholders::_1)));
    }

    void DisplayState::update(const ros::TimerEvent& Event)
    {
        if (fabs(velocities_.first) > 0.0 || fabs(velocities_.second) > 0.0)
        {
            auto tfBase2Map = listenTf("map", frame_baselink_property_->getString().toStdString(), ros::Time(0));
            geometry_msgs::Pose baselink;
            baselink.position.x = tfBase2Map.transform.translation.x;
            baselink.position.y = tfBase2Map.transform.translation.y;
            double dist = distance(baselink, goal_);
            std::string etaStr = dist < 0.1 ? "arrived" : "in " + toStringWithPrecision(dist / velocities_.first, 2);
            panel_->setEta(etaStr);
        }
    }

    geometry_msgs::TransformStamped DisplayState::listenTf(const std::string& DstFrame, const std::string& SrcFrame,
        const ros::Time& Time)
    {
        try
        {
            return buffer_.lookupTransform(DstFrame, SrcFrame, Time, ros::Duration(1.0));
        }
        catch (tf2::TransformException &e)
        {
            ROS_ERROR("%s", e.what());
            return geometry_msgs::TransformStamped();
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

    void DisplayState::subCallbackMotionInterface(const whi_interfaces::WhiMotionInterface::ConstPtr& MotionInterface)
    {
        panel_->setMotionInterface(MotionInterface->state);
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

    void DisplayState::updateMotionInterfaceTopic()
    {
        sub_motion_interface_ = std::make_unique<ros::Subscriber>(
            node_handle_->subscribe<whi_interfaces::WhiMotionInterface>(
		    motion_topic_inteface_property_->getTopicStd(), 10,
            std::bind(&DisplayState::subCallbackMotionInterface, this, std::placeholders::_1)));
    }

    void DisplayState::updateBaselinkFrame()
    {
        // do nothing so far
    }

    PLUGINLIB_EXPORT_CLASS(whi_rviz_plugins::DisplayState, rviz::Display)
} // end namespace whi_rviz_plugins