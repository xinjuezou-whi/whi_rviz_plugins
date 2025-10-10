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

Changelog:
2023-06-04: Initial version
2025-08-04: Migrate from ROS 1
2025-xx-xx: xxx
******************************************************************/
#pragma once
#include "panel_state.h"
#include <whi_interfaces/msg/whi_motion_state.hpp>
#include <whi_interfaces/msg/whi_battery.hpp>
#include <whi_interfaces/msg/whi_rc_state.hpp>
#include <whi_interfaces/msg/whi_temperature_humidity.hpp>

#include <rviz_common/display.hpp>
#include <rviz_common/panel_dock_widget.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// forward declaration
namespace rviz_common
{
	namespace properties
	{
		class RosTopicProperty;
		class TfFrameProperty;
	}
}

namespace whi_rviz_plugins
{
	// declare a new subclass of rviz_common::Display
	// every display which can be listed in the "Displays" panel is a subclass of rviz_common::Display
	class DisplayState : public rviz_common::Display 
	{
		Q_OBJECT
	public:
		// pluginlib::ClassLoader creates instances by calling the default constructor,
		// so make sure you have one
		DisplayState();
		virtual ~DisplayState();

		// overrides of protected virtual functions from Display as much as possible,
		// when Displays are not enabled, they should not be subscribed to incoming data,
		// and should not show anything in the 3D view
		// these functions are where these connections are made and broken
	protected:
		virtual void onInitialize();

    private:
		void subCallbackOdom(const nav_msgs::msg::Odometry::SharedPtr Msg);
		void subCallbackGoal(const geometry_msgs::msg::PoseStamped::SharedPtr Msg);
		void subCallbackMotionState(const whi_interfaces::msg::WhiMotionState::SharedPtr Msg);
		void subCallbackBattery(const whi_interfaces::msg::WhiBattery::SharedPtr Msg);
		void subCallbackRcState(const whi_interfaces::msg::WhiRcState::SharedPtr Msg);
		void subCallbackArmState(const whi_interfaces::msg::WhiMotionState::SharedPtr Msg);
		void subCallbackImu(const sensor_msgs::msg::Imu::SharedPtr Msg);
		void subCallbackTempHum(const whi_interfaces::msg::WhiTemperatureHumidity::SharedPtr Msg);

	private Q_SLOTS:
		// these Qt slots get connected to signals indicating changes in the user-editable properties
        void updateBaselinkFrame();

	private:
		rviz_common::PanelDockWidget* frame_dock_{ nullptr };
		StatePanel* panel_{ nullptr };

		rclcpp::Node::SharedPtr node_handle_{ nullptr };

		// humble: multiple topic properties and subscribtion introduce undefined behavior
		// based on node from context_->getRosNodeAbstraction().lock()->get_raw_node(),
		// taking the independent node so far and it is required to spin manually
		std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
		std::thread executor_thread_;

		// user-editable property variables
		rviz_common::properties::RosTopicProperty* odom_topic_property_;
        // rviz_common::properties::RosTopicProperty* goal_topic_property_;
		// rviz_common::properties::RosTopicProperty* feedback_topic_property_;
        rviz_common::properties::RosTopicProperty* motion_state_topic_property_;
		rviz_common::properties::RosTopicProperty* battery_topic_property_;
		rviz_common::properties::RosTopicProperty* rc_state_topic_property_;
		rviz_common::properties::RosTopicProperty* arm_state_topic_property_;
		rviz_common::properties::RosTopicProperty* imu_topic_property_;
		rviz_common::properties::RosTopicProperty* estop_topic_property_;
		rviz_common::properties::RosTopicProperty* temp_hum_topic_property_;
		rviz_common::properties::TfFrameProperty* frame_property_;
        // subscriber
		rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_{ nullptr };
		rclcpp::Subscription<whi_interfaces::msg::WhiMotionState>::SharedPtr sub_motion_state_{ nullptr };
		rclcpp::Subscription<whi_interfaces::msg::WhiBattery>::SharedPtr sub_battery_{ nullptr };
		rclcpp::Subscription<whi_interfaces::msg::WhiRcState>::SharedPtr sub_rc_state_{ nullptr };
		rclcpp::Subscription<whi_interfaces::msg::WhiMotionState>::SharedPtr sub_arm_state_{ nullptr };
		rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_{ nullptr };
		rclcpp::Subscription<whi_interfaces::msg::WhiTemperatureHumidity>::SharedPtr sub_temp_hum_{ nullptr };
		rclcpp::Subscription<nav2_msgs::action::NavigateToPose::Impl::SendGoalService::Request>::SharedPtr sub_goal_{ nullptr };
		rclcpp::Subscription<nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage>::SharedPtr sub_navi_feedback_;
        std::pair<double, double> velocities_;
        geometry_msgs::msg::Pose goal_;
	};
} // end namespace whi_rviz_plugins
