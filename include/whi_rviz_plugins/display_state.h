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
#include <whi_interfaces/msg/whi_state.hpp>
#include <whi_interfaces/msg/whi_battery.hpp>
#include <whi_interfaces/msg/whi_rc_state.hpp>
#include <whi_interfaces/msg/whi_temperature_humidity.hpp>

#include <rviz_common/display.hpp>
#include <rviz_common/panel_dock_widget.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <nav_msgs/msg/path.hpp>
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
		void onInitialize() override;
		void onEnable() override;
		void onDisable() override;

	protected Q_SLOTS:
		void updateTopicFeedback();
		void updateTopicPath();
		void updateTopicBattery();
		void updateTopicWhiState();
		void updateTopicEstop();
		void updateTopicRc();

    private:
		void subCallbackPath(const nav_msgs::msg::Path::SharedPtr Msg);
		void subCallbackBattery(const whi_interfaces::msg::WhiBattery::SharedPtr Msg);
		void subCallbackWhiState(const whi_interfaces::msg::WhiState::SharedPtr Msg);

	private Q_SLOTS:
		// these Qt slots get connected to signals indicating changes in the user-editable properties
        void updateBaselinkFrame();

	private:
		rviz_common::PanelDockWidget* frame_dock_{ nullptr };
		StatePanel* panel_{ nullptr };

		rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr node_rviz_weak_;

		// user-editable property variables
		// rviz_common::properties::RosTopicProperty* feedback_topic_property_;
		rviz_common::properties::RosTopicProperty* path_topic_property_;
		rviz_common::properties::RosTopicProperty* battery_topic_property_;
        rviz_common::properties::RosTopicProperty* whi_state_topic_property_;
		rviz_common::properties::RosTopicProperty* estop_topic_property_;
		rviz_common::properties::RosTopicProperty* rc_state_topic_property_;
		
		rviz_common::properties::TfFrameProperty* frame_property_;
        // subscriber
		rclcpp::Subscription<nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage>::SharedPtr sub_navi_feedback_;
		rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_{ nullptr };
		rclcpp::Subscription<whi_interfaces::msg::WhiBattery>::SharedPtr sub_battery_{ nullptr };
		rclcpp::Subscription<whi_interfaces::msg::WhiState>::SharedPtr sub_whi_state_{ nullptr };
		
        geometry_msgs::msg::Pose goal_;
	};
} // end namespace whi_rviz_plugins
