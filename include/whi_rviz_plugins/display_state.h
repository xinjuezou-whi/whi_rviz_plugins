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
2022-xx-xx: xxx
******************************************************************/
#pragma once
#include <whi_interfaces/WhiMotionState.h>
#include <whi_interfaces/WhiBattery.h>
#include <whi_interfaces/WhiRcState.h>

#include <rviz/display.h>
#include <rviz/panel_dock_widget.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>

#include "panel_state.h"

// forward declaration
namespace rviz
{
	class RosTopicProperty;
	class TfFrameProperty;
	class FrameManager;
}

namespace whi_rviz_plugins
{
	// declare a new subclass of rviz::Display
	// every display which can be listed in the "Displays" panel is a subclass of rviz::Display
	class DisplayState : public rviz::Display 
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
        void update(const ros::TimerEvent& Event);
        geometry_msgs::TransformStamped listenTf(const std::string& DstFrame, const std::string& SrcFrame,
            const ros::Time& Time);
        double distance(const geometry_msgs::Pose& Pose1, const geometry_msgs::Pose& Pose2);
        void subCallbackOdom(const nav_msgs::Odometry::ConstPtr& Odom);
        void subCallbackGoal(const geometry_msgs::PoseStamped::ConstPtr& Goal);
		void subCallbackMotionState(const whi_interfaces::WhiMotionState::ConstPtr& MotionState);
		void subCallbackBattery(const whi_interfaces::WhiBattery::ConstPtr& Battery);
		void subCallbackRcState(const whi_interfaces::WhiRcState::ConstPtr& RcState);
		void subCallbackArmState(const whi_interfaces::WhiMotionState::ConstPtr& ArmState);
		void subCallbackImu(const sensor_msgs::Imu::ConstPtr& Imu);

	private Q_SLOTS:
		// these Qt slots get connected to signals indicating changes in the user-editable properties
		void updateOdomTopic();
		void updateGoalTopic();
		void updateMotionStateTopic();
		void updateBatteryTopic();
		void updateRcStateTopic();
		void updateArmStateTopic();
		void updateImuTopic();
		void updateEstopTopic();
        void updateBaselinkFrame();

	private:
        std::unique_ptr<ros::NodeHandle> node_handle_{ nullptr };
        std::unique_ptr<ros::Timer> non_realtime_loop_{ nullptr };
        rviz::PanelDockWidget* frame_dock_{ nullptr };
        StatePanel* panel_{ nullptr };
		// user-editable property variables
		rviz::RosTopicProperty* odom_topic_property_;
        rviz::RosTopicProperty* goal_topic_property_;
        rviz::RosTopicProperty* motion_state_topic_property_;
		rviz::RosTopicProperty* battery_topic_property_;
		rviz::RosTopicProperty* rc_state_topic_property_;
		rviz::RosTopicProperty* arm_state_topic_property_;
		rviz::RosTopicProperty* imu_topic_property_;
		rviz::RosTopicProperty* estop_topic_property_;
		rviz::TfFrameProperty* frame_property_;
		std::shared_ptr<rviz::FrameManager> frame_manager_{ nullptr };
        // subscriber
        std::unique_ptr<ros::Subscriber> sub_odom_{ nullptr };
        std::unique_ptr<ros::Subscriber> sub_goal_{ nullptr };
		std::unique_ptr<ros::Subscriber> sub_motion_state_{ nullptr };
		std::unique_ptr<ros::Subscriber> sub_battery_{ nullptr };
		std::unique_ptr<ros::Subscriber> sub_rc_state_{ nullptr };
		std::unique_ptr<ros::Subscriber> sub_arm_state_{ nullptr };
		std::unique_ptr<ros::Subscriber> sub_imu_{ nullptr };
        std::pair<double, double> velocities_;
        geometry_msgs::Pose goal_;
		tf2_ros::Buffer buffer_;
		std::unique_ptr<tf2_ros::TransformListener> tf_listener_{ nullptr };
	};
} // end namespace whi_rviz_plugins
