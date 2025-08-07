/******************************************************************
navigation goals logic under ROS 2

Features:
- multiple navigation goals logic and process
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2022-10-28: Initial version
2023-09-20: Add task plugin execution
2025-08-07: Migrate from ROS 1
2025-xx-xx: xxx
******************************************************************/
#pragma once
#include "base_plugin.h"

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
// #include <move_base_msgs/MoveBaseAction.h>
// #include <move_base_msgs/MoveBaseActionGoal.h>
// #include <actionlib/client/simple_action_client.h>

#include <mutex>

using VisualizeEta = std::function<void(const geometry_msgs::msg::Pose&, double)>;
using ExecutionState = std::function<void(int, std::shared_ptr<std::string> Info)>;
using WaypointPack = std::pair<geometry_msgs::msg::PoseStamped, bool>;

class GoalsHandle
{
public:
	enum State { STA_STANDBY = 0, STA_POINT_APPROACHED, STA_DONE, STA_ABORTED };

	class GoalPack
	{
	public:
		GoalPack() = default;
		GoalPack(const geometry_msgs::msg::Pose& Pose, bool IsRelative, const std::string& Task, bool IsLast = false)
			: request_pose_(Pose), absolute_pose_(Pose), is_relative_(IsRelative), task_(Task), is_last_(IsLast) {};
		GoalPack(const GoalPack& SrcObj)
		{
			*this = SrcObj;
		};
		~GoalPack() = default;
		GoalPack& operator=(const GoalPack& SrcObj)
		{
			if (this != &SrcObj)
			{
				request_pose_ = SrcObj.request_pose_;
				absolute_pose_ = SrcObj.absolute_pose_;
				is_relative_ = SrcObj.is_relative_;
				task_ = SrcObj.task_;
				is_last_ = SrcObj.is_last_;
			}

			return *this;
		};

	public:
		geometry_msgs::msg::Pose request_pose_;
		geometry_msgs::msg::Pose absolute_pose_;
		bool is_relative_{ false };
		std::string task_;
		bool is_last_{ false };
	};

public:
	using Twist = geometry_msgs::msg::TwistStamped;

public:
    GoalsHandle() = delete;
	GoalsHandle(std::shared_ptr<rclcpp::Node> Node, const std::string& Namespace, bool Remote = false);
    ~GoalsHandle() = default;

public:
	bool execute(const std::vector<WaypointPack>& WaypointPacks, double PointSpan, double StopSpan,
		bool Loop = false);
	bool execute(const std::vector<WaypointPack>& WaypointPacks, const std::map<int, std::string>& Tasks,
		double PointSpan, double StopSpan, bool Loop = false);
	void cancel();
	void reset();
	void setLooping(bool Looping);
	void setPointSpan(double Span);
	void setStopSpan(double Span);
	geometry_msgs::msg::Pose getMapOrigin() const;
	geometry_msgs::msg::Pose getCurrentPose() const;
	void registerEatUpdater(VisualizeEta Func);
	void registerExecutionUpdater(ExecutionState Func);
	bool isActive() const;
	void unbindCallback();
	bool isMapReceived();
	void setTaskPlugin(std::shared_ptr<whi_rviz_plugins::BasePlugin> Plugin);
	void setBaselinkFrame(const std::string& Frame);
	void setStuckTimeout(double Timeout);
	void setRecoveryMaxTryCount(int Count);
	void setTolerance(double XyTolerance, double YawTolerance);
	void setUseStampedVel(bool Flag);

private:
	void setNamespace(const std::string& Namespace);
	void init(bool IsRemote = false);
	bool setGoal(const geometry_msgs::msg::Pose& Goal, bool Recovery = false);
	void cancelGoal() const;
	geometry_msgs::msg::TransformStamped listenTf(const std::string& DstFrame, const std::string& SrcFrame) const;
	void handleGoalAndState(const geometry_msgs::msg::PoseStamped& Pose);
	void updateStateInfo(const GoalPack& Goal);
	void subCallbackMapData(const nav_msgs::msg::MapMetaData::SharedPtr Msg);
	void subCallbackTwist(const Twist::SharedPtr Msg);
	void subCallbackTwistUnstamped(const geometry_msgs::msg::Twist::SharedPtr Msg);
	// void callbackGoalDone(const actionlib::SimpleClientGoalState& State,
	// 	const move_base_msgs::MoveBaseResultConstPtr& Result);
	// void callbackGoalActive();
	// void callbackGoalFeedback(const move_base_msgs::MoveBaseFeedbackConstPtr& Feedback);
	using NavigateToPose = nav2_msgs::action::NavigateToPose;
	using NavGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;
	void callbackNavGoalResponse(std::shared_future<NavGoalHandle::SharedPtr> Future);
	void callbackNavGoalFeedback(NavGoalHandle::SharedPtr GoalHandle,
		const std::shared_ptr<const NavigateToPose::Feedback> Feedback);
	void callbackNavGoalResult(const NavGoalHandle::WrappedResult& Result);
	void callbackTimer();
	int findBeginIndex(const std::vector<WaypointPack>& WaypointPacks);
	void executeTask(bool ForceClean = false);
	std::array<double, 3> locationDelta();
	bool isStill() const;
	bool metTolerance(const geometry_msgs::msg::Pose& PoseA, const geometry_msgs::msg::Pose& PoseB) const;
	geometry_msgs::msg::Pose constructAbsGoal(const geometry_msgs::msg::Pose& Relative) const;

private:
	static bool metDistance(const geometry_msgs::msg::Pose& Pose1, const geometry_msgs::msg::Pose& Pose2, double Tolerance);
	static double distance(const geometry_msgs::msg::Pose& Pose1, const geometry_msgs::msg::Pose& Pose2);

private:
	std::string namespace_;
    std::shared_ptr<rclcpp::Node> node_handle_{ nullptr };
	// nav2
	rclcpp_action::Client<NavigateToPose>::SharedPtr client_nav_{ nullptr };
	// tf
	std::shared_ptr<tf2_ros::Buffer> buffer_{ nullptr };
	std::unique_ptr<tf2_ros::TransformListener> tf_listener_{ nullptr };
    geometry_msgs::msg::Pose map_origin_;
    geometry_msgs::msg::Pose current_pose_;
	GoalPack active_goal_;
	std::list<GoalPack> goals_list_;
	bool looping_{ false };
	double point_span_{ 0.3 };
	double stop_span_{ 0.3 };
	double current_linear_{ 0.0001 };
	double current_angular_{ 0.0 };
	rclcpp::TimerBase::SharedPtr non_realtime_loop_{ nullptr };
	bool map_received_{ false };
	// subscriber
	rclcpp::Subscription<nav_msgs::msg::MapMetaData>::SharedPtr sub_map_data_{ nullptr };
	rclcpp::Subscription<Twist>::SharedPtr sub_twist_{ nullptr };
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_unstamped_{ nullptr };
	// updater
	VisualizeEta func_eta_{ nullptr };
	ExecutionState func_execution_state_{ nullptr };
	int waypoints_num_{ 0 };
	int loop_count_{ 0 };
	std::shared_ptr<whi_rviz_plugins::BasePlugin> task_plugin_{ nullptr };
	std::string baselink_frame_{ "base_link" };
	rclcpp::Time state_last_;
	bool state_task_{ false };
	double stuck_timeout_{ 10.0 };
	int stuck_relocate_count_{ 3 };
	int recovery_max_try_count_{ 3 };
	bool is_recovery_{ false };
	double xy_goal_tolerance_{ 0.15 };
	double yaw_goal_tolerance_{ 0.15 };
};
