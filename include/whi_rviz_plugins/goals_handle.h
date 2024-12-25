/******************************************************************
navigation goals logic under ROS 1

Features:
- multiple navigation goals logic and process
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2022-10-28: Initial version
2023-09-20: Add task plugin execution
2023-xx-xx: xxx
******************************************************************/
#pragma once
#include "base_plugin.h"

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/MapMetaData.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <mutex>

using VisualizeEta = std::function<void(const geometry_msgs::Pose&, double)>;
using ExecutionState = std::function<void(int, std::shared_ptr<std::string> Info)>;
using WaypointPack = std::pair<geometry_msgs::PoseStamped, bool>;

class GoalsHandle
{
public:
	enum State { STA_STANDBY = 0, STA_POINT_APPROACHED, STA_DONE, STA_ABORTED };

	class GoalPack
	{
	public:
		GoalPack() = default;
		GoalPack(const geometry_msgs::Pose& Pose, bool IsRelative, const std::string& Task, bool IsLast = false)
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
		geometry_msgs::Pose request_pose_;
		geometry_msgs::Pose absolute_pose_;
		bool is_relative_{ false };
		std::string task_;
		bool is_last_{ false };
	};

public:
    GoalsHandle() = delete;
	GoalsHandle(const std::string& Namespace, bool Remote = false);
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
	geometry_msgs::Pose getMapOrigin() const;
	geometry_msgs::Pose getCurrentPose() const;
	void registerEatUpdater(VisualizeEta Func);
	void registerExecutionUpdater(ExecutionState Func);
	bool isActive() const;
	void unbindCallback();
	bool isMapReceived();
	void setTaskPlugin(boost::shared_ptr<whi_rviz_plugins::BasePlugin> Plugin);
	void setBaselinkFrame(const std::string& Frame);
	void setStuckTimeout(double Timeout);
	void setRecoveryMaxTryCount(int Count);
	void setTolerance(double XyTolerance, double YawTolerance);

private:
	void setNamespace(const std::string& Namespace);
	void init(bool IsRemote = false);
	bool setGoal(const geometry_msgs::Pose& Goal, bool Recovery = false);
	void cancelGoal() const;
	geometry_msgs::TransformStamped listenTf(const std::string& DstFrame, const std::string& SrcFrame,
        const ros::Time& Time) const;
	void handleGoalAndState(const geometry_msgs::PoseStamped& Pose);
	void updateStateInfo(const GoalPack& Goal);
	void subCallbackMapData(const nav_msgs::MapMetaData::ConstPtr& MapData);
	void subCallbackCmdVel(const geometry_msgs::Twist::ConstPtr& CmdVel);
	void callbackGoalDone(const actionlib::SimpleClientGoalState& State,
		const move_base_msgs::MoveBaseResultConstPtr& Result);
	void callbackGoalActive();
	void callbackGoalFeedback(const move_base_msgs::MoveBaseFeedbackConstPtr& Feedback);
	void callbackTimer(const ros::TimerEvent& Event);
	int findBeginIndex(const std::vector<WaypointPack>& WaypointPacks);
	void executeTask(bool ForceClean = false);
	std::array<double, 3> locationDelta();
	bool isStill() const;
	bool metTolerance(const geometry_msgs::Pose& PoseA, const geometry_msgs::Pose& PoseB) const;
	geometry_msgs::Pose constructAbsGoal(const geometry_msgs::Pose& Relative) const;

private:
	static bool metDistance(const geometry_msgs::Pose& Pose1, const geometry_msgs::Pose& Pose2, double Tolerance);
	static double distance(const geometry_msgs::Pose& Pose1, const geometry_msgs::Pose& Pose2);

private:
	std::string namespace_;
    std::unique_ptr<ros::NodeHandle> node_handle_{ nullptr };
	using MoveBaseClient = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;
	std::unique_ptr<MoveBaseClient> movebase_client_{ nullptr };
	tf2_ros::Buffer buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_{ nullptr };
    geometry_msgs::Pose map_origin_;
    geometry_msgs::Pose current_pose_;
	GoalPack active_goal_;
	std::list<GoalPack> goals_list_;
	bool looping_{ false };
	double point_span_{ 0.3 };
	double stop_span_{ 0.3 };
	double current_linear_{ 0.0001 };
	double current_angular_{ 0.0 };
	std::unique_ptr<ros::Timer> non_realtime_loop_{ nullptr };
	bool map_received_{ false };
	// subscriber
	std::unique_ptr<ros::Subscriber> sub_map_data_{ nullptr };
	std::unique_ptr<ros::Subscriber> sub_cmd_vel_{ nullptr };
	// updater
	VisualizeEta func_eta_{ nullptr };
	ExecutionState func_execution_state_{ nullptr };
	int waypoints_num_{ 0 };
	int loop_count_{ 0 };
	boost::shared_ptr<whi_rviz_plugins::BasePlugin> task_plugin_{ nullptr };
	std::string baselink_frame_{ "base_link" };
	ros::Time state_last_;
	bool state_task_{ false };
	double stuck_timeout_{ 10.0 };
	int stuck_relocate_count_{ 3 };
	int recovery_max_try_count_{ 3 };
	bool is_recovery_{ false };
	double xy_goal_tolerance_{ 0.15 };
	double yaw_goal_tolerance_{ 0.15 };
};
