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

class GoalsHandle
{
public:
	enum State { STA_STANDBY = 0, STA_POINT_APPROACHED, STA_DONE, STA_ABORTED };

public:
    GoalsHandle() = delete;
	GoalsHandle(const std::string& Namespace, bool Remote = false);
    ~GoalsHandle() = default;

public:
	bool execute(const std::vector<geometry_msgs::Pose>& Waypoints, double PointSpan, double StopSpan,
		bool Loop = false);
	bool execute(const std::vector<geometry_msgs::Pose>& Waypoints, const std::map<int, std::string>& Tasks,
		double PointSpan, double StopSpan, bool Loop = false);
	void cancel();
	void reset();
	void setLooping(bool Looping);
	void setPointSpan(double Span);
	void setStopSpan(double Span);
	geometry_msgs::Pose getMapOrigin() const;
	geometry_msgs::Pose getCurrentPose();
	void registerEatUpdater(VisualizeEta Func);
	void registerExecutionUpdater(ExecutionState Func);
	bool isActive() const;
	void unbindCallback();
	bool isMapReceived();
	void setTaskPlugin(boost::shared_ptr<whi_rviz_plugins::BasePlugin> Plugin);
	void setBaselinkFrame(const std::string& Frame);
	void setStuckTimeout(double Timeout);

private:
	void setNamespace(const std::string& Namespace);
	void init(bool IsRemote = false);
	bool setGoal(const geometry_msgs::Pose& Goal);
	void cancelGoal() const;
	geometry_msgs::TransformStamped listenTf(const std::string& DstFrame, const std::string& SrcFrame,
        const ros::Time& Time);
	void handleGoalAndState(const geometry_msgs::Pose& Pose);
	void updateStateInfo();
	void subCallbackPlanPath(const nav_msgs::Path::ConstPtr& PlanPath);
	void subCallbackMapData(const nav_msgs::MapMetaData::ConstPtr& MapData);
	void subCallbackCmdVel(const geometry_msgs::Twist::ConstPtr& CmdVel);
	void callbackGoalDone(const actionlib::SimpleClientGoalState& State, const move_base_msgs::MoveBaseResultConstPtr& Result);
	void callbackGoalActive();
	void callbackGoalFeedback(const move_base_msgs::MoveBaseFeedbackConstPtr& Feedback);
	void callbackTimer(const ros::TimerEvent& Event);
	int findBeginIndex(const std::vector<geometry_msgs::Pose>& Waypoints);
	void executeTask();
	std::array<double, 3> locationDelta();
	bool isStill() const;

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
	std::list<std::tuple<geometry_msgs::Pose, std::string>> goals_list_;
	geometry_msgs::Pose active_goal_;
	geometry_msgs::Pose final_goal_;
	std::string active_goal_task_;
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
};
