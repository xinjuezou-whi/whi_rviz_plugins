/******************************************************************
navigation goals logic under ROS 1

Features:
- multiple navigation goals logic and process
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2022-10-28: Initial version
2022-xx-xx: xxx
******************************************************************/
#pragma once
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/MapMetaData.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <actionlib/client/simple_action_client.h>

#include <mutex>

using VisualizeEta = std::function<void(const geometry_msgs::Pose&, double)>;
using ExecutionState = std::function<void(int)>;

class GoalsHandle
{
public:
	enum State { STA_STANDBY = 0, STA_DONE, STA_ABORTED };

public:
    GoalsHandle();
    ~GoalsHandle() = default;

public:
	void execute(std::vector<geometry_msgs::Pose> Waypoints, double PointSpan, double StopSpan, bool Loop = false);
	void cancel();
	void setLooping(bool Looping);
	void setPointSpan(double Span);
	void setStopSpan(double Span);
	geometry_msgs::Pose getMapOrigin() const;
	geometry_msgs::Pose getCurrentPose();
	void registerEatUpdater(VisualizeEta Func);
	void registerExecutionUpdater(ExecutionState Func);

private:
	void setGoal(const geometry_msgs::Pose& Goal);
	void cancelGoal() const;
	void handleGoalAndState(const geometry_msgs::Pose& Pose);
	void handleGoalAndStateUx(const geometry_msgs::Pose& Pose);
	void subCallbackPlanPath(const nav_msgs::Path::ConstPtr& PlanPath);
	void subCallbackMapData(const nav_msgs::MapMetaData::ConstPtr& MapData);
	void subCallbackEstimated(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& Estimated);
	void subCallbackCmdVel(const geometry_msgs::Twist::ConstPtr& CmdVel);
	void callbackGoalDone(const actionlib::SimpleClientGoalState& State, const move_base_msgs::MoveBaseResultConstPtr& Result);
	void callbackGoalActive();
	void callbackGoalFeedback(const move_base_msgs::MoveBaseFeedbackConstPtr& Feedback);
	void callbackTimer(const ros::TimerEvent& Event);

private:
	static bool metDistance(const geometry_msgs::Pose& Pose1, const geometry_msgs::Pose& Pose2, double Tolerance);
	static double distance(const geometry_msgs::Pose& Pose1, const geometry_msgs::Pose& Pose2);

private:
    std::unique_ptr<ros::NodeHandle> node_handle_{ nullptr };
	using MoveBaseClient = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;
	std::unique_ptr<MoveBaseClient> movebase_client_{ nullptr };
    nav_msgs::Path::_poses_type traj_poses_;
    geometry_msgs::Pose map_origin_;
    geometry_msgs::Pose estimated_;
	std::list<geometry_msgs::Pose> goals_list_;
	geometry_msgs::Pose active_goal_;
	geometry_msgs::Pose final_goal_;
	bool looping_{ false };
	double point_span_{ 0.3 };
	double stop_span_{ 0.3 };
	double current_linear_{ 0.0001 };
	std::mutex mtx_estimated_;
	std::unique_ptr<ros::Timer> non_realtime_loop_{ nullptr };
	bool with_ux_{ true };
	// subscriber
	std::unique_ptr<ros::Subscriber> sub_planned_path_{ nullptr };
	std::unique_ptr<ros::Subscriber> sub_map_data_{ nullptr };
	std::unique_ptr<ros::Subscriber> sub_estimate_{ nullptr };
	std::unique_ptr<ros::Subscriber> sub_cmd_vel_{ nullptr };
	// updater
	VisualizeEta func_eta_{ nullptr };
	ExecutionState func_execution_state_{ nullptr };
};
