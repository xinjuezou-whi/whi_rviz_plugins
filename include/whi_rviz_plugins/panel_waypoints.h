/******************************************************************
rviz plugin for navigation waypoints

Features:
- waypoints
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2022-10-26: Initial version
2023-09-20: Add plugin mechanism
2023-xx-xx: xxx
******************************************************************/
#pragma once
#include "goals_handle.h"
#include "base_plugin.h"
#include <whi_interfaces/WhiMotionState.h>
#include <whi_interfaces/WhiRcState.h>

#include <rviz/panel.h>
#include <geometry_msgs/PoseStamped.h>
#include <pluginlib/class_loader.hpp>

#include <QWidget>

class QTimer;
class QPushButton;

namespace Ui
{
class NaviWaypoints;
}

namespace whi_rviz_plugins
{
	using VisualizeWaypoints = std::function<void(int, const std::vector<geometry_msgs::PoseStamped>&)>;

	// forward declaration
	class WaypointsDisplay;

	class WaypointsPanel : public QWidget
	{
		Q_OBJECT
	public:
		WaypointsPanel(VisualizeWaypoints FuncWaypoints, VisualizeEta FuncEta,
			QWidget* Parent = nullptr);
		~WaypointsPanel() override;

	public:
		void setRemoteFlag(bool Flag);
		void updateWaypoint(int Index, const geometry_msgs::Pose& Pose);
		void updateHeight(double Height);
		void setBaselinkFrame(const std::string& Frame);
		void setStuckTimeout(double Timeout);
		void setRecoveryMaxTryCount(int Count);
		void setTolerance(double XyTolerance, double YawTolerance);
		void setMotionStateTopic(const std::string& Topic);
		void setRcStateTopic(const std::string& Topic);

	private:
		void configureNs(const std::string& Namespace);
		void fillWaypoint(int RowIndex, bool WithCurrent = false, const std::vector<double>* Point = nullptr);
		void retrieveWaypoints(std::vector<geometry_msgs::PoseStamped>& Waypoints) const;
		void retrieveWaypoint(int Index, geometry_msgs::PoseStamped& Waypoint) const;
		void visualizeWaypoints(int Row) const;
		void addButtonClicked();
		void insertButtonClicked();
		void removeButtonClicked();
		void executionState(int State, std::shared_ptr<std::string> Info);
		bool loadWaypoints(std::string File);
		void saveWaypoints(std::string File);
		bool loadWaypointsNs(std::string File);
		void storeItem2Map(int RowIndex, bool Insert = true);
		void storeAll2Map(const std::string& Namespace);
		void addWaypoint();
		void insertWaypoint();
		double getYawFromPose(const geometry_msgs::Pose& Pose) const;
		void enableUi(bool Flag);
		bool nsExisted(const std::string& Namespace) const;
		bool loadPlugin(const std::string& Config, const std::string& Namespace);
		bool createTaskPlugin(const YAML::Node& Node, const std::string& Namespace);
		QPushButton* bindTaskPlugin(int Row);
		void refreshTasksMap();
		void subCallbackMotionState(const whi_interfaces::WhiMotionState::ConstPtr& MotionState);
		void subCallbackRcState(const whi_interfaces::WhiRcState::ConstPtr& RcState);
		void abort();
		bool isBypassed();

	private:
		Ui::NaviWaypoints* ui_{ nullptr };
		VisualizeWaypoints func_visualize_waypoints_{ nullptr };
		VisualizeEta func_visualize_eta_{ nullptr };
		std::map<std::string, std::unique_ptr<GoalsHandle>> goals_map_;
		std::string waypoints_file_;
		std::map<std::string, std::vector<geometry_msgs::Pose>> waypoints_map_;
		std::string ns_from_load_;
		bool is_remote_{ false };
		std::string pre_ns_;
		std::map<std::string, boost::shared_ptr<BasePlugin>> plugins_map_;
		using WaypointsTask = std::map<int, std::string>;
		std::map<std::string, WaypointsTask> tasks_map_;
		std::unique_ptr<pluginlib::ClassLoader<BasePlugin>> plugin_loader_{ nullptr };
		std::string task_plugin_name_;
		std::string baselink_frame_{ "base_link" };
		double stuck_timeout_{ 10.0 };
		int recovery_max_try_count_{ 3 };
		double xy_goal_tolerance_{ 0.15 };
		double yaw_goal_tolerance_{ 0.15 };
		std::unique_ptr<ros::Subscriber> sub_motion_state_{ nullptr };
		std::unique_ptr<ros::Subscriber> sub_rc_state_{ nullptr };
		std::unique_ptr<ros::NodeHandle> node_handle_{ nullptr };
		std::atomic_bool toggle_estop_{ false };
		std::atomic_bool toggle_collision_{ false };
		std::atomic_bool remote_mode_{ false };
	};
} // end namespace whi_rviz_plugins
