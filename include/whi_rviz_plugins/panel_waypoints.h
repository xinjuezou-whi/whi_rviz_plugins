/******************************************************************
rviz plugin for navigation waypoints

Features:
- waypoints
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2022-10-26: Initial version
2022-xx-xx: xxx
******************************************************************/
#pragma once
#include "goals_handle.h"

#include <rviz/panel.h>
#include <geometry_msgs/PoseStamped.h>

#include <QWidget>

class QTimer;

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

	private:
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
		void mapTrigger();
		double getYawFromPose(const geometry_msgs::Pose& Pose) const;
		void enableUi(bool Flag);
		bool nsExisted(const std::string& Namespace) const;

	private:
		enum TriggerState { TRIGGER_LOAD = 0, TRIGGER_ADD, TRIGGER_INSERT, TRIGGER_NA };
		int trigger_state_{ TRIGGER_NA };
		Ui::NaviWaypoints* ui_{ nullptr };
		VisualizeWaypoints func_visualize_waypoints_{ nullptr };
		VisualizeEta func_visualize_eta_{ nullptr };
		std::unique_ptr<GoalsHandle> goals_{ nullptr };
		std::string waypoints_file_;
		std::map<std::string, std::vector<geometry_msgs::Pose>> waypoints_map_;
		std::string ns_from_load_;
		bool is_remote_{ false };
		QTimer* timer_map_{ nullptr };
	};
} // end namespace whi_rviz_plugins
