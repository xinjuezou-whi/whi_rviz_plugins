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
		WaypointsPanel(VisualizeWaypoints FuncWaypoints, VisualizeEta FuncEta, QWidget* Parent = nullptr);
		~WaypointsPanel() override;

	public:
		void updateWaypoint(int Index, const geometry_msgs::Pose& Pose);
		void updateHeight(double Height);

	private:
		void fillWaypoint(int RowIndex, bool WithCurrent = false, const std::vector<double>* Point = nullptr);
		void retrieveWaypoints(std::vector<geometry_msgs::PoseStamped>& Waypoints) const;
		void visualizeWaypoints(int Row) const;
		void addButtonClicked();
		void insertButtonClicked();
		void removeButtonClicked();
		bool loadWaypoints(std::string File);
		void saveWaypoints(std::string File);

	private:
		Ui::NaviWaypoints* ui_{ nullptr };
		VisualizeWaypoints func_visualize_waypoints_{ nullptr };
		VisualizeEta func_visualize_eta_{ nullptr };
		std::unique_ptr<GoalsHandle> goals_{ nullptr };
	};
} // end namespace whi_rviz_plugins
