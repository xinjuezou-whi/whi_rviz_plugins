/******************************************************************
rviz plugin for robot model viewer

Features:
- OGRE view
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2023-06-05: Initial version
2022-xx-xx: xxx
******************************************************************/
#pragma once
#include <rviz/panel.h>

#include "whi_rviz_plugins/mouse_event_handler.h"

namespace Ui
{
class NaviRobotModelViewer;
}

namespace rviz
{
class Display;
class RenderPanel;
class VisualizationManager;
}

namespace whi_rviz_plugins
{
	class RobotModelViewerPanel : public QWidget
	{
		Q_OBJECT
	public:
		RobotModelViewerPanel(QWidget* Parent = nullptr);
		~RobotModelViewerPanel() override;

	public:

	private:
		void onViewIndexChanged(int Index, QWidget* Group);

	private:
		Ui::NaviRobotModelViewer* ui_{ nullptr };
        rviz::VisualizationManager* manager_;
        rviz::RenderPanel* render_panel_;
        rviz::Display* grid_;
		rviz::Display* robot_model_;
		MouseEventHandler* mouse_event_handler_;
	};
} // end namespace whi_rviz_plugins
