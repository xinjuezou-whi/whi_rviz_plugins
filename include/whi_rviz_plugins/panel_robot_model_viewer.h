/******************************************************************
rviz plugin for robot model viewer

Features:
- OGRE view
- viewcontroller/viewmanager
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2023-06-05: Initial version
2022-xx-xx: xxx
******************************************************************/
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include <OgrePrerequisites.h>
#include <OgreVector3.h>

namespace Ui
{
	class NaviRobotModelViewer;
}

namespace rviz_common
{
	class Display;
	class DisplayContext;
	class RenderPanel;
	class VisualizationManager;
}

namespace whi_rviz_plugins
{
	class RobotModelViewerPanel : public QWidget
	{
		Q_OBJECT
	public:
		RobotModelViewerPanel(rviz_common::DisplayContext* DisplayContext, Ogre::SceneNode* SceneNode, QWidget* Parent = nullptr);
		~RobotModelViewerPanel() override;

	public:
		void setBackgroundColor(const QColor& Color);
		void setFixedFrame(const QString& Frame);
		void setRobotDescription(const QString& Description);
		void setTfPrefix(const QString& Prefix);
		void load(const rviz_common::Config& Config);
        void save(rviz_common::Config Config) const;

	private:
		void onViewIndexChanged(int Index, QWidget* Group);

	private:
		void updateCameraParams();

	private:
		Ui::NaviRobotModelViewer* ui_{ nullptr };
		rviz_common::DisplayContext* display_context_{ nullptr };
		Ogre::SceneNode* scene_node_{ nullptr };
        rviz_common::VisualizationManager* manager_{ nullptr };
        rviz_common::RenderPanel* render_panel_{ nullptr };
        rviz_common::Display* grid_{ nullptr };
		rviz_common::Display* robot_model_{ nullptr };
	};
} // end namespace whi_rviz_plugins
