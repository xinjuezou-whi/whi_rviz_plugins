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
#include <rviz/panel.h>
#include <OgrePrerequisites.h>
#include <OgreVector3.h>

namespace Ui
{
class NaviRobotModelViewer;
}

namespace rviz
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
		RobotModelViewerPanel(Ogre::SceneNode* SceneNode, QWidget* Parent = nullptr);
		~RobotModelViewerPanel() override;

	public:
		void setBackgroundColor(const QColor& Color);
		void setFixedFrame(const QString& Frame);
		void setRobotDescription(const QString& Description);
		void setTfPrefix(const QString& Prefix);
		void load(const rviz::Config& Config); // override;
        void save(rviz::Config Config) const; // override;

	private:
		void onViewIndexChanged(int Index, QWidget* Group);

	private:
		void updateCameraParams();

	private:
		Ui::NaviRobotModelViewer* ui_{ nullptr };
		Ogre::SceneNode* scene_node_{ nullptr };
        rviz::VisualizationManager* manager_;
        rviz::RenderPanel* render_panel_;
        rviz::Display* grid_;
		rviz::Display* robot_model_;
	};
} // end namespace whi_rviz_plugins
