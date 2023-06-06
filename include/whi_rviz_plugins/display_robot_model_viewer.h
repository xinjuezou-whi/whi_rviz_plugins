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
#include <rviz/display.h>
#include <rviz/panel_dock_widget.h>

#include "panel_robot_model_viewer.h"

namespace rviz
{
	class ColorProperty;
	class StringProperty;
}

namespace whi_rviz_plugins
{
	// declare a new subclass of rviz::Display
	// every display which can be listed in the "Displays" panel is a subclass of rviz::Display
	class DisplayRobotModelViewer : public rviz::Display 
	{
		Q_OBJECT
	public:
		// pluginlib::ClassLoader creates instances by calling the default constructor,
		// so make sure you have one
		DisplayRobotModelViewer();
		virtual ~DisplayRobotModelViewer();

		// overrides of protected virtual functions from Display as much as possible,
		// when Displays are not enabled, they should not be subscribed to incoming data,
		// and should not show anything in the 3D view
		// these functions are where these connections are made and broken
	protected:
		virtual void onInitialize();

    private:
		void update(float WallDt, float RosDt) override;
		void load(const rviz::Config& Config) override;
		void save(rviz::Config Config) const override;

	private Q_SLOTS:
		// these Qt slots get connected to signals indicating changes in the user-editable properties
		void updateBackgroundColor();
		void updateFixedFrame();
		void updateRobotDescription();

	private:
        std::unique_ptr<ros::NodeHandle> node_handle_{ nullptr };
        rviz::PanelDockWidget* frame_dock_{ nullptr };
        RobotModelViewerPanel* panel_{ nullptr };
		// user-editable property variables
		rviz::ColorProperty* color_property_;
		rviz::StringProperty* fixed_frame_property_;
		rviz::StringProperty* robot_description_property_;
		// rviz::StringProperty* topic_odom_property_;
        // rviz::StringProperty* topic_goal_property_;
        // rviz::StringProperty* frame_baselink_property_;
	};
} // end namespace whi_rviz_plugins
