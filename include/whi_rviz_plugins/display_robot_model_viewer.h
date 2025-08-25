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
2025-08-25: Migrate from ROS 1
2025-xx-xx: xxx
******************************************************************/
#pragma once
#include "panel_robot_model_viewer.h"

#include <rviz_common/display.hpp>
#include <rviz_common/panel_dock_widget.hpp>

namespace rviz_common
{
	namespace properties
	{
		class ColorProperty;
		class StringProperty;
	}
}

namespace whi_rviz_plugins
{
	// declare a new subclass of rviz::Display
	// every display which can be listed in the "Displays" panel is a subclass of rviz::Display
	class DisplayRobotModelViewer : public rviz_common::Display 
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
		void load(const rviz_common::Config& Config) override;
		void save(rviz_common::Config Config) const override;

	private Q_SLOTS:
		// these Qt slots get connected to signals indicating changes in the user-editable properties
		void updateBackgroundColor();
		void updateFixedFrame();
		void updateRobotDescription();
		void updateTfPrefix();

	private:
        std::shared_ptr<rclcpp::Node> node_handle_{ nullptr };

        rviz_common::PanelDockWidget* frame_dock_{ nullptr };
        RobotModelViewerPanel* panel_{ nullptr };

		// user-editable property variables
		rviz_common::properties::ColorProperty* color_property_;
		rviz_common::properties::StringProperty* fixed_frame_property_;
		rviz_common::properties::StringProperty* robot_description_property_;
		rviz_common::properties::StringProperty* tf_prefix_property_;
	};
} // end namespace whi_rviz_plugins
