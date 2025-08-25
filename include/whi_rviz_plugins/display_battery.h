/******************************************************************
rviz plugin for showing battery infomation

Features:
- power
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2022-07-27: Initial version
2025-08-06: Migrate from ROS 1
2025-xx-xx: xxx
******************************************************************/
#pragma once
#ifndef Q_MOC_RUN
#include <boost/circular_buffer.hpp>

#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/panel_dock_widget.hpp>

#include "panel_battery.h"
#include "whi_interfaces/msg/whi_battery.hpp"
#endif

namespace Ogre
{
	class SceneNode;
	class ColourValue;
}

namespace rviz_common
{
	namespace properties
	{
		class ColorProperty;
		class FloatProperty;
		class IntProperty;
		class VectorProperty;
		class BoolProperty;
		class RosTopicProperty;
	}
}

namespace whi_rviz_plugins
{
	class BatteryVisual;

	// declare a new subclass of rviz_common::Display
	// every display which can be listed in the "Displays" panel is a subclass of rviz_common::Display
	//
	// DisplayBat will show a movable_text showing the power info of battery,
	// it will also optionally show a history of recent info vectors, which will be stored in a circular buffer
	//
	// the DisplayBat class itself just implements the circular buffer, editable parameters, and Display subclass machinery
	// the visuals themselves are represented by a separate class, BatteryVisual
	// the idiom for the visuals is that when the objects exist, they appear in the scene,
	// and when they are deleted, they disappear
	class DisplayBat : public rviz_common::MessageFilterDisplay<whi_interfaces::msg::WhiBattery>
	{
		Q_OBJECT
	public:
		// pluginlib::ClassLoader creates instances by calling the default constructor,
		// so make sure you have one
		DisplayBat();
		virtual ~DisplayBat();

		// overrides of protected virtual functions from Display as much as possible,
		// when Displays are not enabled, they should not be subscribed to incoming data,
		// and should not show anything in the 3D view
		// these functions are where these connections are made and broken
	protected:
		virtual void onInitialize();
		// a helper function to clear this display back to the initial state
		virtual void reset();

	private Q_SLOTS:
		// these Qt slots get connected to signals indicating changes in the user-editable properties
		void updateColorAndAlpha();
		void updateHistoryLength();
		void updateSize();
		void updateOffsets();
		void updateOrientation();
		void updateShowPanel();

	private:
		// function to handle an incoming ROS message
		void processMessage(const whi_interfaces::msg::WhiBattery::ConstSharedPtr Msg);

	private:
		rclcpp::Node::SharedPtr node_handle_{ nullptr };

		// storage for the list of visuals. It is a circular buffer,
		// where data gets popped from the front (oldest) and pushed to the back (newest)
		boost::circular_buffer<std::shared_ptr<BatteryVisual>> visuals_;

		// user-editable property variables
		rviz_common::properties::ColorProperty* color_property_;
		rviz_common::properties::FloatProperty* alpha_property_;
		rviz_common::properties::IntProperty* history_length_property_;
		rviz_common::properties::FloatProperty* size_property_;
		rviz_common::properties::VectorProperty* offsets_property_;
		rviz_common::properties::VectorProperty* orientation_property_;
		// other properties
		std::shared_ptr<Ogre::ColourValue> color_red_{ nullptr };
		// panel
		rviz_common::properties::BoolProperty* show_panel_property_;
		rviz_common::properties::RosTopicProperty* charging_state_topic_property_;

		// dock and panel
		rviz_common::PanelDockWidget* frame_dock_{ nullptr };
        BatteryPanel* panel_{ nullptr };
	};
} // end namespace whi_rviz_plugins
