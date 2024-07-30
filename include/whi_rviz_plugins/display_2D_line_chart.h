/******************************************************************
rviz plugin for 2D line chart

Features:
- 2D line
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2024-07-29: Initial version
2024-xx-xx: xxx
******************************************************************/
#pragma once
#include <rviz/display.h>
#include <rviz/panel_dock_widget.h>

#include "panel_2D_line_chart.h"

// forward declaration
namespace rviz
{
	class BoolProperty;
    class FloatProperty;
    class StringProperty;
	class RosTopicProperty;
}

namespace whi_rviz_plugins
{
	// declare a new subclass of rviz::Display
	// every display which can be listed in the "Displays" panel is a subclass of rviz::Display
	class Display2DLineChart : public rviz::Display 
	{
		Q_OBJECT
	public:
		// pluginlib::ClassLoader creates instances by calling the default constructor,
		// so make sure you have one
		Display2DLineChart();
		virtual ~Display2DLineChart();

		// overrides of protected virtual functions from Display as much as possible,
		// when Displays are not enabled, they should not be subscribed to incoming data,
		// and should not show anything in the 3D view
		// these functions are where these connections are made and broken
	protected:
		virtual void onInitialize();

	private Q_SLOTS:
		// these Qt slots get connected to signals indicating changes in the user-editable properties
		// void updateEnable();
		// void updatePubFrequency();
		// void updatePubTopic();
		// void updateLinearMin();
		// void updateLinearMax();
		// void updateLinearStep();
		// void updateAngularMin();
		// void updateAngularMax();
		// void updateAngularStep();
		// void updateMotionStateTopic();
		void updateDataTopic();

	private:
        rviz::PanelDockWidget* frame_dock_{ nullptr };
        LineChart2DPanel* panel_{ nullptr };

		// user-editable property variables
		// rviz::BoolProperty* enable_property_;
		// rviz::FloatProperty* pub_frequency_property_;
		// rviz::StringProperty* pub_topic_property_;
		// rviz::FloatProperty* linear_min_;
		// rviz::FloatProperty* linear_max_;
		// rviz::FloatProperty* linear_step_;
		// rviz::FloatProperty* angular_min_;
		// rviz::FloatProperty* angular_max_;
		// rviz::FloatProperty* angular_step_;
		// rviz::RosTopicProperty* motion_state_topic_property_;
		rviz::RosTopicProperty* data_topic_property_;
	};
} // end namespace whi_rviz_plugins
