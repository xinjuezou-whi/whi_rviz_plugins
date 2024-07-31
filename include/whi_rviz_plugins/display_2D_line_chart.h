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
	class RosTopicProperty;
	class IntProperty;
	class FloatProperty;
	class ColorProperty;
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
		void updateDataTopic();
		void updateMaxDataLength();
		void updateGridDataSize();
		void updateGridMajorSize();
		void updateGridMinorSize();
		void updateDataColor();
		void updateMajorColor();
		void updateMinorColor();
		void updateCanvasColor();

	private:
        rviz::PanelDockWidget* frame_dock_{ nullptr };
        LineChart2DPanel* panel_{ nullptr };

		// user-editable property variables
		rviz::RosTopicProperty* data_topic_property_;
		rviz::IntProperty* max_data_length_property_;
		rviz::FloatProperty* data_size_property_;
		rviz::FloatProperty* grid_major_size_property_;
		rviz::FloatProperty* grid_minor_size_property_;
		rviz::ColorProperty* data_color_property_;
		rviz::ColorProperty* grid_major_color_property_;
		rviz::ColorProperty* grid_minor_color_property_;
		rviz::ColorProperty* canvas_color_property_;
	};
} // end namespace whi_rviz_plugins
