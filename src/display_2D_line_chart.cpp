/******************************************************************
rviz plugin for 2D line chart

Features:
- 2D line
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_rviz_plugins/display_2D_line_chart.h"

#include <pluginlib/class_list_macros.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/window_manager_interface.h>
#include <rviz/display_context.h>

namespace whi_rviz_plugins
{
    Display2DLineChart::Display2DLineChart()
        : Display()
    {
        std::cout << "\nWHI RViz plugin for 2D line chart VERSION 00.2.1" << std::endl;
        std::cout << "Copyright @ 2024-2025 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n" << std::endl;

        data_topic_property_ = new rviz::RosTopicProperty("Data topic", "line_data_2D",
            "whi_interfaces/WhiLineChart2D", "Topic of 2D data",
            this, SLOT(updateDataTopic()));
        max_data_length_property_ = new rviz::IntProperty("Max data length", 100, "Maxium data length that recoreded for plotting",
            this, SLOT(updateMaxDataLength()));
        max_data_length_property_->setMin(1);
        data_size_property_ = new rviz::FloatProperty("Data size", 2.0, "Data grid size",
            this, SLOT(updateGridDataSize()));
        data_color_property_ = new rviz::ColorProperty("Data grid color", Qt::yellow, "Data grid color.",
            this, SLOT(updateDataColor()));
        grid_major_size_property_ = new rviz::FloatProperty("Major grid size", 0.5, "Major grid size",
            this, SLOT(updateGridMajorSize()));
        grid_major_color_property_ = new rviz::ColorProperty("Major grid color", Qt::cyan, "Major grid color.",
            this, SLOT(updateMajorColor()));
        grid_minor_size_property_ = new rviz::FloatProperty("Minor grid size", 0.2, "Minor grid size",
            this, SLOT(updateGridMinorSize()));
        grid_minor_color_property_ = new rviz::ColorProperty("Minor grid color", Qt::gray, "Minor grid color.",
            this, SLOT(updateMinorColor()));
        canvas_color_property_ = new rviz::ColorProperty("Canvas color", QColor(38, 64, 115), "Canvas grid color.",
            this, SLOT(updateCanvasColor()));
    }

    Display2DLineChart::~Display2DLineChart()
    {
        delete frame_dock_;
    }
    
    void Display2DLineChart::onInitialize()
    {
        Display::onInitialize();

        panel_ = new LineChart2DPanel();
        rviz::WindowManagerInterface* windowContext = context_->getWindowManager();
        if (windowContext)
        {
            frame_dock_ = windowContext->addPane("2DLineChart", panel_); // getName() return "" ???
            frame_dock_->setIcon(getIcon()); // set the image name as same as the name of plugin
        }

        updateDataTopic();
        updateMaxDataLength();
		updateGridDataSize();
		updateGridMajorSize();
		updateGridMinorSize();
		updateDataColor();
		updateMajorColor();
		updateMinorColor();
		updateCanvasColor();
    }

    void Display2DLineChart::updateDataTopic()
    {
        panel_->setDataTopic(data_topic_property_->getTopicStd());
    }

    void Display2DLineChart::updateMaxDataLength()
    {
        panel_->setMaxDataLength(max_data_length_property_->getInt());
    }

    void Display2DLineChart::updateGridDataSize()
    {
        panel_->setGridDataSize(data_size_property_->getFloat());
    }

    void Display2DLineChart::updateGridMajorSize()
    {
        panel_->setGridMajorSize(grid_major_size_property_->getFloat());
    }

    void Display2DLineChart::updateGridMinorSize()
    {
        panel_->setGridMinorSize(grid_minor_size_property_->getFloat());
    }

    void Display2DLineChart::updateDataColor()
    {
        panel_->setGridDataColor(data_color_property_->getColor());
    }

    void Display2DLineChart::updateMajorColor()
    {
        panel_->setGridMajorColor(grid_major_color_property_->getColor());
    }

    void Display2DLineChart::updateMinorColor()
    {
        panel_->setGridMinorColor(grid_minor_color_property_->getColor());
    }

    void Display2DLineChart::updateCanvasColor()
    {
        panel_->setGridCanvasColor(canvas_color_property_->getColor());
    }

    PLUGINLIB_EXPORT_CLASS(whi_rviz_plugins::Display2DLineChart, rviz::Display)
} // end namespace whi_rviz_plugins
