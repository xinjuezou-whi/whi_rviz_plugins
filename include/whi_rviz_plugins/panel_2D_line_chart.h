/******************************************************************
rviz plugin for 2D line chart panel

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
#include <ros/ros.h>
#include <rviz/panel.h>
#include <memory>
#include <thread>

#include <whi_interfaces/WhiLineChart2D.h>

namespace Ui
{
class LineChart2D;
}

// forward declaration
class QwtPlotCurve;
class QwtPlotGrid;

namespace whi_rviz_plugins
{
	class TimeData
	{
	public:
		TimeData() = default;
		TimeData(double Value, const std::string& Unit,
			double Sec = ros::Time::now().toSec(), double BaseTime = ros::Time::now().toSec())
			: value_(Value), unit_(Unit)
			, raw_time_(static_cast<std::time_t>(Sec)), base_time_(static_cast<std::time_t>(BaseTime)) {};
		~TimeData() = default;

	public:
		double get() const;
		double normalTime() const;
		std::time_t rawTime() const;
		void setBaseTime(double Sec);
		std::string unit() const;
		std::string formatStr(const std::string& Format, size_t Length) const;
	
	protected:
		double value_{ 0.0 };
		std::string unit_;
		std::time_t raw_time_;
		std::time_t base_time_;
	};

	class TimeSeriesData
	{
	public:
		TimeSeriesData(int MaxLength = 100) : max_length_(MaxLength) {};
		~TimeSeriesData() = default;

	public:
		void add(const std::string& Key, double Value, const std::string& Unit, double Sec = ros::Time::now().toSec());
		std::vector<TimeData> get(const std::string& Key) const;
		void setBaseTime(double Sec);
		std::vector<std::string> dataNames() const;
		void setMaxLength(int MaxLength);
	
	protected:
		std::map<std::string, std::vector<TimeData>> series_map_;
		double base_time_{ 0.0 };
		int max_length_{ 100 };
	};

	class LineChart2DPanel : public QWidget
	{
		Q_OBJECT
	public:
		LineChart2DPanel(QWidget* Parent = nullptr);
		~LineChart2DPanel() override;

	public:
		void setDataTopic(const std::string& Topic);
		void setMaxDataLength(int Length);
		void setGridDataSize(double Size);
		void setGridMajorSize(double Size);
		void setGridMinorSize(double Size);
		void setGridDataColor(const QColor& Color);
		void setGridMajorColor(const QColor& Color);
		void setGridMinorColor(const QColor& Color);
		void setGridCanvasColor(const QColor& Color);

	private:
		void resetButtonClicked();
		void subCallbackData(const whi_interfaces::WhiLineChart2D::ConstPtr& Data);
		void plot();
		void plot(const std::vector<TimeData>& Data, const std::string& Unit);

	private:
		Ui::LineChart2D* ui_{ nullptr };
		QwtPlotCurve* curve_{ nullptr };
		QwtPlotGrid* grid_{ nullptr };
		std::unique_ptr<ros::NodeHandle> node_handle_{ nullptr };
		std::unique_ptr<ros::Subscriber> sub_data_{ nullptr };
		std::map<std::string, TimeSeriesData> chart_map_;
		int max_data_length_{ 100 };
		double grid_data_size_{ 2.0 };
		double grid_major_size_{ 0.5 };
		double grid_minor_size_{ 0.2 };
		QColor color_data_{ Qt::yellow };
		QColor color_major_{ Qt::cyan };
		QColor color_minor_{ Qt::gray };
		QColor color_canvas_{ QColor(38, 64, 115) };
	};
} // end namespace whi_rviz_plugins
