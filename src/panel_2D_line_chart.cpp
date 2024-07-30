/******************************************************************
rviz plugin for 2D line chart panel

Features:
- 2D line
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_rviz_plugins/panel_2D_line_chart.h"
#include "ui_line_chart_2D.h"

#include <iostream>
#include <sstream>
#include <math.h>
#include <boost/filesystem.hpp>
#include <ros/package.h>

#include <qwt_plot_curve.h>
#include <qwt_plot_grid.h>
#include <QVector>
#include <QBrush>

namespace whi_rviz_plugins
{
    double TimeData::get() const
    {
        return value_;
    }

    double TimeData::normalTime() const
    {
        return double(raw_time_ - base_time_) / 3600.0;
    }

    std::time_t TimeData::rawTime() const
    {
        return raw_time_;
    }

    void TimeData::setBaseTime(double Sec)
    {
        base_time_ = static_cast<time_t>(Sec);
    }

    std::string TimeData::unit() const
    {
        return unit_;
    }

    std::string TimeData::formatStr(const std::string& Format, size_t Length) const
    {
        struct tm* timeInfo = localtime(&raw_time_);
        char output[Length];
        std::strftime(output, Length, Format.c_str(), timeInfo);

        return output;
    }

    void TimeSeriesData::add(const std::string& Key, double Value, const std::string& Unit, double Sec/* = ros::Time::now().toSec()*/)
    {
        if (auto search = series_map_.find(Key); search == series_map_.end())
        {
            series_map_[Key] = std::vector<TimeData>();
        }

        if (base_time_ < 1e-5)
        {
            base_time_ = Sec;
        }

        series_map_[Key].push_back(TimeData(Value, Unit, Sec, base_time_));
    }
	
    std::vector<TimeData> TimeSeriesData::get(const std::string& Key) const
    {
        if (auto search = series_map_.find(Key); search != series_map_.end())
        {
            return search->second;
        }
        else
        {
            return std::vector<TimeData>();
        }
    }

    void TimeSeriesData::setBaseTime(double Sec)
    {
        std::time_t base = static_cast<std::time_t>(Sec);
        for (auto& item : series_map_)
        {
            for (std::vector<TimeData>::iterator it = item.second.begin(); it != item.second.end(); )
            {
                if (it->rawTime() - base < 0)
                {
                    it = item.second.erase(it);
                }
                else
                {
                    it->setBaseTime(Sec);
                    ++it;
                }
            }
        }
    }

    std::vector<std::string> TimeSeriesData::dataNames() const
    {
        std::vector<std::string> names;
        for (const auto& it : series_map_)
        {
            names.push_back(it.first);
        }

        return names;
    }


    LineChart2DPanel::LineChart2DPanel(QWidget* Parent/* = nullptr*/)
		: QWidget(Parent), ui_(new Ui::LineChart2D())
        , node_handle_(std::make_unique<ros::NodeHandle>())
	{
		// set up the GUI
		ui_->setupUi(this);
        ui_->qwtPlot->setCanvasBackground(QBrush(QColor(38, 64, 115)));
        curve_ = new QwtPlotCurve("curve");
        QwtPlotGrid* grid = new QwtPlotGrid();
        grid->enableXMin(true);
        grid->enableYMin(true);
        grid->setMajorPen(QPen(Qt::cyan, 0.5, Qt::DotLine));
        grid->setMinorPen(QPen(Qt::gray, 0.2, Qt::DotLine));
        grid->attach(ui_->qwtPlot);
        // a widget normally must setFocusPolicy() to something other than Qt::NoFocus in order to receive focus events
        setFocusPolicy(Qt::StrongFocus);

		// WHI logo
		boost::filesystem::path path(ros::package::getPath("whi_rviz_plugins"));
		QImage logo;
		if (logo.load(QString(path.string().c_str()) + "/icons/classes/whi_logo.png"))
		{
			QImage scaled = logo.scaledToHeight(48);
			ui_->label_logo->setPixmap(QPixmap::fromImage(scaled));
		}

        // signals
        connect(ui_->pushButton_reset, &QPushButton::clicked, this, [=]() { resetButtonClicked(); });
		connect(ui_->comboBox_channel, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [=](int Index)
		{
            // update its data names
            auto channel = ui_->comboBox_channel->currentText().toStdString();
            ui_->comboBox_item->clear();
            for (const auto& name : chart_map_[channel].dataNames())
            {
                ui_->comboBox_item->addItem(name.c_str());
            }

            // then plot
            plot();
		});
		connect(ui_->comboBox_item, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [=](int Index)
		{
            plot();
		});
    }

    LineChart2DPanel::~LineChart2DPanel()
	{
		delete ui_;
	}

    void LineChart2DPanel::setDataTopic(const std::string& Topic)
    {
		if (!Topic.empty())
		{
        	sub_data_ = std::make_unique<ros::Subscriber>(
            	node_handle_->subscribe<whi_interfaces::WhiLineChart2D>(Topic, 10,
            	std::bind(&LineChart2DPanel::subCallbackData, this, std::placeholders::_1)));
		}
    }

    void LineChart2DPanel::resetButtonClicked()
    {
        auto channel = ui_->comboBox_channel->currentText().toStdString();
        chart_map_[channel].setBaseTime(ros::Time::now().toSec());

        plot();
    }
    
    void LineChart2DPanel::subCallbackData(const whi_interfaces::WhiLineChart2D::ConstPtr& Data)
    {
        // check if the channel changes
        bool chChanged = false;
        if (ui_->comboBox_channel->count() != Data->array.size())
        {
            chChanged = true;
        }
        else
        {
            for (int i = 0; i < ui_->comboBox_channel->count(); ++i)
            {
                if (ui_->comboBox_channel->itemText(i).toStdString() != Data->array[i].name)
                {
                    chChanged = true;
                    break;
                }
            }
        }

        if (chChanged)
        {
            ui_->comboBox_channel->blockSignals(true);
            ui_->comboBox_channel->clear();
        }
        for (int i = 0; i < Data->array.size(); ++i)
        {
            if (chChanged)
            {
                ui_->comboBox_channel->addItem(Data->array[i].name.c_str());
            }

            if (auto search = chart_map_.find(Data->array[i].name); search == chart_map_.end())
            {
                chart_map_[Data->array[i].name] = TimeSeriesData();
            }
            for (int j = 0; j < std::min(Data->array[i].data.size(), Data->array[i].items_name.size()); ++j)
            {
                chart_map_[Data->array[i].name].add(Data->array[i].items_name[j],
                    Data->array[i].data[j], Data->array[i].items_unit[j], Data->header.stamp.toSec());
            }
        }

        int index = ui_->comboBox_channel->currentIndex();
        bool namesChanged = false;
        if (chChanged)
        {
            // visualize the first one
            ui_->comboBox_channel->setCurrentIndex(0);
            ui_->comboBox_channel->blockSignals(false);
            namesChanged = true;
        }
        else
        {
            // check if the data names change
            if (ui_->comboBox_item->count() != Data->array[index].items_name.size())
            {
                namesChanged = true;
            }
            else
            {
                auto channel = ui_->comboBox_channel->currentText().toStdString();
                auto names = chart_map_[channel].dataNames();
                for (int i = 0; i < ui_->comboBox_item->count(); ++i)
                {
                    if (ui_->comboBox_item->itemText(i).toStdString() != names[i])
                    {
                        namesChanged = true;
                        break;
                    }
                }
            }
        }

        if (namesChanged)
        {
            ui_->comboBox_item->blockSignals(true);
            ui_->comboBox_item->clear();
            auto channel = ui_->comboBox_channel->currentText().toStdString();
            for (const auto& name : chart_map_[channel].dataNames())
            {
                ui_->comboBox_item->addItem(name.c_str());
            }
            ui_->comboBox_item->blockSignals(false);
        }

        plot();
    }

    void LineChart2DPanel::plot()
    {
        auto channel = ui_->comboBox_channel->currentText().toStdString();
        auto dataName = ui_->comboBox_item->currentText().toStdString();
        auto raw = chart_map_[channel].get(dataName);
        auto unit = raw.empty() ? "N/A" : raw.front().unit();
        plot(raw, unit);
    }

    void LineChart2DPanel::plot(const std::vector<TimeData>& Data, const std::string& Unit)
    {
        curve_->detach();

        QVector<double> xData, yData;
        for (const auto& it : Data)
        {
            xData.append(it.normalTime());
            yData.append(it.get());
        }

        curve_->setPen(Qt::yellow, 1.0);
        curve_->setSamples(xData, yData);
        curve_->attach(ui_->qwtPlot);
        ui_->qwtPlot->setAxisTitle(QwtPlot::yLeft, Unit.c_str());
        ui_->qwtPlot->setAxisTitle(QwtPlot::xBottom, "second");
        ui_->qwtPlot->replot();
    }
} // end namespace whi_rviz_plugins
