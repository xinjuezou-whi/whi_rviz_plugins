/******************************************************************
rviz plugin for motion status

Features:
- panel widget
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_rviz_plugins/panel_state.h"
#include "ui_navi_state.h"

#include <ros/package.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <angles/angles.h>
#include <boost/filesystem.hpp>
#include <iostream>

namespace whi_rviz_plugins
{
    StatePanel::StatePanel(QWidget* Parent/* = nullptr*/)
		: QWidget(Parent), ui_(new Ui::NaviState())
        , node_handle_(std::make_unique<ros::NodeHandle>())
	{
        first_state_msg_.header.seq = std::numeric_limits<uint32_t>::max();

		// set up the GUI
		ui_->setupUi(this);

		// WHI logo
		std::string pkgPath = getPackagePath();
		QImage logo;
		if (logo.load(QString(pkgPath.c_str()) + "/icons/classes/whi_logo.png"))
		{
			QImage scaled = logo.scaledToHeight(48);
			ui_->label_logo->setPixmap(QPixmap::fromImage(scaled));
		}
        // other properties
        ui_->label_goal->setText("none");
        ui_->label_eta->setText("no info");
        // indicator
        setIndicatorIcon(ui_->label_indicator_1, INDICATOR_GREY);
        setIndicatorIcon(ui_->label_indicator_2, INDICATOR_GREY);
        setIndicatorIcon(ui_->label_indicator_3, INDICATOR_GREY);
        setIndicatorIcon(ui_->label_indicator_4, INDICATOR_GREY);
        setIndicatorIcon(ui_->label_indicator_5, INDICATOR_GREY);
        setIndicatorIcon(ui_->label_indicator_6, INDICATOR_GREY);
        setIndicatorIcon(ui_->label_indicator_7, INDICATOR_GREY);
        setIndicatorIcon(ui_->label_indicator_8, INDICATOR_GREY);
        setIndicatorText(ui_->label_indicator_cap_1, "inactve");
        setIndicatorText(ui_->label_indicator_cap_2, "auto");
        setIndicatorText(ui_->label_indicator_cap_3, "task");
        setIndicatorText(ui_->label_indicator_cap_4, "reserved");
        setIndicatorText(ui_->label_indicator_cap_5, "reserved");
        setIndicatorText(ui_->label_indicator_cap_6, "reserved");
        setIndicatorText(ui_->label_indicator_cap_7, "reserved");
        setIndicatorText(ui_->label_indicator_cap_8, "reserved");
        // signals
		connect(ui_->pushButton_clear, &QPushButton::clicked, this, [=]() { clearButtonClicked(); });
    }

    StatePanel::~StatePanel()
	{
		delete ui_;
	}

    void StatePanel::setVelocities(double Linear, double Angular)
    {
        ui_->label_linear->setText(QString::number(Linear, 'f', 2));
        ui_->label_angular->setText(QString::number(Angular, 'f', 2));
    }

    void StatePanel::setGoal(const geometry_msgs::Pose& Goal)
    {
        tf2::Quaternion quaternion(Goal.orientation.x, Goal.orientation.y, Goal.orientation.z, Goal.orientation.w);
        double roll = 0.0, pitch = 0.0, yaw = 0.0;
  		tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

        QString goal;
        goal = "x: " + QString::number(Goal.position.x, 'f', 2) + ", y:" + QString::number(Goal.position.y, 'f', 2) +
            ", yaw: " + QString::number(angles::to_degrees(yaw), 'f', 2);
        ui_->label_goal->setText(goal);
    }

    void StatePanel::setEta(const std::string& Eta)
    {
        ui_->label_eta->setText(Eta.c_str());
    }

    void StatePanel::setMotionState(const whi_interfaces::WhiMotionState::ConstPtr& State)
    {
        if (State->header.seq < first_state_msg_.header.seq)
        {
            first_state_msg_.header.seq = State->header.seq;
            first_state_msg_.header.stamp = ros::Time::now();
        }
        std::time_t rawTime = static_cast<time_t>(first_state_msg_.header.stamp.toSec());
        struct tm* timeInfo = localtime(&rawTime);
        const int LEN = 64;
        char output[LEN];
        std::strftime(output, LEN, "%Y.%m.%d-%H:%M:%S", timeInfo);
        ui_->label_started->setText(output);
        double sec = (State->header.stamp.toSec() - first_state_msg_.header.stamp.toSec()) / 3600.0;
        if (sec > 0.0)
        {
            ui_->label_running_hours->setText(QString::number(sec, 'f', 2));
        }

        if (State->state == whi_interfaces::WhiMotionState::STA_STANDBY)
        {
            setIndicatorIcon(ui_->label_indicator_1, INDICATOR_GREEN);
            setIndicatorText(ui_->label_indicator_cap_1, "standby");
            setIndicatorIcon(ui_->label_indicator_3, INDICATOR_GREY);
            setIndicatorText(ui_->label_indicator_cap_3, "task");
        }
        else if (State->state == whi_interfaces::WhiMotionState::STA_RUNNING)
        {
            setIndicatorIcon(ui_->label_indicator_1, INDICATOR_YELLOW);
            setIndicatorText(ui_->label_indicator_cap_1, "running");
            setIndicatorIcon(ui_->label_indicator_3, INDICATOR_GREY);
            setIndicatorText(ui_->label_indicator_cap_3, "task");
        }
        else if (State->state == whi_interfaces::WhiMotionState::STA_OPERATING)
        {
            setIndicatorIcon(ui_->label_indicator_3, INDICATOR_YELLOW);
            setIndicatorText(ui_->label_indicator_cap_3, "operating");
        }
        else if (State->state == whi_interfaces::WhiMotionState::STA_FAULT)
        {
            setIndicatorIcon(ui_->label_indicator_1, INDICATOR_RED);
            setIndicatorText(ui_->label_indicator_cap_1, "fault");
        }
        else if (State->state == whi_interfaces::WhiMotionState::STA_ESTOP)
        {
            setIndicatorIcon(ui_->label_indicator_1, INDICATOR_ORANGE);
            setIndicatorText(ui_->label_indicator_cap_1, "E-Stop");
        }
        else if (State->state == whi_interfaces::WhiMotionState::STA_CRITICAL_COLLISION)
        {
            setIndicatorIcon(ui_->label_indicator_1, INDICATOR_ORANGE);
            setIndicatorText(ui_->label_indicator_cap_1, "collision");
        }
    }

    void StatePanel::setRcState(const whi_interfaces::WhiRcState::ConstPtr& State)
    {
        if (State->state == whi_interfaces::WhiRcState::STA_AUTO)
        {
            setIndicatorIcon(ui_->label_indicator_2, INDICATOR_GREEN);
            setIndicatorText(ui_->label_indicator_cap_2, "auto");
        }
        else if (State->state == whi_interfaces::WhiRcState::STA_REMOTE)
        {
            setIndicatorIcon(ui_->label_indicator_2, INDICATOR_BLUE);
            setIndicatorText(ui_->label_indicator_cap_2, "remote");
        }
    }

    void StatePanel::setArmState(const whi_interfaces::WhiMotionState::ConstPtr& State)
    {
        if (State->state == whi_interfaces::WhiMotionState::STA_BOOTING)
        {
            setIndicatorIcon(ui_->label_indicator_4, INDICATOR_YELLOW);
            setIndicatorText(ui_->label_indicator_cap_4, "arm booting");
        }
        else if (State->state == whi_interfaces::WhiMotionState::STA_STANDBY)
        {
            setIndicatorIcon(ui_->label_indicator_4, INDICATOR_GREEN);
            setIndicatorText(ui_->label_indicator_cap_4, "arm standby");
        }
        else if (State->state == whi_interfaces::WhiMotionState::STA_FAULT)
        {
            setIndicatorIcon(ui_->label_indicator_4, INDICATOR_RED);
            setIndicatorText(ui_->label_indicator_cap_4, "arm fault");
        }
    }

    void StatePanel::setRcStateTopic(const std::string& Topic)
    {
        pub_rc_state_ = std::make_unique<ros::Publisher>(
            node_handle_->advertise<whi_interfaces::WhiRcState>(Topic, 50));
    }

    void StatePanel::setBatteryInfo(int Soc, int Soh)
    {
        ui_->label_soc->setText(QString::number(Soc) + "%");
        ui_->label_soh->setText(QString::number(Soh) + "%");

        setBatteryIcon(ui_->label_battery, Soc);
    }

    std::string StatePanel::getPackagePath() const
    {
        boost::filesystem::path path(ros::package::getPath("whi_rviz_plugins"));
        return path.string();
    }

    void StatePanel::setIndicatorIcon(QLabel* Label, int Type)
    {
        std::string iconFile("/icons/classes/indicator_");
        switch (Type)
        {
        case INDICATOR_RED:
            iconFile += "red.png";
            break;
        case INDICATOR_ORANGE:
            iconFile += "orange.png";
            break;
        case INDICATOR_YELLOW:
            iconFile += "yellow.png";
            break;
        case INDICATOR_GREEN:
            iconFile += "green.png";
            break;
        case INDICATOR_BLUE:
            iconFile += "blue.png";
            break;
        default:
            iconFile += "grey.png";
            break;
        }

        setLabelIcon(Label, iconFile, 24);
    }

    void StatePanel::setIndicatorText(QLabel* Label, const std::string& Text)
    {
        Label->setText(Text.c_str());
    }

    void StatePanel::setBatteryIcon(QLabel* Label, int Soc)
    {
        std::string iconFile("/icons/classes/bat_");
        if (Soc > 80)
        {
            iconFile += "100.png";
        }
        else if (Soc > 60 && Soc <= 80)
        {
            iconFile += "80.png";
        }
        else if (Soc > 40 && Soc <= 60)
        {
            iconFile += "60.png";
        }
        else if (Soc > 20 && Soc <= 40)
        {
            iconFile += "40.png";
        }
        else
        {
            iconFile += "20.png";
        }

        setLabelIcon(Label, iconFile, 24);
    }

    void StatePanel::setLabelIcon(QLabel* Label, const std::string& IconFile, int Scale)
    {
        QImage indicator;
        if (indicator.load(QString(getPackagePath().c_str()) + IconFile.c_str()))
		{
            QImage scaled = indicator.scaledToHeight(Scale);
			Label->setPixmap(QPixmap::fromImage(scaled));
		}
    }

    void StatePanel::clearButtonClicked()
    {
        whi_interfaces::WhiRcState msgState;
        msgState.state = whi_interfaces::WhiRcState::STA_CLEAR_FAULT;
        pub_rc_state_->publish(msgState);
    }
} // end namespace whi_rviz_plugins
