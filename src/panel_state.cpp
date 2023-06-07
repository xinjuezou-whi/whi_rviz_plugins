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
#include <whi_interfaces/WhiMotionInterface.h>
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
	{
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
        setIndicatorText(ui_->label_indicator_cap_2, "reserved");
        setIndicatorText(ui_->label_indicator_cap_3, "reserved");
        setIndicatorText(ui_->label_indicator_cap_4, "reserved");
        setIndicatorText(ui_->label_indicator_cap_5, "reserved");
        setIndicatorText(ui_->label_indicator_cap_6, "reserved");
        setIndicatorText(ui_->label_indicator_cap_7, "reserved");
        setIndicatorText(ui_->label_indicator_cap_8, "reserved");
    }

    StatePanel::~StatePanel()
	{
		delete ui_;
	}

    void StatePanel::setVelocities(double Linear, double Angular)
    {
        ui_->label_linear->setText(QString::number(Linear, 'g', 2));
        ui_->label_angular->setText(QString::number(Angular, 'g', 2));
    }

    void StatePanel::setGoal(const geometry_msgs::Pose& Goal)
    {
        tf2::Quaternion quaternion(Goal.orientation.x, Goal.orientation.y, Goal.orientation.z, Goal.orientation.w);
        double roll = 0.0, pitch = 0.0, yaw = 0.0;
  		tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

        QString goal;
        goal = "x: " + QString::number(Goal.position.x, 'g', 2) + ", y:" + QString::number(Goal.position.y, 'g', 2) +
            ", yaw: " + QString::number(angles::to_degrees(yaw), 'g', 2);
        ui_->label_goal->setText(goal);
    }

    void StatePanel::setEta(const std::string& Eta)
    {
        ui_->label_eta->setText(Eta.c_str());
    }

    void StatePanel::setMotionInterface(int State)
    {
        if (State == whi_interfaces::WhiMotionInterface::STA_STANDBY)
        {
            setIndicatorIcon(ui_->label_indicator_1, INDICATOR_GREEN);
            setIndicatorText(ui_->label_indicator_cap_1, "standby");
        }
        else if (State == whi_interfaces::WhiMotionInterface::STA_FAULT)
        {
            setIndicatorIcon(ui_->label_indicator_1, INDICATOR_ORANGE);
            setIndicatorText(ui_->label_indicator_cap_1, "fault");
        }
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
        case INDICATOR_ORANGE:
            iconFile += "orange.png";
            break;
        case INDICATOR_YELLOW:
            iconFile += "yellow.png";
            break;
        case INDICATOR_GREEN:
            iconFile += "green.png";
            break;
        default:
            iconFile += "grey.png";
            break;
        }

        QImage indicator;
        if (indicator.load(QString(getPackagePath().c_str()) + iconFile.c_str()))
		{
            QImage scaled = indicator.scaledToHeight(24);
			Label->setPixmap(QPixmap::fromImage(scaled));
		}
    }

    void StatePanel::setIndicatorText(QLabel* Label, const std::string& Text)
    {
        Label->setText(Text.c_str());
    }
} // end namespace whi_rviz_plugins
