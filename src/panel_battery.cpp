/******************************************************************
rviz plugin for battery info

Features:
- SOC
- state of charging

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_rviz_plugins/panel_battery.h"
#include "ui_navi_battery.h"

#include <ros/package.h>
#include <boost/filesystem.hpp>
#include <ctime>

namespace whi_rviz_plugins
{
    BatteryPanel::BatteryPanel(QWidget* Parent/* = nullptr*/)
		: QWidget(Parent), ui_(new Ui::NaviBattery())
        , node_handle_(std::make_unique<ros::NodeHandle>())
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
        // default properties
        ui_->label_soc->setText("N/A");
        ui_->label_charging->setText("unavailable");
        ui_->label_last_time->setText("no charging yet");
        setBatteryIcon(100);
    }

    BatteryPanel::~BatteryPanel()
	{
		delete ui_;
	}

    void BatteryPanel::setSoc(int Soc)
    {
        setBatteryIcon(Soc);
        ui_->label_soc->setText(QString::number(Soc) + "%");
    }

    void BatteryPanel::setChargingStateTopic(const std::string& Topic)
    {
        sub_charging_state_ = std::make_unique<ros::Subscriber>(node_handle_->subscribe<std_msgs::String>(
		    Topic, 10, std::bind(&BatteryPanel::subCallbackChargingState, this, std::placeholders::_1)));
    }

    std::string BatteryPanel::getPackagePath() const
    {
        boost::filesystem::path path(ros::package::getPath("whi_rviz_plugins"));
        return path.string();
    }

    void BatteryPanel::setBatteryIcon(int Soc)
    {
        std::string iconFile;
        if (Soc > 80)
        {
            iconFile = "/icons/classes/bat_100.png";
        }
        else if (Soc > 60 && Soc <= 80)
        {
            iconFile = "/icons/classes/bat_80.png";
        }
        else if (Soc > 40 && Soc <= 60)
        {
            iconFile = "/icons/classes/bat_60.png";
        }
        else if (Soc > 20 && Soc <= 40)
        {
            iconFile = "/icons/classes/bat_40.png";
        }
        else
        {
            iconFile = "/icons/classes/bat_20.png";
        }

        QImage bat;
        if (bat.load(QString(getPackagePath().c_str()) + iconFile.c_str()))
		{
            QImage scaled = bat.scaledToHeight(24);
			ui_->label_battery->setPixmap(QPixmap::fromImage(scaled));
		}
    }

    void BatteryPanel::subCallbackChargingState(const std_msgs::String::ConstPtr& Msg)
    {
        ui_->label_charging->setText(Msg->data.c_str());
        if (Msg->data.find("good for task") != std::string::npos ||
            Msg->data.find("full") != std::string::npos)
        {
            std::time_t time = std::time(nullptr);
            char timeString[std::size("yyyy-mm-ddThh:mm:ssZ")];
            std::strftime(std::data(timeString), std::size(timeString), "%FT%TZ", std::localtime(&time));
            ui_->label_last_time->setText(timeString);
        }
    }
} // end namespace whi_rviz_plugins
