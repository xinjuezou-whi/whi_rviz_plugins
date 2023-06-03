/******************************************************************
rviz plugin for battery info

Features:
- SOC
- state of charging

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2023-06-03: Initial version
2022-xx-xx: xxx
******************************************************************/
#pragma once
#include <ros/ros.h>
#include <rviz/panel.h>
#include <std_msgs/String.h>
#include <memory>

namespace Ui
{
class NaviBattery;
}

namespace whi_rviz_plugins
{
	class BatteryPanel : public QWidget
	{
		Q_OBJECT
	public:
		BatteryPanel(QWidget* Parent = nullptr);
		~BatteryPanel() override;

	public:
        void setSoc(int Soc);
        void setChargingStateTopic(const std::string& Topic);

	private:
        std::string getPackagePath() const;
        void setBatteryIcon(int Soc);
        void subCallbackChargingState(const std_msgs::String::ConstPtr& Msg);

	private:
		Ui::NaviBattery* ui_{ nullptr };
		std::unique_ptr<ros::NodeHandle> node_handle_{ nullptr };
		std::unique_ptr<ros::Subscriber> sub_charging_state_{ nullptr };
	};
} // end namespace whi_rviz_plugins
