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
2025-08-06：Migrate from ROS 1
2025-xx-xx: xxx
******************************************************************/
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/string.hpp>

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
		void subCallbackChargingState(const std_msgs::msg::String::SharedPtr Msg);

	private:
		Ui::NaviBattery* ui_{ nullptr };
		std::unique_ptr<rclcpp::Node> node_handle_{ nullptr };
		rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_charging_state_{ nullptr };
	};
} // end namespace whi_rviz_plugins
