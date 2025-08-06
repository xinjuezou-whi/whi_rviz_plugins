/******************************************************************
rviz plugin for motion status

Features:
- panel widget
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2023-06-04: Initial version
2025-08-04: Migrate from ROS 1
2025-xx-xx: xxx
******************************************************************/
#pragma once
#include <whi_interfaces/msg/whi_motion_state.hpp>
#include <whi_interfaces/msg/whi_rc_state.hpp>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/bool.hpp>
#include "rviz_common/panel.hpp"

namespace Ui
{
class NaviState;
}

class QLabel;

namespace whi_rviz_plugins
{
	class StatePanel : public QWidget
	{
		Q_OBJECT
	public:
		StatePanel(std::shared_ptr<rclcpp::Node> NodeHandle, QWidget* Parent = nullptr);
		~StatePanel() override;

	public:
        void setVelocities(double Linear, double Angular);
        void setGoal(const geometry_msgs::msg::Pose& Goal);
        void setEta(const std::string& Eta);
		void setMotionState(const whi_interfaces::msg::WhiMotionState::SharedPtr State);
		void setRcState(const whi_interfaces::msg::WhiRcState::SharedPtr State);
		void setArmState(const whi_interfaces::msg::WhiMotionState::SharedPtr State);
		void setImuState();
		void setRcStateTopic(const std::string& Topic);
		void setEstopTopic(const std::string& Topic);
		void setBatteryInfo(int Soc, int Soh);
		void setTempHum(double Temperature, double Humidity);

	private:
        void setIndicatorIcon(QLabel* Label, int Type);
		void setIndicatorText(QLabel* Label, const std::string& Text);
		void setTempHumVisibility(bool Visibale);
		void setBatteryIcon(QLabel* Label, int Soc);
		void setLabelIcon(QLabel* Label, const std::string& IconFile, int Scale);
		void clearButtonClicked();
		void resetImuButtonClicked();
		void resetRcButtonClicked();
		void resetRgbdButtonClicked();
		void estopButtonToggled(bool Checked);
		void setEstopIcon(bool Checked);
		void update();
		std::string getPackagePath() const;

	private:
        enum IndicatorType { INDICATOR_GREY = 0, INDICATOR_RED, INDICATOR_ORANGE,
			INDICATOR_YELLOW, INDICATOR_GREEN, INDICATOR_BLUE };
		Ui::NaviState* ui_{ nullptr };
		std::unique_ptr<whi_interfaces::msg::WhiMotionState> first_state_msg_{ nullptr };
		std::shared_ptr<rclcpp::Node> node_handle_{ nullptr };
		rclcpp::Publisher<whi_interfaces::msg::WhiRcState>::SharedPtr pub_rc_state_{ nullptr };
		rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_estop_{ nullptr };
		rclcpp::TimerBase::SharedPtr non_realtime_loop_{ nullptr };
		rclcpp::Time last_updated_imu_;
		rclcpp::Time last_updated_rc_;
		std::unique_ptr<rclcpp::Time> last_updated_arm_{ nullptr };
		int estop_init_height_{ 50 };
	};
} // end namespace whi_rviz_plugins
