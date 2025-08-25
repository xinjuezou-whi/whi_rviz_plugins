/******************************************************************
rviz plugin for generic teleop

Features:
- teleop with twist message
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2022-11-18: Initial version
2025-08-01: Migrate from ROS 1
2025-xx-xx: xxx
******************************************************************/
#pragma once
#include "widget_twist.h"
#include <whi_interfaces/msg/whi_motion_state.hpp>
#include <whi_interfaces/msg/whi_rc_state.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <memory>
#include <thread>

namespace Ui
{
class NaviTeleop;
}

namespace whi_rviz_plugins
{
    class TwistWidget;

	class TeleopPanel : public QWidget
	{
		Q_OBJECT
	public:
		TeleopPanel(std::shared_ptr<rclcpp::Node> Node, QWidget* Parent = nullptr);
		~TeleopPanel() override;

	public:
		void setLinearMin(float Min);
		void setLinearMax(float Max);
		void setLinearStep(float Step);
		void setAngularMin(float Min);
		void setAngularMax(float Max);
		void setAngularStep(float Step);
		void setPubTopic(const std::string& Topic);
		void setPubFunctionality(bool Active);
		void setPubFrequency(float Frequency);
		void setUseStampedVel(bool Use);
		void moveLinear(int Dir);
		void moveAngular(int Dir);
		void halt();
		void setMotionStateTopic(const std::string& Topic);
		void setSwEstopTopic(const std::string& Topic);
		void setRcStateTopic(const std::string& Topic);

	private:
		void keyPressEvent(QKeyEvent* Event) override;
		void focusOutEvent(QFocusEvent* Event) override;
		void focusInEvent(QFocusEvent* Event) override;

		void subCallbackMotionState(const whi_interfaces::msg::WhiMotionState::SharedPtr Msg);
		void subCallbackSwEstop(const std_msgs::msg::Bool::SharedPtr Msg);
		void subCallbackRcState(const whi_interfaces::msg::WhiRcState::SharedPtr Msg);
		bool isBypassed();
		void refreshTopic();

	private:
		Ui::NaviTeleop* ui_{ nullptr };
        TwistWidget* twist_widget_{ nullptr };
		QTimer* timer_toggle_{ nullptr };
		int interval_toggle_{ 500 };
		int interval_pub_{ 200 };
		bool toggle_publishing_{ true };
		std::shared_ptr<rclcpp::Node> node_handle_{ nullptr };
		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_twist_unstamped_{ nullptr };
		using Twist = geometry_msgs::msg::TwistStamped;
		rclcpp::Publisher<Twist>::SharedPtr pub_twist_{ nullptr };
		std::string topic_;
		float linear_{ 0.0 };
		float angular_{ 0.0 };
		rclcpp::Subscription<whi_interfaces::msg::WhiMotionState>::SharedPtr sub_motion_state_{ nullptr };
		rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_sw_estop_{ nullptr };
		rclcpp::Subscription<whi_interfaces::msg::WhiRcState>::SharedPtr sub_rc_state_{ nullptr };
		bool sw_estopped_{ false };
		bool use_stamped_vel_{ true };
		std::atomic_bool toggle_estop_{ false };
		std::atomic_bool toggle_collision_{ false };
		std::atomic_bool remote_mode_{ false };
		std::atomic_bool activated_{ true };
		std::atomic_bool terminated_{ false };
		std::thread th_publish_;
	};
} // end namespace whi_rviz_plugins
