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
2022-xx-xx: xxx
******************************************************************/
#pragma once
#include <ros/ros.h>
#include <rviz/panel.h>
#include <std_msgs/Bool.h>
#include <whi_interfaces/WhiMotionState.h>
#include <whi_interfaces/WhiRcState.h>

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
		TeleopPanel(QWidget* Parent = nullptr);
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

		void subCallbackMotionState(const whi_interfaces::WhiMotionState::ConstPtr& MotionState);
		void subCallbackSwEstop(const std_msgs::Bool::ConstPtr& Msg);
		void subCallbackRcState(const whi_interfaces::WhiRcState::ConstPtr& RcState);
		bool isBypassed();

	private:
		Ui::NaviTeleop* ui_{ nullptr };
        TwistWidget* twist_widget_{ nullptr };
		QTimer* timer_toggle_{ nullptr };
		int interval_toggle_{ 500 };
		int interval_pub_{ 200 };
		bool toggle_publishing_{ true };
		std::unique_ptr<ros::NodeHandle> node_handle_{ nullptr };
		std::unique_ptr<ros::Publisher> pub_{ nullptr };
		std::string topic_;
		float linear_{ 0.0 };
		float angular_{ 0.0 };
		std::unique_ptr<ros::Subscriber> sub_motion_state_{ nullptr };
		std::unique_ptr<ros::Subscriber> sub_sw_estop_{ nullptr };
		std::unique_ptr<ros::Subscriber> sub_rc_state_{ nullptr };
		bool sw_estopped_{ false };
		std::atomic_bool toggle_estop_{ false };
		std::atomic_bool toggle_collision_{ false };
		std::atomic_bool remote_mode_{ false };
		std::atomic_bool activated_{ true };
		std::atomic_bool terminated_{ false };
		std::thread th_publish_;
	};
} // end namespace whi_rviz_plugins
