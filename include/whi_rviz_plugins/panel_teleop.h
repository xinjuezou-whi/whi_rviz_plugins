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
#include <memory>

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

	private:
		void keyPressEvent(QKeyEvent* Event) override;
		void focusOutEvent(QFocusEvent* Event) override;
		void focusInEvent(QFocusEvent* Event) override;

	private:
		Ui::NaviTeleop* ui_{ nullptr };
        TwistWidget* twist_widget_{ nullptr };
		QTimer* timer_pub_{ nullptr };
		int interval_pub_{ 200 };
		bool toggle_publishing_{ true };
		std::unique_ptr<ros::NodeHandle> node_handle_{ nullptr };
		std::unique_ptr<ros::Publisher> pub_{ nullptr };
		std::string topic_;
		float linear_{ 0.0 };
		float angular_{ 0.0 };
	};
} // end namespace whi_rviz_plugins
