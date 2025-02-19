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
2022-xx-xx: xxx
******************************************************************/
#pragma once
#include <ros/ros.h>
#include <rviz/panel.h>
#include <geometry_msgs/Pose.h>

#include <whi_interfaces/WhiMotionState.h>
#include <whi_interfaces/WhiRcState.h>

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
		StatePanel(QWidget* Parent = nullptr);
		~StatePanel() override;

	public:
        void setVelocities(double Linear, double Angular);
        void setGoal(const geometry_msgs::Pose& Goal);
        void setEta(const std::string& Eta);
		void setMotionState(const whi_interfaces::WhiMotionState::ConstPtr& State);
		void setRcState(const whi_interfaces::WhiRcState::ConstPtr& State);
		void setArmState(const whi_interfaces::WhiMotionState::ConstPtr& State);
		void setImuState();
		void setRcStateTopic(const std::string& Topic);
		void setEstopTopic(const std::string& Topic);
		void setBatteryInfo(int Soc, int Soh);
		void setTempHum(double Temperature, double Humidity);

	private:
        std::string getPackagePath() const;
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
		void update(const ros::TimerEvent& Event);

	private:
        enum IndicatorType { INDICATOR_GREY = 0, INDICATOR_RED, INDICATOR_ORANGE,
			INDICATOR_YELLOW, INDICATOR_GREEN, INDICATOR_BLUE };
		Ui::NaviState* ui_{ nullptr };
		whi_interfaces::WhiMotionState first_state_msg_;
		std::unique_ptr<ros::NodeHandle> node_handle_{ nullptr };
		std::unique_ptr<ros::Publisher> pub_rc_state_{ nullptr };
		std::unique_ptr<ros::Publisher> pub_estop_{ nullptr };
		std::unique_ptr<ros::Timer> non_realtime_loop_{ nullptr };
		ros::Time last_updated_imu_;
		ros::Time last_updated_rc_;
		std::unique_ptr<ros::Time> last_updated_arm_{ nullptr };
		int estop_init_height_{ 50 };
	};
} // end namespace whi_rviz_plugins
