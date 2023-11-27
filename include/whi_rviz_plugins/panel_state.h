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
#include <rviz/panel.h>
#include <geometry_msgs/Pose.h>

#include <whi_interfaces/WhiMotionState.h>

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
		void setBatteryInfo(int Soc, int Soh);

	private:
        std::string getPackagePath() const;
        void setIndicatorIcon(QLabel* Label, int Type);
		void setIndicatorText(QLabel* Label, const std::string& Text);
		void setBatteryIcon(QLabel* Label, int Soc);
		void setLabelIcon(QLabel* Label, const std::string& IconFile, int Scale);

	private:
        enum IndicatorType { INDICATOR_GREY = 0, INDICATOR_RED, INDICATOR_ORANGE,
			INDICATOR_YELLOW, INDICATOR_GREEN, INDICATOR_BLUE };
		Ui::NaviState* ui_{ nullptr };
		whi_interfaces::WhiMotionState first_state_msg_;
	};
} // end namespace whi_rviz_plugins
