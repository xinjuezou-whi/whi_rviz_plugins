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
		void setMotionState(int State);

	private:
        std::string getPackagePath() const;
        void setIndicatorIcon(QLabel* Label, int Type);
		void setIndicatorText(QLabel* Label, const std::string& Text);

	private:
        enum IndicatorType { INDICATOR_GREY = 0, INDICATOR_ORANGE, INDICATOR_YELLOW, INDICATOR_GREEN };
		Ui::NaviState* ui_{ nullptr };
	};
} // end namespace whi_rviz_plugins
