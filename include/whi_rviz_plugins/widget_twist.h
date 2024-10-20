/******************************************************************
custom widget for drawing twist

Features:
- drawing linear and angular
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2022-11-18: Initial version
2022-xx-xx: xxx
******************************************************************/
#pragma once
#include <QWidget>

namespace whi_rviz_plugins
{
	class TwistWidget : public QWidget
	{
		Q_OBJECT
	public:
		TwistWidget(QWidget* Parent = nullptr);

    public:
        QSize minimumSizeHint() const override;
        QSize sizeHint() const override;

	public:
		void setLinear(float Linear);
		void setLinearMin(float Min);
		void setLinearMax(float Max);
		void setLinearStep(float Step);
		void setAngular(float Angular);
		void setAngularMin(float Min);
		void setAngularMax(float Max);
		void setAngularStep(float Step);
		float getLinear();
		float getLinearMin();
		float getLinearMax();
		float getLinearStep();
		float getAngular();
		float getAngularMin();
		float getAngularMax();
		float getAngularStep();
		void toggleIndicator(bool Toggle, bool State);

    protected:
        void paintEvent(QPaintEvent* Event) override;

	private:
		float linear_velocity_{ 0.0 };
  		float angular_velocity_{ 0.0 };
		float linear_min_{ 0.05 };
		float linear_scale_{ 2.0 };
		float linear_step_{ 0.01 };
		float angular_min_{ 0.1 };
		float angular_scale_{ 1.57 };
		float angular_step_{ 0.05 };
		bool toggle_indicator_{ true };
		bool state_active_{ true };
	};
} // end namespace whi_rviz_plugins
