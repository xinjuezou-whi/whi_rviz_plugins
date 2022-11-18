/******************************************************************
rviz plugin for generic teleop

Features:
- teleop with twist message
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_rviz_plugins/panel_teleop.h"
#include "whi_rviz_plugins/widget_twist.h"
#include "ui_navi_teleop.h"

#include <iostream>
#include <sstream>
#include <math.h>
#include <boost/filesystem.hpp>
#include <ros/package.h>
#include <QPainter>

namespace whi_rviz_plugins
{
    template <typename T>
    std::string to_string_with_precision(const T Value, const int Digits = 6)
    {
        std::ostringstream out;
        out.precision(Digits);
        out << std::fixed << Value;
        return out.str();
    }

    TeleopPanel::TeleopPanel(QWidget* Parent/* = nullptr*/)
		: QWidget(Parent), ui_(new Ui::NaviTeleop())
	{
		// set up the GUI
		ui_->setupUi(this);

        // create the twist widget
        twist_widget_ = new TwistWidget;
        ui_->horizontalLayout_twist->insertWidget(0, twist_widget_);

		// WHI logo
		boost::filesystem::path path(ros::package::getPath("whi_rviz_plugins"));
		QImage logo;
		if (logo.load(QString(path.string().c_str()) + "/icons/classes/whi_logo.png"))
		{
			QImage scaled = logo.scaledToHeight(48);
			ui_->label_logo->setPixmap(QPixmap::fromImage(scaled));
		}
        // pushbutton icons which cannot be found with relative path
        QIcon iconLeft;
        iconLeft.addFile(QString(path.string().c_str()) + "/icons/classes/left.png", QSize(), QIcon::Normal, QIcon::Off);
        ui_->pushButton_left->setIcon(iconLeft);
        QIcon iconRight;
        iconRight.addFile(QString(path.string().c_str()) + "/icons/classes/right.png", QSize(), QIcon::Normal, QIcon::Off);
        ui_->pushButton_right->setIcon(iconRight);
        QIcon iconForward;
        iconForward.addFile(QString(path.string().c_str()) + "/icons/classes/forward.png", QSize(), QIcon::Normal, QIcon::Off);
        ui_->pushButton_forward->setIcon(iconForward);
        QIcon iconBackward;
        iconBackward.addFile(QString(path.string().c_str()) + "/icons/classes/backward.png", QSize(), QIcon::Normal, QIcon::Off);
        ui_->pushButton_backward->setIcon(iconBackward);
        QIcon iconHalt;
        iconHalt.addFile(QString(path.string().c_str()) + "/icons/classes/halt.png", QSize(), QIcon::Normal, QIcon::Off);
        ui_->pushButton_halt->setIcon(iconHalt);

        // signals
        connect(ui_->pushButton_forward, &QPushButton::clicked, this, [=]()
        {
            float linear = fabs(twist_widget_->getLinear()) < 1e-3 ? twist_widget_->getLinearMin() :
                twist_widget_->getLinear() + twist_widget_->getLinearStep();
            linear = fabs(linear) < twist_widget_->getLinearMin() ? 0.0 : linear;
            if (fabs(linear) < twist_widget_->getLinearMax() + 1e-3)
            {
                twist_widget_->setLinear(linear);

                ui_->label_linear->setText(to_string_with_precision(linear, 2).c_str());
            }
        });
        connect(ui_->pushButton_backward, &QPushButton::clicked, this, [=]()
        {
            float linear = fabs(twist_widget_->getLinear()) < 1e-3 ? -twist_widget_->getLinearMin() :
                twist_widget_->getLinear() - twist_widget_->getLinearStep();
            linear = fabs(linear) < twist_widget_->getLinearMin() ? 0.0 : linear;
            if (fabs(linear) < twist_widget_->getLinearMax() + 1e-3)
            {
                twist_widget_->setLinear(linear);

                ui_->label_linear->setText(to_string_with_precision(linear, 2).c_str());
            }
        });
        connect(ui_->pushButton_left, &QPushButton::clicked, this, [=]()
        {
            float angular = twist_widget_->getAngular() + twist_widget_->getAngularStep();
            if (fabs(angular) < twist_widget_->getAngularMax() + 1e-3)
            {
                twist_widget_->setAngular(angular);

                ui_->label_angular->setText(to_string_with_precision(angular, 2).c_str());
            }
        });
        connect(ui_->pushButton_right, &QPushButton::clicked, this, [=]()
        {
            float angular = twist_widget_->getAngular() - twist_widget_->getAngularStep();
            if (fabs(angular) < twist_widget_->getAngularMax() + 1e-3)
            {
                twist_widget_->setAngular(angular);

                ui_->label_angular->setText(to_string_with_precision(angular, 2).c_str());
            }
        });
        connect(ui_->pushButton_halt, &QPushButton::clicked, this, [=]()
        {
            twist_widget_->setLinear(0.0);
            twist_widget_->setAngular(0.0);

            ui_->label_linear->setText("0.0");
            ui_->label_angular->setText("0.0");
        });
    }

    TeleopPanel::~TeleopPanel()
	{
		delete ui_;
	}

    void TeleopPanel::setLinearMin(float Min)
    {
        twist_widget_->setLinearMin(Min);
    }

    void TeleopPanel::setLinearMax(float Max)
    {
        twist_widget_->setLinearMax(Max);
    }

    void TeleopPanel::setLinearStep(float Step)
    {
        twist_widget_->setLinearStep(Step);
    }

	void TeleopPanel::setAngularMin(float Min)
    {
        twist_widget_->setAngularMin(Min);
    }

	void TeleopPanel::setAngularMax(float Max)
    {
        twist_widget_->setAngularMax(Max);
    }

	void TeleopPanel::setAngularStep(float Step)
    {
        twist_widget_->setAngularStep(Step);
    }
} // end namespace whi_rviz_plugins
