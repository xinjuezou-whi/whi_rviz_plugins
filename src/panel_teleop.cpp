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
#include <QKeyEvent>
#include <QTimer>

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
        ui_->label_key_active->setStyleSheet("background-color: rgb(217, 217, 217);");
        // a widget normally must setFocusPolicy() to something other than Qt::NoFocus in order to receive focus events
        setFocusPolicy(Qt::StrongFocus);

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
        connect(ui_->pushButton_forward, &QPushButton::clicked, this, [=]() { moveLinear(1); });
        connect(ui_->pushButton_backward, &QPushButton::clicked, this, [=]() { moveLinear(-1); });
        connect(ui_->pushButton_left, &QPushButton::clicked, this, [=]() { moveAngular(1); });
        connect(ui_->pushButton_right, &QPushButton::clicked, this, [=]() { moveAngular(-1); });
        connect(ui_->pushButton_halt, &QPushButton::clicked, this, [=]() { halt(); });
    }

    TeleopPanel::~TeleopPanel()
	{
        timer_pub_->stop();
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

    void TeleopPanel::setPubFunctionality(bool Active)
    {
        if (Active)
        {
            if (timer_pub_)
            {
                if (!timer_pub_->isActive())
                {
                    timer_pub_->start(interval_pub_);
                }
            }
            else
            {
                // publish timer
                timer_pub_ = new QTimer(this);
                connect(timer_pub_, &QTimer::timeout, this, [=]()
                {
                    twist_widget_->toggleIndicator(toggle_publishing_, true);
                    toggle_publishing_ = !toggle_publishing_;
                });
                timer_pub_->start(interval_pub_);
            }
        }
        else
        {
            if (timer_pub_)
            {
                timer_pub_->stop();
            }
            twist_widget_->toggleIndicator(true, false);
        }
    }

    void TeleopPanel::setPubFrequency(float Frequency)
    {
        interval_pub_ = int(1000.0 / Frequency);
        if (timer_pub_)
        {
            timer_pub_->setInterval(interval_pub_);
        }
    }

    void TeleopPanel::moveLinear(int Dir)
    {
        float linear = fabs(twist_widget_->getLinear()) < 1e-3 ? Dir * twist_widget_->getLinearMin() :
            twist_widget_->getLinear() + Dir * twist_widget_->getLinearStep();
        linear = fabs(linear) < twist_widget_->getLinearMin() ? 0.0 : linear;
        if (fabs(linear) < twist_widget_->getLinearMax() + 1e-3)
        {
            twist_widget_->setLinear(linear);

            ui_->label_linear->setText(to_string_with_precision(linear, 2).c_str());
        }
    }

	void TeleopPanel::moveAngular(int Dir)
    {
        float angular = fabs(twist_widget_->getAngular()) < 1e-3 ? Dir * twist_widget_->getAngularStep() :
            twist_widget_->getAngular() + Dir * twist_widget_->getAngularStep();
        angular = fabs(angular) < twist_widget_->getAngularMin() ? 0.0 : angular;
        if (fabs(angular) < twist_widget_->getAngularMax() + 1e-3)
        {
            twist_widget_->setAngular(angular);

            ui_->label_angular->setText(to_string_with_precision(angular, 2).c_str());
        }
    }

    void TeleopPanel::halt()
    {
        twist_widget_->setLinear(0.0);
        twist_widget_->setAngular(0.0);

        ui_->label_linear->setText("0.0");
        ui_->label_angular->setText("0.0");
    }

    void TeleopPanel::keyPressEvent(QKeyEvent* Event)
    {
        switch (Event->key())
        {
        case Qt::Key_W:
            moveLinear(1);
            break;
        case Qt::Key_X:
            moveLinear(-1);
            break;
        case Qt::Key_A:
            moveAngular(1);
            break;
        case Qt::Key_D:
            moveAngular(-1);
            break;
        case Qt::Key_S:
        case Qt::Key_Space:
            halt();
            break;
        default:
            break;
        }
    }

    void TeleopPanel::focusOutEvent(QFocusEvent* Event)
    {
        ui_->label_key_active->setStyleSheet("background-color: rgb(217, 217, 217);");
    }

    void TeleopPanel::focusInEvent(QFocusEvent* Event)
    {
        ui_->label_key_active->setStyleSheet("background-color: rgb(146, 208, 80);");
    }
} // end namespace whi_rviz_plugins
