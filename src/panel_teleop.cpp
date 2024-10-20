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

#include <geometry_msgs/Twist.h>
#include <iostream>
#include <sstream>
#include <math.h>
#include <boost/filesystem.hpp>
#include <ros/package.h>
#include <QKeyEvent>
#include <QTimer>
#include <QMessageBox>

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
        , node_handle_(std::make_unique<ros::NodeHandle>())
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
        terminated_.store(true);
        if (th_publish_.joinable())
        {
            th_publish_.join();
        }
        timer_toggle_->stop();
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

    void TeleopPanel::setPubTopic(const std::string& Topic)
    {
        if (topic_ != Topic)
        {
            topic_ = Topic;

            terminated_.store(true);
            if (th_publish_.joinable())
            {
                th_publish_.join();
            }
            terminated_.store(false);

            pub_ = std::make_unique<ros::Publisher>(node_handle_->advertise<geometry_msgs::Twist>(topic_, 50));

            th_publish_ = std::thread
            {
                [this]() -> void
                {
                    while (!terminated_.load())
                    {
                        if (activated_ && !toggle_estop_.load() && !remote_mode_.load() && !toggle_collision_.load())
                        {
                            geometry_msgs::Twist msgTwist;
                            msgTwist.linear.x = linear_;
                            msgTwist.angular.z = angular_;
                            pub_->publish(msgTwist);
                        }

                        std::this_thread::sleep_for(std::chrono::milliseconds(interval_pub_));
                    }
                }
            };
        }
    }

    void TeleopPanel::setPubFunctionality(bool Active)
    {
        activated_.store(Active);

        if (Active)
        {
            if (timer_toggle_)
            {
                if (!timer_toggle_->isActive())
                {
                    timer_toggle_->start(interval_toggle_);
                }
            }
            else
            {
                // toggle timer
                timer_toggle_ = new QTimer(this);
                connect(timer_toggle_, &QTimer::timeout, this, [=]()
                {
                    if (!toggle_estop_.load() && !remote_mode_.load() && !toggle_collision_.load())
                    {
                        twist_widget_->toggleIndicator(toggle_publishing_, true);
                        toggle_publishing_ = !toggle_publishing_;
                    }
                });
                timer_toggle_->start(interval_toggle_);
            }
        }
        else
        {
            if (timer_toggle_)
            {
                timer_toggle_->stop();
            }
            twist_widget_->toggleIndicator(true, false);
        }
    }

    void TeleopPanel::setPubFrequency(float Frequency)
    {
        interval_pub_ = int(1000.0 / Frequency);
    }

    void TeleopPanel::moveLinear(int Dir)
    {
        if (isBypassed())
        {
            return;
        }

        linear_ = twist_widget_->getLinear() + Dir * twist_widget_->getLinearStep();
        linear_ = fabs(linear_) > twist_widget_->getLinearMax() ? Dir * twist_widget_->getLinearMax() : linear_;
        if (fabs(linear_) > 1e-3 && fabs(linear_) < twist_widget_->getLinearMin())
        {
            linear_ = Dir * twist_widget_->getLinearMin();
        }

        twist_widget_->setLinear(linear_);
        ui_->label_linear->setText(to_string_with_precision(linear_, 2).c_str());
    }

	void TeleopPanel::moveAngular(int Dir)
    {
        if (isBypassed())
        {
            return;
        }

        angular_ = twist_widget_->getAngular() + Dir * twist_widget_->getAngularStep();
        angular_ = fabs(angular_) > twist_widget_->getAngularMax() ? Dir * twist_widget_->getAngularMax() : angular_;
        if (fabs(angular_) > 1e-3 && fabs(angular_) < twist_widget_->getAngularMin())
        {
            angular_ = Dir * twist_widget_->getAngularMin();
        }

        twist_widget_->setAngular(angular_);
        ui_->label_angular->setText(to_string_with_precision(angular_, 2).c_str());
    }

    void TeleopPanel::halt()
    {
        if (isBypassed())
        {
            return;
        }

        linear_ = 0.0;
        angular_ = 0.0;
        twist_widget_->setLinear(linear_);
        twist_widget_->setAngular(angular_);

        ui_->label_linear->setText("0.0");
        ui_->label_angular->setText("0.0");
    }

	void TeleopPanel::setMotionStateTopic(const std::string& Topic)
	{
		if (!Topic.empty())
		{
        	sub_motion_state_ = std::make_unique<ros::Subscriber>(
            	node_handle_->subscribe<whi_interfaces::WhiMotionState>(Topic, 10,
            	std::bind(&TeleopPanel::subCallbackMotionState, this, std::placeholders::_1)));
		}
	}

    void TeleopPanel::setRcStateTopic(const std::string& Topic)
    {
		if (!Topic.empty())
		{
        	sub_rc_state_ = std::make_unique<ros::Subscriber>(
            	node_handle_->subscribe<whi_interfaces::WhiRcState>(Topic, 10,
            	std::bind(&TeleopPanel::subCallbackRcState, this, std::placeholders::_1)));
		}
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

    void TeleopPanel::subCallbackMotionState(const whi_interfaces::WhiMotionState::ConstPtr& MotionState)
    {
        if (MotionState->state == whi_interfaces::WhiMotionState::STA_ESTOP)
	    {
		    if (!toggle_estop_.load())
		    {
			    halt();
		    }
		    toggle_estop_.store(true);
	    }
	    else if (MotionState->state == whi_interfaces::WhiMotionState::STA_ESTOP_CLEAR)
	    {
		    toggle_estop_.store(false);
	    }

        if (MotionState->state == whi_interfaces::WhiMotionState::STA_CRITICAL_COLLISION)
	    {
		    if (!toggle_collision_.load())
		    {
			    halt();
		    }
		    toggle_collision_.store(true);
	    }
	    else if (MotionState->state == whi_interfaces::WhiMotionState::STA_CRITICAL_COLLISION_CLEAR)
	    {
		    toggle_collision_.store(false);
	    }
    }
    
    void TeleopPanel::subCallbackRcState(const whi_interfaces::WhiRcState::ConstPtr& RcState)
    {
        if (RcState->state == whi_interfaces::WhiRcState::STA_REMOTE)
        {
            if (!remote_mode_.load())
            {
                halt();
            }
            remote_mode_.store(true);
        }
        else if (RcState->state == whi_interfaces::WhiRcState::STA_AUTO)
        {
            remote_mode_.store(false);
        }
    }

    bool TeleopPanel::isBypassed()
    {
        if (toggle_estop_.load())
        {
            QMessageBox::information(this, tr("Info"), tr("E-Stop detected, command is ignored"));
        }
        else if (toggle_collision_.load())
        {
            QMessageBox::information(this, tr("Info"), tr("critical collision detected, command is ignored"));
        }
        else if (remote_mode_.load())
        {
            QMessageBox::information(this, tr("Info"), tr("vehicle is in remote mode, command is ignored"));
        }
        else
        {
            return false;
        }

        return true;
    }
} // end namespace whi_rviz_plugins
