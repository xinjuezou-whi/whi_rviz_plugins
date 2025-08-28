/******************************************************************
rviz plugin for motion status

Features:
- panel widget
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_rviz_plugins/panel_state.h"
#include "whi_rviz_plugins/utility.h"
#include "ui_navi_state.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <angles/angles.h>
#include "ament_index_cpp/get_package_share_directory.hpp"

#include <iostream>
#include <sstream>
#include <thread>

#include <QMessageBox>

namespace whi_rviz_plugins
{
    StatePanel::StatePanel(std::shared_ptr<rclcpp::Node> NodeHandle, QWidget* Parent/* = nullptr*/)
		: node_handle_(NodeHandle)
        , QWidget(Parent), ui_(new Ui::NaviState())
	{
		// set up the GUI
		ui_->setupUi(this);

		// WHI logo
		auto packagePath = getPackagePath();
		QImage logo;
		if (logo.load(QString((packagePath + "/icons/classes/whi_logo.png").c_str())))
		{
			QImage scaled = logo.scaledToHeight(48);
			ui_->label_logo->setPixmap(QPixmap::fromImage(scaled));
		}
        // other properties
        ui_->label_goal->setText("none");
        ui_->label_eta->setText("no info");
        // indicator
        setIndicatorIcon(ui_->label_indicator_1, INDICATOR_GREY);
        setIndicatorIcon(ui_->label_indicator_2, INDICATOR_GREY);
        setIndicatorIcon(ui_->label_indicator_3, INDICATOR_GREY);
        setIndicatorIcon(ui_->label_indicator_4, INDICATOR_GREY);
        setIndicatorIcon(ui_->label_indicator_5, INDICATOR_GREY);
        setIndicatorIcon(ui_->label_indicator_6, INDICATOR_GREY);
        setIndicatorIcon(ui_->label_indicator_7, INDICATOR_GREY);
        setIndicatorIcon(ui_->label_indicator_8, INDICATOR_GREY);
        setIndicatorText(ui_->label_indicator_cap_1, "inactve");
        setIndicatorText(ui_->label_indicator_cap_2, "auto");
        setIndicatorText(ui_->label_indicator_cap_3, "task");
        setIndicatorText(ui_->label_indicator_cap_4, "reserved");
        setIndicatorText(ui_->label_indicator_cap_5, "reserved");
        setIndicatorText(ui_->label_indicator_cap_6, "reserved");
        setIndicatorText(ui_->label_indicator_cap_7, "reserved");
        setIndicatorText(ui_->label_indicator_cap_8, "reserved");
        // dynamic ones
        setTempHumVisibility(false);
        setLabelIcon(ui_->label_temperature, "/icons/classes/temperature.png", 24);
        setLabelIcon(ui_->label_humidity, "/icons/classes/humidity.png", 24);
        // buttons
        ui_->pushButton_estop->setCheckable(true);
        QIcon icon;
        icon.addFile(QString((getPackagePath() + "/icons/classes/estop_released.png").c_str()),
            QSize(), QIcon::Normal, QIcon::Off);
        ui_->pushButton_estop->setIcon(icon);
        estop_init_height_ = ui_->pushButton_estop->height() + 10;
        ui_->pushButton_estop->setIconSize(QSize(estop_init_height_, estop_init_height_));
        // signals
		connect(ui_->pushButton_clear, &QPushButton::clicked, this, [=]() { clearButtonClicked(); });
        connect(ui_->pushButton_reset_imu, &QPushButton::clicked, this, [=]() { resetImuButtonClicked(); });
        connect(ui_->pushButton_reset_rc, &QPushButton::clicked, this, [=]() { resetRcButtonClicked(); });
        connect(ui_->pushButton_reset_rgbd, &QPushButton::clicked, this, [=]() { resetRgbdButtonClicked(); });
        connect(ui_->pushButton_estop, &QPushButton::toggled, this, [=](bool Checked) { estopButtonToggled(Checked); });

        // advertised estop topic
        setEstopTopic("estop");
    }

    StatePanel::~StatePanel()
	{
		delete ui_;
	}

    void StatePanel::setVelocities(double Linear, double Angular)
    {
        ui_->label_linear->setText(QString::number(Linear, 'f', 2));
        ui_->label_angular->setText(QString::number(Angular, 'f', 2));
    }

    void StatePanel::setGoal(const geometry_msgs::msg::Pose& Goal)
    {
        tf2::Quaternion quaternion(Goal.orientation.x, Goal.orientation.y, Goal.orientation.z, Goal.orientation.w);
        double roll = 0.0, pitch = 0.0, yaw = 0.0;
  		tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

        QString goal;
        goal = "x: " + QString::number(Goal.position.x, 'f', 2) + ", y:" + QString::number(Goal.position.y, 'f', 2) +
            ", yaw: " + QString::number(angles::to_degrees(yaw), 'f', 2);
        ui_->label_goal->setText(goal);
    }

    void StatePanel::setEta(const std::string& Eta)
    {
        ui_->label_eta->setText(Eta.c_str());
    }

    void StatePanel::setMotionState(const whi_interfaces::msg::WhiMotionState::SharedPtr State)
    {
        if (!first_state_msg_)
        {
            first_state_msg_ = std::make_unique<whi_interfaces::msg::WhiMotionState>();
            first_state_msg_->header.stamp = node_handle_->get_clock()->now();
        }
        rclcpp::Time stamp(first_state_msg_->header.stamp);
        std::time_t rawTime = static_cast<std::time_t>(stamp.seconds());
        struct tm* timeInfo = localtime(&rawTime);
        const int LEN = 64;
        char output[LEN];
        std::strftime(output, LEN, "%Y.%m.%d-%H:%M:%S", timeInfo);
        ui_->label_started->setText(output);
        double hours = (rclcpp::Time(State->header.stamp) - rclcpp::Time(first_state_msg_->header.stamp)).seconds() / 3600.0;
        if (hours > 0.0)
        {
            ui_->label_running_hours->setText(QString::number(hours, 'f', 2));
        }

        if (State->state == whi_interfaces::msg::WhiMotionState::STA_STANDBY)
        {
            if (ui_->label_indicator_cap_1->text() != "SE-Stop")
            {
                ui_->pushButton_estop->setChecked(false);
                setIndicatorIcon(ui_->label_indicator_1, INDICATOR_GREEN);
                setIndicatorText(ui_->label_indicator_cap_1, "standby");
            }
            setIndicatorIcon(ui_->label_indicator_3, INDICATOR_GREY);
            setIndicatorText(ui_->label_indicator_cap_3, "task");
        }
        else if (State->state == whi_interfaces::msg::WhiMotionState::STA_RUNNING)
        {
            if (ui_->label_indicator_cap_1->text() != "SE-Stop")
            {
                setIndicatorIcon(ui_->label_indicator_1, INDICATOR_YELLOW);
                setIndicatorText(ui_->label_indicator_cap_1, "running");
            }
            setIndicatorIcon(ui_->label_indicator_3, INDICATOR_GREY);
            setIndicatorText(ui_->label_indicator_cap_3, "task");
        }
        else if (State->state == whi_interfaces::msg::WhiMotionState::STA_OPERATING)
        {
            static double preSetSec = hours;
            static bool toggle = true;
            const double duration = 0.5 / 3600.0;
            if (hours - preSetSec > duration)
            {
                if (toggle)
                {
                    setIndicatorIcon(ui_->label_indicator_3, INDICATOR_BLUE);
                }
                else
                {
                    setIndicatorIcon(ui_->label_indicator_3, INDICATOR_GREY);
                }
                toggle = !toggle;
                preSetSec = hours;
            }
            setIndicatorText(ui_->label_indicator_cap_3, "operating");
        }
        else if (State->state == whi_interfaces::msg::WhiMotionState::STA_FAULT)
        {
            setIndicatorIcon(ui_->label_indicator_1, INDICATOR_RED);
            setIndicatorText(ui_->label_indicator_cap_1, "fault");
        }
        else if (State->state == whi_interfaces::msg::WhiMotionState::STA_ESTOP)
        {
            ui_->pushButton_estop->setChecked(true);

            setIndicatorText(ui_->label_indicator_cap_1, "E-Stop");
            setIndicatorIcon(ui_->label_indicator_3, INDICATOR_GREY);
        }
        else if (State->state == whi_interfaces::msg::WhiMotionState::STA_CRITICAL_COLLISION)
        {
            setIndicatorIcon(ui_->label_indicator_1, INDICATOR_ORANGE);
            setIndicatorText(ui_->label_indicator_cap_1, "collision");
        }
    }

    void StatePanel::setRcState(const whi_interfaces::msg::WhiRcState::SharedPtr State)
    {
        if (!non_realtime_loop_)
        {
            auto period = std::chrono::milliseconds(200);
            non_realtime_loop_ = node_handle_->create_wall_timer(
                period, std::bind(&StatePanel::update, this));  
        }

        if (State->state == whi_interfaces::msg::WhiRcState::STA_AUTO)
        {
            setIndicatorIcon(ui_->label_indicator_2, INDICATOR_GREEN);
            setIndicatorText(ui_->label_indicator_cap_2, "auto");
        }
        else if (State->state == whi_interfaces::msg::WhiRcState::STA_REMOTE)
        {
            setIndicatorIcon(ui_->label_indicator_2, INDICATOR_BLUE);
            setIndicatorText(ui_->label_indicator_cap_2, "remote");
        }

        last_updated_rc_ = system_clock_.now();
    }

    void StatePanel::setArmState(const whi_interfaces::msg::WhiMotionState::SharedPtr State)
    {
        if (State != nullptr)
        {
            if (State->state == whi_interfaces::msg::WhiMotionState::STA_BOOTING)
            {
                setIndicatorIcon(ui_->label_indicator_4, INDICATOR_YELLOW);
                setIndicatorText(ui_->label_indicator_cap_4, "arm booting");
            }
            else if (State->state == whi_interfaces::msg::WhiMotionState::STA_STANDBY)
            {
                setIndicatorIcon(ui_->label_indicator_4, INDICATOR_GREEN);
                setIndicatorText(ui_->label_indicator_cap_4, "arm standby");
            }
            else if (State->state == whi_interfaces::msg::WhiMotionState::STA_FAULT)
            {
                setIndicatorIcon(ui_->label_indicator_4, INDICATOR_RED);
                setIndicatorText(ui_->label_indicator_cap_4, "arm fault");
            }

            if (last_updated_arm_ == nullptr)
            {
                last_updated_arm_ = std::make_unique<rclcpp::Time>(system_clock_.now());
            }
            else
            {
                *last_updated_arm_ = system_clock_.now();
            }
        }
        else
        {
            setIndicatorIcon(ui_->label_indicator_4, INDICATOR_GREY);
            setIndicatorText(ui_->label_indicator_cap_4, "reserved");
        }
    }

    void StatePanel::setImuState()
    {
        if (!non_realtime_loop_)
        {
            auto updateFreq = std::chrono::milliseconds(200);
            non_realtime_loop_ = node_handle_->create_wall_timer(
                updateFreq, std::bind(&StatePanel::update, this));  
        }
        
        setIndicatorIcon(ui_->label_indicator_5, INDICATOR_GREEN);
        setIndicatorText(ui_->label_indicator_cap_5, "IMU");

        last_updated_imu_ = system_clock_.now();
    }

    void StatePanel::setRcStateTopic(const std::string& Topic)
    {
        pub_rc_state_ = node_handle_->create_publisher<whi_interfaces::msg::WhiRcState>(Topic, 50);
    }

    void StatePanel::setEstopTopic(const std::string& Topic)
    {
        pub_estop_ = node_handle_->create_publisher<std_msgs::msg::Bool>(Topic, 50);
    }

    void StatePanel::setBatteryInfo(int Soc, int Soh)
    {
        ui_->label_soc->setText(QString::number(Soc) + "%");
        ui_->label_soh->setText(QString::number(Soh) + "%");

        setBatteryIcon(ui_->label_battery, Soc);
    }

    void StatePanel::setTempHum(double Temperature, double Humidity)
    {
        setTempHumVisibility(true);
        ui_->label_temperature_text->setText(QString::number(Temperature, 'f', 1) + "°C");
        ui_->label_humidity_text->setText(QString::number(Humidity, 'f', 1) + "%");
    }

    void StatePanel::setIndicatorIcon(QLabel* Label, int Type)
    {
        std::string iconFile("/icons/classes/indicator_");
        switch (Type)
        {
        case INDICATOR_RED:
            iconFile += "red.png";
            break;
        case INDICATOR_ORANGE:
            iconFile += "orange.png";
            break;
        case INDICATOR_YELLOW:
            iconFile += "yellow.png";
            break;
        case INDICATOR_GREEN:
            iconFile += "green.png";
            break;
        case INDICATOR_BLUE:
            iconFile += "blue.png";
            break;
        default:
            iconFile += "grey.png";
            break;
        }

        setLabelIcon(Label, iconFile, 24);
    }

    void StatePanel::setIndicatorText(QLabel* Label, const std::string& Text)
    {
        Label->setText(Text.c_str());
    }

    void StatePanel::setBatteryIcon(QLabel* Label, int Soc)
    {
        std::string iconFile("/icons/classes/bat_");
        if (Soc > 80)
        {
            iconFile += "100.png";
        }
        else if (Soc > 60 && Soc <= 80)
        {
            iconFile += "80.png";
        }
        else if (Soc > 40 && Soc <= 60)
        {
            iconFile += "60.png";
        }
        else if (Soc > 20 && Soc <= 40)
        {
            iconFile += "40.png";
        }
        else
        {
            iconFile += "20.png";
        }

        setLabelIcon(Label, iconFile, 24);
    }

	void StatePanel::setTempHumVisibility(bool Visibale)
    {
        ui_->label_temperature->setVisible(Visibale);
        ui_->label_temperature_text->setVisible(Visibale);
        ui_->label_humidity->setVisible(Visibale);
        ui_->label_humidity_text->setVisible(Visibale);
    }

    void StatePanel::setLabelIcon(QLabel* Label, const std::string& IconFile, int Scale)
    {
        QImage indicator;
        if (indicator.load(QString((getPackagePath() + IconFile).c_str())))
		{
            QImage scaled = indicator.scaledToHeight(Scale);
			Label->setPixmap(QPixmap::fromImage(scaled));
		}
    }

    void StatePanel::clearButtonClicked()
    {
        whi_interfaces::msg::WhiRcState msgState;
        msgState.state = whi_interfaces::msg::WhiRcState::STA_CLEAR_FAULT;
        pub_rc_state_->publish(msgState);
    }

    static std::vector<std::string> splitStringBySpace(const std::string& Src)
    {
        std::vector<std::string> vec;

        std::istringstream is(Src);
        std::string dummy;
        while (is >> dummy)
        {
            vec.push_back(dummy);
        }

        return vec;
    }

    static bool killProcedure(const std::string& Name)
    {
        std::string cmd("ps aux | grep " + Name + "| grep -v grep");
		std::vector<std::string> res = pipeExecute(cmd.c_str());
        if (res.size() > 0)
        {
            for (const auto& it : res)
            {
                std::vector<std::string> separated = splitStringBySpace(it);
#ifdef DEBUG
                for (const auto& itd : res)
                {
                    std::cout << "----------------" << std::endl;
                    std::cout << itd << std::endl;
                }
                std::cout << "===================" << std::endl;
                for (const auto& itd : separated)
                {
                    std::cout << itd << ",";
                }
                std::cout << std::endl;
#endif
                if (!separated.empty())
                {
                    std::string cmd("sudo kill -9 " + separated[1]);
                    system(cmd.c_str());

                    return true;
                }
            }
        }

        return false;
    }

    static void launchProcedure(const std::string& Name)
    {
        std::string cmd("${HOME}/ros2_ws/./" + Name + ".sh &");
        system(cmd.c_str());
    }

	void StatePanel::resetImuButtonClicked()
    {
        std::string name("whi_imu_node");
        if (killProcedure(name))
        {
            RCLCPP_INFO_STREAM(node_handle_->get_logger(), name << " was successfully terminated");
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            launchProcedure(name);
        }
        else
        {
            QMessageBox::information(nullptr, tr("Info"), tr("Not in IMU fuse mode"));
        }
    }

	void StatePanel::resetRcButtonClicked()
    {
        std::string name("whi_rc_bridge_node");
        if (killProcedure(name))
        {
            RCLCPP_INFO_STREAM(node_handle_->get_logger(), name << " was successfully terminated");
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            launchProcedure(name);
        }
        else
        {
            QMessageBox::information(nullptr, tr("Info"), tr("No RC node running"));
        }
    }

    void StatePanel::resetRgbdButtonClicked()
    {
        std::string name("whi_realsense2_camera_node");
        if (killProcedure(name))
        {
            RCLCPP_INFO_STREAM(node_handle_->get_logger(), name << " was successfully terminated");
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            launchProcedure(name);
        }
        else
        {
            QMessageBox::information(nullptr, tr("Info"), tr("No realsense2 camera node running"));
        }
    }

    void StatePanel::estopButtonToggled(bool Checked)
    {
        std_msgs::msg::Bool msg;
        msg.data = Checked;
        pub_estop_->publish(msg);

        setEstopIcon(Checked);
    }

    void StatePanel::setEstopIcon(bool Checked)
    {
        std::string pkgPath = getPackagePath();
        QIcon icon;
        if (Checked)
        {
            icon.addFile(QString(pkgPath.c_str()) + "/icons/classes/estop_pressed.png",
                QSize(), QIcon::Normal, QIcon::Off);
            setIndicatorIcon(ui_->label_indicator_1, INDICATOR_ORANGE);
            setIndicatorText(ui_->label_indicator_cap_1, "SE-Stop");
        }
        else
        {
            icon.addFile(QString(pkgPath.c_str()) + "/icons/classes/estop_released.png",
                QSize(), QIcon::Normal, QIcon::Off);
            setIndicatorIcon(ui_->label_indicator_1, INDICATOR_GREY);
            setIndicatorText(ui_->label_indicator_cap_1, "none");
        }
        ui_->pushButton_estop->setIcon(icon);
        ui_->pushButton_estop->setIconSize(QSize(estop_init_height_, estop_init_height_));
    }

    void StatePanel::update()
    {
        auto current = system_clock_.now();

        if ((current - last_updated_imu_).seconds() > 2.0)
        {
            setIndicatorIcon(ui_->label_indicator_5, INDICATOR_RED);
            setIndicatorText(ui_->label_indicator_cap_5, "IMU");
        }
        if ((current - last_updated_rc_).seconds() > 2.0)
        {
            if (ui_->label_indicator_cap_2->text() == "remote")
            {
                setIndicatorIcon(ui_->label_indicator_2, INDICATOR_RED);
            }
        }
        if (last_updated_arm_ && (current - *last_updated_arm_).seconds() > 2.0)
        {
            setIndicatorIcon(ui_->label_indicator_4, INDICATOR_RED);
            setIndicatorText(ui_->label_indicator_cap_4, "arm fault");
        }
    }

    std::string StatePanel::getPackagePath() const
    {
        return ament_index_cpp::get_package_share_directory("whi_rviz_plugins");
    }
} // end namespace whi_rviz_plugins
