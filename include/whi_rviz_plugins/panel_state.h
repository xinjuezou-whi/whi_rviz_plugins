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
2025-08-04: Migrate from ROS 1
2025-xx-xx: xxx
******************************************************************/
#pragma once
#include <whi_interfaces/msg/whi_motion_state.hpp>
#include <whi_interfaces/msg/whi_state.hpp>
#include <whi_interfaces/msg/whi_rc_state.hpp>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <QTimer>

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
		StatePanel(rviz_common::DisplayContext* DisplayContext, QWidget* Parent = nullptr);
		~StatePanel() override;

	public:
		void setRobotFrame(const std::string& Frame);
        void setGoal(const geometry_msgs::msg::Pose& Goal);
        void setEta(const std::string& Eta);
		void setWhiState(const whi_interfaces::msg::WhiState::SharedPtr State);
		void setRcStateTopic(const std::string& Topic);
		void setEstopTopic(const std::string& Topic);
		void setBatteryInfo(int Soc, int Soh);

	private:
		void setPose(QLabel* Label, const geometry_msgs::msg::Pose& Pose);
		void setVelocities(double Linear, double Angular);
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
		void update();
		std::string getPackagePath() const;
		bool getCurrentPose(geometry_msgs::msg::PoseStamped& GlobalPose, const std::string& GlobalFrame = "map",
			const std::string& RobotFrame = "base_link", const double TransformTimeout = 0.2,
			const rclcpp::Time Stamp = rclcpp::Time());

	private:
        enum IndicatorType { INDICATOR_GREY = 0, INDICATOR_RED, INDICATOR_ORANGE,
			INDICATOR_YELLOW, INDICATOR_GREEN, INDICATOR_BLUE };
		Ui::NaviState* ui_{ nullptr };
		rviz_common::DisplayContext* context_;
		rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr node_rviz_weak_;
		rclcpp::Publisher<whi_interfaces::msg::WhiRcState>::SharedPtr pub_rc_state_{ nullptr };
		rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_estop_{ nullptr };
		QTimer* qtimer_{ nullptr };
		rclcpp::Time start_;
		rclcpp::Time last_updated_imu_;
		rclcpp::Time last_updated_rc_;
		rclcpp::Time last_updated_estop_;
		std::unique_ptr<rclcpp::Time> last_updated_arm_{ nullptr };
		int estop_init_height_{ 50 };
		enum EstopState { ESTOP_CLEAR = 0, ESTOP_HW, ESTOP_SW };
		int estop_state_{ ESTOP_CLEAR };
		rclcpp::Clock system_clock_{RCL_SYSTEM_TIME};
		std::shared_ptr<tf2_ros::Buffer> tf_;
		std::unique_ptr<tf2_ros::TransformListener> tf_listener_{ nullptr };
		std::string robot_frame_{ "base_link" };
	};
} // end namespace whi_rviz_plugins
