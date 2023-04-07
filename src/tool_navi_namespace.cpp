/******************************************************************
tool for navigation initial and goal controll under namespace

Features:
- 2D initial pose
- 2D navigation goal
- namespace

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_rviz_plugins/tool_navi_namespace.h"

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <rviz/window_manager_interface.h>
#include <rviz/display_context.h>

#include <pluginlib/class_list_macros.h>

namespace whi_rviz_plugins
{
    NaviNsTool::NaviNsTool()
        : node_handle_(std::make_unique<ros::NodeHandle>())
    {
        std::cout << "\nWHI RViz plugin for navigation goal with namespace VERSION 00.01.ing" << std::endl;
        std::cout << "Copyright @ 2023-2024 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n" << std::endl;

        shortcut_key_ = 'n';
    }

    void NaviNsTool::onInitialize()
    {
        PoseTool::onInitialize();
        setName("2D Pose Estimate");
        updateTopic();

        panel_ = new NaviNsPanel();
        rviz::WindowManagerInterface* windowContext = context_->getWindowManager();
        if (windowContext)
        {
            frame_dock_ = windowContext->addPane("Navi_namespace", panel_); // getName() return "" ???
            frame_dock_->setIcon(getIcon()); // set the image name as same as the name of plugin
        }
    }

    void NaviNsTool::onPoseSet(double X, double Y, double Theta)
    {
        // std::string fixed_frame = context_->getFixedFrame().toStdString();
        // geometry_msgs::PoseWithCovarianceStamped pose;
        // pose.header.frame_id = fixed_frame;
        // pose.header.stamp = ros::Time::now();
        // pose.pose.pose.position.x = x;
        // pose.pose.pose.position.y = y;

        // geometry_msgs::Quaternion quat_msg;
        // tf2::Quaternion quat;
        // quat.setRPY(0.0, 0.0, theta);
        // pose.pose.pose.orientation = tf2::toMsg(quat);
        // pose.pose.covariance[6 * 0 + 0] = std::pow(std_dev_x_->getFloat(), 2);
        // pose.pose.covariance[6 * 1 + 1] = std::pow(std_dev_y_->getFloat(), 2);
        // pose.pose.covariance[6 * 5 + 5] = std::pow(std_dev_theta_->getFloat(), 2);
        // ROS_INFO("Setting pose: %.3f %.3f %.3f [frame=%s]", x, y, theta, fixed_frame.c_str());
        // pub_.publish(pose);
    }

    void NaviNsTool::load(const rviz::Config& Config)
    {
        rviz::Tool::load(Config);

        panel_->load(Config);
    }

    void NaviNsTool::save(rviz::Config Config) const
    {
        rviz::Tool::save(Config);

        panel_->save(Config);
    }

    void NaviNsTool::updateTopic()
    {
        try
        {
            pub_ = std::make_unique<ros::Publisher>(
                node_handle_->advertise<geometry_msgs::PoseWithCovarianceStamped>("test", 10));
        }
        catch (const ros::Exception& e)
        {
            ROS_ERROR_STREAM_NAMED("InitialPoseTool", e.what());
        }
    }

    PLUGINLIB_EXPORT_CLASS(whi_rviz_plugins::NaviNsTool, rviz::Tool)
}  // namespace whi_rviz_plugins
