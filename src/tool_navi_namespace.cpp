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

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <rviz/window_manager_interface.h>
#include <rviz/display_context.h>
#include <rviz/ogre_helpers/arrow.h>
#include <pluginlib/class_list_macros.h>

#include <QMessageBox>

namespace whi_rviz_plugins
{
    NaviNsTool::NaviNsTool()
        : node_handle_(std::make_unique<ros::NodeHandle>())
    {
        std::cout << "\nWHI RViz plugin for navigation goal with namespace VERSION 00.05" << std::endl;
        std::cout << "Copyright @ 2023-2024 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n" << std::endl;

        shortcut_key_ = 'n';
        // properties for initial pose
        std_dev_x_ = new rviz::FloatProperty("X std deviation", 0.5, "X standard deviation for initial pose [m]",
            getPropertyContainer());
        std_dev_y_ = new rviz::FloatProperty("Y std deviation", 0.5, "Y standard deviation for initial pose [m]",
            getPropertyContainer());
        std_dev_theta_ = new rviz::FloatProperty("Theta std deviation", M_PI / 12.0,
            "Theta standard deviation for initial pose [rad]", getPropertyContainer());
        std_dev_x_->setMin(0);
        std_dev_y_->setMin(0);
        std_dev_theta_->setMin(0);
        motion_state_topic_property_ = new rviz::RosTopicProperty("Motion state topic", "motion_state",
            "whi_interfaces/WhiMotionState", "Topic of motion state",
            getPropertyContainer(), SLOT(updateMotionStateTopic()), this);
    }

    NaviNsTool::~NaviNsTool()
    {
        delete frame_dock_;
    }

    void NaviNsTool::onInitialize()
    {
        PoseTool::onInitialize();
        setToolType(type_);

        panel_ = new NaviNsPanel();
        panel_->registerTypeSetting(std::bind(&NaviNsTool::setToolType, this, std::placeholders::_1));
        panel_->registerNsSetting(std::bind(&NaviNsTool::setNs, this, std::placeholders::_1));
        rviz::WindowManagerInterface* windowContext = context_->getWindowManager();
        if (windowContext)
        {
            frame_dock_ = windowContext->addPane("Navi_namespace", panel_); // getName() return "" ???
            frame_dock_->setIcon(getIcon()); // set the image name as same as the name of plugin
        }
    }

    void NaviNsTool::onPoseSet(double X, double Y, double Theta)
    {
        std::string fixedFrame = context_->getFixedFrame().toStdString();
        if (type_ == TYPE_INITIAL_POSE)
        {
            geometry_msgs::PoseWithCovarianceStamped pose;
            pose.header.frame_id = fixedFrame;
            pose.header.stamp = ros::Time::now();
            pose.pose.pose.position.x = X;
            pose.pose.pose.position.y = Y;

            geometry_msgs::Quaternion quat_msg;
            tf2::Quaternion quat;
            quat.setRPY(0.0, 0.0, Theta);
            pose.pose.pose.orientation = tf2::toMsg(quat);
            pose.pose.covariance[6 * 0 + 0] = std::pow(std_dev_x_->getFloat(), 2);
            pose.pose.covariance[6 * 1 + 1] = std::pow(std_dev_y_->getFloat(), 2);
            pose.pose.covariance[6 * 5 + 5] = std::pow(std_dev_theta_->getFloat(), 2);
            ROS_INFO("Setting pose: %.3f %.3f %.3f [frame=%s]", X, Y, Theta, fixedFrame.c_str());
            pub_->publish(pose);
        }
        else if (type_ == TYPE_GOAL)
        {
            if (critical_collision_.load() || remote_mode_.load())
			{
				QString reason = critical_collision_.load() ? tr("critical collision") : tr("remote mode");
				QMessageBox::information(nullptr, tr("Info"), tr("vehicle is in ") + reason + tr(" command is ignored"));

				return;
			}

            tf2::Quaternion quat;
            quat.setRPY(0.0, 0.0, Theta);
            geometry_msgs::PoseStamped goal;
            goal.pose.orientation = tf2::toMsg(quat);
            goal.pose.position.x = X;
            goal.pose.position.y = Y;
            goal.header.frame_id = fixedFrame;
            goal.header.stamp = ros::Time::now();
            ROS_INFO("Setting goal: Frame:%s, Position(%.3f, %.3f, %.3f), Orientation(%.3f, %.3f, %.3f, %.3f) = "
                "Angle: %.3f\n",fixedFrame.c_str(), goal.pose.position.x, goal.pose.position.y, goal.pose.position.z,
                goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w, Theta);
            pub_->publish(goal);
        }
    }

    void NaviNsTool::setToolType(int Type)
    {
        type_ = Type;
        if (type_ == TYPE_INITIAL_POSE)
        {
            arrow_->setColor(0.0f, 1.0f, 0.0f, 1.0f);
        }
        else if (type_ == TYPE_GOAL)
        {
            arrow_->setColor(1.0f, 0.0f, 1.0f, 1.0f);
        }
        setName("Navi_namespace");
        updateTopic();
    }

    void NaviNsTool::setNs(const std::string& Namespace)
    {
        namespace_ = Namespace;
        updateTopic();
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

	void NaviNsTool::subCallbackMotionState(const whi_interfaces::WhiMotionState::ConstPtr& MotionState)
    {
        if (MotionState->state == whi_interfaces::WhiMotionState::STA_CRITICAL_COLLISION)
	    {
            critical_collision_.store(true);
	    }
	    else if (MotionState->state == whi_interfaces::WhiMotionState::STA_CRITICAL_COLLISION_CLEAR)
	    {
            critical_collision_.store(false);
	    }

        if (MotionState->state == whi_interfaces::WhiMotionState::STA_REMOTE)
        {
            remote_mode_.store(true);
        }
        else if (MotionState->state == whi_interfaces::WhiMotionState::STA_AUTO)
        {
            remote_mode_.store(false);
        }
    }

    void NaviNsTool::updateTopic()
    {
        try
        {
            if (type_ == TYPE_INITIAL_POSE)
            {
                pub_ = std::make_unique<ros::Publisher>(
                    node_handle_->advertise<geometry_msgs::PoseWithCovarianceStamped>(namespace_ + "/initialpose", 10));
            }
            else if (type_ == TYPE_GOAL)
            {
                pub_ = std::make_unique<ros::Publisher>(
                    node_handle_->advertise<geometry_msgs::PoseStamped>(namespace_ + "/move_base_simple/goal", 10));
            }
        }
        catch (const ros::Exception& e)
        {
            ROS_ERROR_STREAM_NAMED("WHI Navi tool with ns", e.what());
        }
    }

    void NaviNsTool::updateMotionStateTopic()
    {
        sub_motion_state_ = std::make_unique<ros::Subscriber>(
            node_handle_->subscribe<whi_interfaces::WhiMotionState>(motion_state_topic_property_->getTopicStd(), 10,
            std::bind(&NaviNsTool::subCallbackMotionState, this, std::placeholders::_1)));
    }

    PLUGINLIB_EXPORT_CLASS(whi_rviz_plugins::NaviNsTool, rviz::Tool)
}  // namespace whi_rviz_plugins
