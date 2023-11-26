/******************************************************************
tool for navigation initial and goal controll under namespace

Features:
- 2D initial pose
- 2D navigation goal
- namespace

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2023-04-07: Initial version
2023-xx-xx: xxx
******************************************************************/
#pragma once
#include "panel_navi_namespace.h"
#include <whi_interfaces/WhiMotionState.h>

#include <rviz/default_plugin/tools/pose_tool.h>
#include <rviz/panel_dock_widget.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/ros_topic_property.h>

#include <memory>

namespace whi_rviz_plugins
{
    class NaviNsTool: public rviz::PoseTool
    {
        Q_OBJECT
    public:
        NaviNsTool();
        ~NaviNsTool();

        void onInitialize() override;

    protected:
        void onPoseSet(double X, double Y, double Theta) override;

    private:
        void setToolType(int Type);
        void setNs(const std::string& Namespace);
        void load(const rviz::Config& Config) override;
        void save(rviz::Config Config) const override;
        void subCallbackMotionState(const whi_interfaces::WhiMotionState::ConstPtr& MotionState);

    private Q_SLOTS:
        void updateTopic();
        void updateMotionStateTopic();

    private:
        std::unique_ptr<ros::NodeHandle> node_handle_{ nullptr };
        std::unique_ptr<ros::Publisher> pub_{ nullptr };
        rviz::PanelDockWidget* frame_dock_{ nullptr };
        NaviNsPanel* panel_{ nullptr };
        int type_{ TYPE_INITIAL_POSE };
        std::string namespace_;
        rviz::FloatProperty* std_dev_x_;
        rviz::FloatProperty* std_dev_y_;
        rviz::FloatProperty* std_dev_theta_;
        rviz::RosTopicProperty* motion_state_topic_property_;
        std::unique_ptr<ros::Subscriber> sub_motion_state_{ nullptr };
        std::atomic_bool critical_collision_{ false };
		std::atomic_bool remote_mode_{ false };
    };
} // end namespace whi_rviz_plugins
