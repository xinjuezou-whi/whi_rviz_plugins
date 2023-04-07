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
#include <rviz/default_plugin/tools/pose_tool.h>
#include <rviz/panel_dock_widget.h>

#include <memory>

namespace whi_rviz_plugins
{
    class NaviNsTool: public rviz::PoseTool
    {
        Q_OBJECT
    public:
        NaviNsTool();
        ~NaviNsTool() = default;

        void onInitialize() override;

    protected:
        void onPoseSet(double X, double Y, double Theta) override;

    private:
        void load(const rviz::Config& Config) override;
        void save(rviz::Config Config) const override;

    private Q_SLOTS:
        void updateTopic();

    private:
        std::unique_ptr<ros::NodeHandle> node_handle_{ nullptr };
        std::unique_ptr<ros::Publisher> pub_{ nullptr };
        rviz::PanelDockWidget* frame_dock_{ nullptr };
        NaviNsPanel* panel_{ nullptr };
    };
} // end namespace whi_rviz_plugins
