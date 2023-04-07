/******************************************************************
rviz plugin for navigation initial and goal controll under namespace

Features:
- 2D initial pose
- 2D navigation goal
- add namespace
- record namespace

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2023-04-04: Initial version
2023-xx-xx: xxx
******************************************************************/
#pragma once

#include <ros/ros.h>
#include <rviz/panel.h>

namespace Ui
{
class NaviMultipleNs;
}

namespace whi_rviz_plugins
{
    class NaviNsPanel : public rviz::Panel 
    {
        Q_OBJECT
    public:
        NaviNsPanel(QWidget* Parent = nullptr);
        ~NaviNsPanel() = default;

    public:
        void load(const rviz::Config& Config); // override;
        void save(rviz::Config Config) const; // override;

    private:
        Ui::NaviMultipleNs* ui_{ nullptr };
        std::shared_ptr<ros::NodeHandle> node_handle_{ nullptr };
    };
} // end namespace whi_rviz_plugins
