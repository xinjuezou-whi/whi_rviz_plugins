/******************************************************************
rviz plugin for saving map with map_server

Features:
- map_server map_saver -f <map>
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2022-11-15: Initial version
2022-xx-xx: xxx
******************************************************************/
#pragma once
#include <ros/ros.h>
#include "nav_msgs/GetMap.h"

#include <rviz/panel.h>
#include <string>
#include <memory>

namespace whi_rviz_plugins
{
    class MapSaverPanel : public rviz::Panel 
    {
        Q_OBJECT
    public:
        MapSaverPanel(QWidget* Parent = nullptr);
        ~MapSaverPanel() = default;

    private:
		void initLayout();
        bool mapServerValid();
        void save(std::string File);
        void subCallbackMap(const nav_msgs::OccupancyGridConstPtr& Map);

    private:
        std::unique_ptr<ros::Subscriber> map_sub_{ nullptr };
        std::unique_ptr<ros::NodeHandle> node_handle_{ nullptr };
        ros::Time map_received_;
    };
} // end namespace whi_rviz_plugins
