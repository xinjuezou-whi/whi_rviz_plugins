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
2025-08-01: Migrate from ROS 1
2025-xx-xx: xxx
******************************************************************/
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rviz_common/panel.hpp>

#include <string>
#include <memory>

namespace whi_rviz_plugins
{
    class MapSaverPanel : public rviz_common::Panel 
    {
        Q_OBJECT
    public:
        MapSaverPanel(QWidget* Parent = nullptr);
        virtual ~MapSaverPanel() = default;

    public:
        void onInitialize() override;

    private:
		void initLayout();
        bool mapServerValid();
        bool save(const std::string& File);
        void subCallbackMap(const nav_msgs::msg::OccupancyGrid::SharedPtr Msg);

    private:
        rclcpp::Node::SharedPtr node_handle_{ nullptr };
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_{ nullptr };
        rclcpp::Time map_received_{ 0 };
    };
} // end namespace whi_rviz_plugins
