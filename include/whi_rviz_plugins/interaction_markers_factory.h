/******************************************************************
interaction marker factory

Features:
- creation

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2025-08-07: Initial version
2025-xx-xx: xxx
******************************************************************/
#pragma once
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/color_rgba.hpp>

namespace whi_rviz_plugins
{
    visualization_msgs::msg::InteractiveMarker makeEmptyInteractiveMarker(const std::string& Name,
        const geometry_msgs::msg::PoseStamped& Pose, double Scale);

    visualization_msgs::msg::InteractiveMarker make6DOFMarker(const std::string& Name,
        const geometry_msgs::msg::PoseStamped& Pose, double Scale, bool FixedOrientation = false);

    visualization_msgs::msg::InteractiveMarker makePlanarXYMarker(const std::string& Name,
        const geometry_msgs::msg::PoseStamped& Pose, double Scale, bool FixedOrientation = false);

    void addTArrowMarker(visualization_msgs::msg::InteractiveMarker& Marker);

    void addErrorMarker(visualization_msgs::msg::InteractiveMarker& Marker);

    void add6DOFControl(visualization_msgs::msg::InteractiveMarker& Marker, bool FixedOrientation = false);

    void addPlanarXYControl(visualization_msgs::msg::InteractiveMarker& Marker, bool FixedOrientation = false);

    void addOrientationControl(visualization_msgs::msg::InteractiveMarker& Marker, bool FixedOrientation = false);

    void addPositionControl(visualization_msgs::msg::InteractiveMarker& Marker, bool FixedOrientation = false);

    void addViewPlaneControl(visualization_msgs::msg::InteractiveMarker& Marker, double Radius,
        const std_msgs::msg::ColorRGBA& Color, bool PositionFreedom = true, bool OrientationFreedom = true);
}  // namespace whi_rviz_plugins
