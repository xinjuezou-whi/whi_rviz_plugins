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
#include "whi_rviz_plugins/interaction_markers_factory.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <boost/math/constants/constants.hpp>

namespace whi_rviz_plugins
{
    visualization_msgs::msg::InteractiveMarker makeEmptyInteractiveMarker(const std::string& Name,
        const geometry_msgs::msg::PoseStamped& Pose, double Scale)
    {
        visualization_msgs::msg::InteractiveMarker marker;
        marker.header = Pose.header;
        marker.name = Name;
        marker.scale = Scale;
        marker.pose = Pose.pose;

        return marker;
    }

    visualization_msgs::msg::InteractiveMarker make6DOFMarker(const std::string& Name,
        const geometry_msgs::msg::PoseStamped& Pose, double Scale, bool FixedOrientation/* = false*/)
    {
        visualization_msgs::msg::InteractiveMarker marker = makeEmptyInteractiveMarker(Name, Pose, Scale);
        add6DOFControl(marker, FixedOrientation);

        return marker;
    }

    visualization_msgs::msg::InteractiveMarker makePlanarXYMarker(const std::string& Name,
        const geometry_msgs::msg::PoseStamped& Pose, double Scale, bool FixedOrientation/* = false*/)
    {
        visualization_msgs::msg::InteractiveMarker marker = makeEmptyInteractiveMarker(Name, Pose, Scale);
        addPlanarXYControl(marker, FixedOrientation);

        return marker;
    }

    void addTArrowMarker(visualization_msgs::msg::InteractiveMarker& Marker)
    {
        // create an arrow marker
        visualization_msgs::msg::Marker m;
        m.type = visualization_msgs::msg::Marker::ARROW;
        m.scale.x = 0.6 * Marker.scale;
        m.scale.y = 0.12 * Marker.scale;
        m.scale.z = 0.12 * Marker.scale;
        m.ns = "goal_pose_arrow_marker";
        m.id = 1;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.header = Marker.header;
        m.pose = Marker.pose;
        // Arrow points along Z
        tf2::Quaternion imq, tmq;
        tf2::fromMsg(m.pose.orientation, imq);
        tmq.setRPY(0, -boost::math::constants::pi<double>() / 2.0, 0);
        imq = imq * tmq;
        m.pose.orientation = tf2::toMsg(imq);
        m.color.r = 0.0f;
        m.color.g = 1.0f;
        m.color.b = 0.0f;
        m.color.a = 1.0f;

        visualization_msgs::msg::Marker mc;
        mc.type = visualization_msgs::msg::Marker::CYLINDER;
        mc.scale.x = 0.05 * Marker.scale;
        mc.scale.y = 0.05 * Marker.scale;
        mc.scale.z = 0.15 * Marker.scale;
        mc.ns = "goal_pose_arrow_marker";
        mc.id = 2;
        mc.action = visualization_msgs::msg::Marker::ADD;
        mc.header = Marker.header;
        mc.pose = Marker.pose;
        // Cylinder points along Y
        tf2::fromMsg(mc.pose.orientation, imq);
        tmq.setRPY(boost::math::constants::pi<double>() / 2.0, 0, 0);
        imq = imq * tmq;
        mc.pose.orientation = tf2::toMsg(imq);
        mc.pose.position.x -= 0.04;
        mc.pose.position.z += 0.01;
        mc.color.r = 0.0f;
        mc.color.g = 1.0f;
        mc.color.b = 0.0f;
        mc.color.a = 1.0f;

        visualization_msgs::msg::InteractiveMarkerControl control;
        control.always_visible = true;
        control.interaction_mode = control.BUTTON;
        control.markers.push_back(m);
        control.markers.push_back(mc);

        // add the control to the interactive marker
        Marker.controls.push_back(control);
    }

    void addErrorMarker(visualization_msgs::msg::InteractiveMarker& Marker)
    {
        // create a grey box marker
        visualization_msgs::msg::Marker err;
        err.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
        err.scale.x = 0.002 * Marker.scale;
        err.scale.y = 0.002 * Marker.scale;
        err.scale.z = 0.002 * Marker.scale;
        err.mesh_resource = "package://moveit_ros_planning_interface/resources/access-denied.dae";
        err.ns = "robot_interaction_error";
        err.id = 1;
        err.action = visualization_msgs::msg::Marker::ADD;
        err.header = Marker.header;
        err.pose = Marker.pose;
        err.pose.orientation.x = err.pose.orientation.y = 0.7071067811865476;
        err.pose.orientation.z = err.pose.orientation.w = 0.0;
        err.color.r = 1.0f;
        err.color.g = 0.0f;
        err.color.b = 0.0f;
        err.color.a = 1.0f;

        visualization_msgs::msg::InteractiveMarkerControl control;
        control.always_visible = false;
        control.markers.push_back(err);

        // add the control to the interactive marker
        Marker.controls.push_back(control);
    }

    void add6DOFControl(visualization_msgs::msg::InteractiveMarker& Marker, bool FixedOrientation/* = false*/)
    {
        addOrientationControl(Marker, FixedOrientation);
        addPositionControl(Marker, FixedOrientation);
    }

    // value for normalized quaternion: 1.0 / std::sqrt(2.0)
    static const double SQRT2INV = 0.707106781;

    void addPlanarXYControl(visualization_msgs::msg::InteractiveMarker& Marker, bool FixedOrientation/* = false*/)
    {
        visualization_msgs::msg::InteractiveMarkerControl control;

        if (FixedOrientation)
        {
            control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::FIXED;
        }
        control.orientation.w = SQRT2INV;
        control.orientation.x = SQRT2INV;
        control.orientation.y = 0;
        control.orientation.z = 0;
        control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
        Marker.controls.push_back(control);

        control.orientation.w = SQRT2INV;
        control.orientation.x = 0;
        control.orientation.y = SQRT2INV;
        control.orientation.z = 0;
        control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
        Marker.controls.push_back(control);

        control.orientation.w = SQRT2INV;
        control.orientation.x = 0;
        control.orientation.y = 0;
        control.orientation.z = SQRT2INV;
        control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
        Marker.controls.push_back(control);
    }

    void addOrientationControl(visualization_msgs::msg::InteractiveMarker& Marker, bool FixedOrientation/* = false*/)
    {
        visualization_msgs::msg::InteractiveMarkerControl control;

        if (FixedOrientation)
        {
            control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::FIXED;
        }
        control.orientation.w = SQRT2INV;
        control.orientation.x = SQRT2INV;
        control.orientation.y = 0;
        control.orientation.z = 0;
        control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
        Marker.controls.push_back(control);

        control.orientation.w = SQRT2INV;
        control.orientation.x = 0;
        control.orientation.y = SQRT2INV;
        control.orientation.z = 0;
        control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
        Marker.controls.push_back(control);

        control.orientation.w = SQRT2INV;
        control.orientation.x = 0;
        control.orientation.y = 0;
        control.orientation.z = SQRT2INV;
        control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
        Marker.controls.push_back(control);
    }

    void addPositionControl(visualization_msgs::msg::InteractiveMarker& Marker, bool FixedOrientation/* = false*/)
    {
        visualization_msgs::msg::InteractiveMarkerControl control;

        if (FixedOrientation)
        {
            control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::FIXED;
        }
        control.orientation.w = SQRT2INV;
        control.orientation.x = SQRT2INV;
        control.orientation.y = 0;
        control.orientation.z = 0;
        control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
        Marker.controls.push_back(control);

        control.orientation.w = SQRT2INV;
        control.orientation.x = 0;
        control.orientation.y = SQRT2INV;
        control.orientation.z = 0;
        control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
        Marker.controls.push_back(control);

        control.orientation.w = SQRT2INV;
        control.orientation.x = 0;
        control.orientation.y = 0;
        control.orientation.z = SQRT2INV;
        control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
        Marker.controls.push_back(control);
    }

    void addViewPlaneControl(visualization_msgs::msg::InteractiveMarker& Marker, double Radius,
        const std_msgs::msg::ColorRGBA& Color, bool PositionFreedom/* = true*/, bool OrientationFreedom/* = true*/)
    {
        visualization_msgs::msg::InteractiveMarkerControl control;
        control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::VIEW_FACING;
        if (PositionFreedom && OrientationFreedom)
        {
            control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE_3D;
        }
        else if (OrientationFreedom)
        {
            control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_3D;
        }
        else
        {
            control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_3D;
        }
        control.independent_marker_orientation = true;
        control.name = "move";

        visualization_msgs::msg::Marker marker;

        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.scale.x = Radius * 2.0;
        marker.scale.y = Radius * 2.0;
        marker.scale.z = Radius * 2.0;
        marker.color = Color;

        control.markers.push_back(marker);
        control.always_visible = false;

        Marker.controls.push_back(control);
    }
}  // namespace robot_interaction
