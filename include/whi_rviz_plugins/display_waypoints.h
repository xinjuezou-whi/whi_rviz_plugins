/******************************************************************
rviz display for navigation waypoints

Features:
- waypoints
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2022-10-27: Initial version
2025-08-07: Migrate from ROS 1
2025-xx-xx: xxx
******************************************************************/
#pragma once
#include "panel_waypoints.h"

#include <rviz_common/display.hpp>
#include <rviz_common/panel_dock_widget.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <rviz_default_plugins/displays/interactive_markers/interactive_marker.hpp>

#include <memory>

// forward declaration
namespace rviz_common
{
	namespace properties
    {
        class FloatProperty;
        class ColorProperty;
        class BoolProperty;
        class EnumProperty;
        class TfFrameProperty;
        class StringProperty;
        class IntProperty;
        class FrameManager;
        class MovableText;
        class RosTopicProperty;
    }
}

namespace rviz_rendering
{
    class MovableText;
}

namespace whi_rviz_plugins
{
    class WaypointsDisplay : public rviz_common::Display
    {
        Q_OBJECT
    public:
        WaypointsDisplay();
        virtual ~WaypointsDisplay();

    public:
        // overrides from Display
        virtual void onInitialize();
        void clearWaypointsLocationsDisplay();
        void visualizeWaypointsLocations(int InteractiveIndex,
            const std::vector<geometry_msgs::msg::PoseStamped>& WaypointsPose); 
        void visualEta(const geometry_msgs::msg::Pose& Pose, double Eta);

    private Q_SLOTS:
        void interactiveMarkerProcessFeedback(visualization_msgs::msg::InteractiveMarkerFeedback& Feedback);
		// these Qt slots get connected to signals indicating changes in the user-editable properties
		void updateUseStampedVel();
        void updateMarks();
        void updateVisibility();
        void updateSize();
        void updateColor();
        void updateMode();
        void updateBaselinkFrame();
        void updateStuckTimeout();
        void updateRecoveryMaxTryCount();
        void updateTolerance();
        void updateMotionStateTopic();
        void updateSwEstopTopic();
        void updateRcStateTopic();

    private:
		// dock and panel
		rviz_common::PanelDockWidget* frame_dock_{ nullptr };
        WaypointsPanel* panel_{ nullptr };

        std::vector<std::shared_ptr<rviz_default_plugins::displays::InteractiveMarker>> waypoint_markers_;
        std::shared_ptr<rviz_rendering::MovableText> eta_text_{ nullptr };
        // user-editable property variables
        rviz_common::properties::BoolProperty* use_stamped_vel_bool_property_;
        rviz_common::properties::FloatProperty* marker_size_property_;
        rviz_common::properties::FloatProperty* marker_height_property_;
        rviz_common::properties::ColorProperty* marker_color_property_;
        rviz_common::properties::BoolProperty* font_bool_property_;
        rviz_common::properties::FloatProperty* font_size_property_;
        rviz_common::properties::ColorProperty* font_color_property_;
        rviz_common::properties::EnumProperty* mode_property_;
        // rviz_common::properties::TfFrameProperty* frame_property_;
        rviz_common::properties::StringProperty* tmp_frame_property_;
        rviz_common::properties::FloatProperty* stuck_timeout_property_;
        rviz_common::properties::IntProperty* recovery_max_try_count_property_;
        rviz_common::properties::FloatProperty* xy_goal_tolerance_property_;
	    rviz_common::properties::FloatProperty* yaw_goal_tolerance_property_;
        rviz_common::properties::RosTopicProperty* motion_state_topic_property_;
        rviz_common::properties::RosTopicProperty* sw_estop_topic_property_;
        rviz_common::properties::RosTopicProperty* rc_state_topic_property_;
        Ogre::SceneNode* text_display_scene_node_{ nullptr };
        bool remote_mode_{ false };
    };
} // end namespace whi_rviz_plugins
