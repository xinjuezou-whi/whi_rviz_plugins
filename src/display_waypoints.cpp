/******************************************************************
rviz display for navigation waypoints

Features:
- waypoints
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_rviz_plugins/display_waypoints.h"
#include "whi_rviz_plugins/interaction_markers_factory.h"

#include <rviz_common/window_manager_interface.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/tf_frame_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_rendering/objects/movable_text.hpp>
#include <interactive_markers/tools.hpp>

#include <OgreSceneNode.h>

#include <sstream>

namespace whi_rviz_plugins
{
    WaypointsDisplay::WaypointsDisplay()
        : Display()
    {
        std::cout << "\nWHI RViz plugin for navigation waypoints VERSION 02.30.6" << std::endl;
        std::cout << "Copyright @ 2022-2026 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n" << std::endl;

        use_stamped_vel_bool_property_ = new rviz_common::properties::BoolProperty("Whether to use stamped twist", true,
            "Toggle the twist with or without stamp.", this, SLOT(updateUseStampedVel()));
        marker_size_property_ = new rviz_common::properties::FloatProperty("Marker Size", 1.0,
            "Arrow size of waypoint mark.", this, SLOT(updateMarks()));
        marker_height_property_ = new rviz_common::properties::FloatProperty("Marker Height", 1.0,
            "Height of waypoint mark for the accessability.", this, SLOT(updateMarks()));
        marker_color_property_ = new rviz_common::properties::ColorProperty("Marker Color", QColor(0, 255, 0),
            "Color of waypoints arrow.", this, SLOT(updateMarks()));
        font_bool_property_ = new rviz_common::properties::BoolProperty("Show ETA", true,
            "Toggle the visibility of ETA info.", this, SLOT(updateVisibility()));
        font_size_property_ = new rviz_common::properties::FloatProperty("ETA Font Size", 1.0,
            "Characters size of ETA info.", this, SLOT(updateSize()));
        font_color_property_ = new rviz_common::properties::ColorProperty("ETA Font Color", QColor(255, 255, 255),
            "Characters color of ETA info.", this, SLOT(updateColor()));
        QStringList sourceList = { "Local", "Remote" };
        mode_property_ = new rviz_common::properties::EnumProperty("Mode", sourceList[0], "Options of running mode",
            this, SLOT(updateMode()));
        for (int i = 0; i < sourceList.size(); ++i)
        {
            mode_property_->addOption(sourceList[i], i);
        }
        stuck_timeout_property_ = new rviz_common::properties::FloatProperty("Stuck timeout(s)", 10.0,
            "Timeout for break robot from stuck", this, SLOT(updateStuckTimeout()));
        recovery_max_try_count_property_ = new rviz_common::properties::IntProperty("Max recovery try count", 3,
            "Max times to try recovery.", this, SLOT(updateRecoveryMaxTryCount()));
        xy_goal_tolerance_property_ = new rviz_common::properties::FloatProperty("Goal xy tolerance", 0.15,
            "x and y tolerance of goal", this, SLOT(updateTolerance()));
        yaw_goal_tolerance_property_ = new rviz_common::properties::FloatProperty("Goal yaw tolerance", 0.15,
            "yaw tolerance of goal", this, SLOT(updateTolerance()));
        motion_state_topic_property_ = new rviz_common::properties::RosTopicProperty("Motion state topic", "motion_state",
            "whi_interfaces/msg/WhiMotionState", "Topic of motion state", this);
        sw_estop_topic_property_ = new rviz_common::properties::RosTopicProperty("Software EStop topic", "estop",
            "std_msgs/msg/Bool", "Topic of software EStop", this);
        rc_state_topic_property_ = new rviz_common::properties::RosTopicProperty("Remote controller state topic", "rc_state",
            "whi_interfaces/msg/WhiRcState", "Topic of remote controller state", this);
        frame_property_ = new rviz_common::properties::TfFrameProperty("base_frame", "base_link", "Base link frame of robot",
            this, nullptr, false, SLOT(updateBaselinkFrame()));
    }

    WaypointsDisplay::~WaypointsDisplay()
    {
        delete frame_dock_;
    }

    void WaypointsDisplay::onInitialize()
    {
        Display::onInitialize();

        // Access the abstract ROS Node and
        // in the process lock it for exclusive use until the method is done.
        // Get a pointer to the familiar rclcpp::Node for making subscriptions/publishers
        // (as per normal rclcpp code)
        node_handle_ = context_->getRosNodeAbstraction().lock()->get_raw_node();

        motion_state_topic_property_->initialize(context_->getRosNodeAbstraction());
        connect(motion_state_topic_property_, &rviz_common::properties::RosTopicProperty::changed, this, [&]()
        {
            if (initialized())
            {
                panel_->setMotionStateTopic(motion_state_topic_property_->getTopicStd());
            }
        });
        sw_estop_topic_property_->initialize(context_->getRosNodeAbstraction());
        connect(sw_estop_topic_property_, &rviz_common::properties::RosTopicProperty::changed, this, [&]()
        {
            if (initialized())
            {
                panel_->setSwEstopTopic(sw_estop_topic_property_->getTopicStd());
            }
        });
        rc_state_topic_property_->initialize(context_->getRosNodeAbstraction());
        connect(rc_state_topic_property_, &rviz_common::properties::RosTopicProperty::changed, this, [&]()
        {
            if (initialized())
            {
                panel_->setRcStateTopic(rc_state_topic_property_->getTopicStd());
            }
        });

        frame_property_->setFrameManager(context_->getFrameManager());

        rviz_common::WindowManagerInterface* windowContext = context_->getWindowManager();
        if (windowContext)
        {
            panel_ = new WaypointsPanel(node_handle_,
                std::bind(&WaypointsDisplay::visualizeWaypointsLocations, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&WaypointsDisplay::visualEta, this, std::placeholders::_1, std::placeholders::_2)/*,
                ((rviz::VisualizationFrame*)windowContext)->getManager()*/);
            frame_dock_ = windowContext->addPane("Navi_waypoints", panel_); // getName() return "" ???
            frame_dock_->setIcon(getIcon()); // set the image name as same as the name of plugin
        }
        else
        {
            delete frame_dock_;
            frame_dock_ = nullptr;
            return;
        }

        eta_text_.reset(new rviz_rendering::MovableText("."));
        eta_text_->setLineSpacing(1.0);

        // the scene node that contains everything
        text_display_scene_node_ = scene_node_->createChildSceneNode();

        updateUseStampedVel();
        updateMarks();
        updateSize();
        updateColor();
        updateVisibility();
        updateMode();
        updateBaselinkFrame();
        updateStuckTimeout();
        updateRecoveryMaxTryCount();
        updateTolerance();
    }

    void WaypointsDisplay::clearWaypointsLocationsDisplay()
    {
        for (auto& it : waypoint_markers_)
        {
            it.reset();
        }
        waypoint_markers_.clear();
    }

    void WaypointsDisplay::visualizeWaypointsLocations(int InteractiveIndex,
        const std::vector<geometry_msgs::msg::PoseStamped>& WaypointsPose)
    {
        clearWaypointsLocationsDisplay();
        waypoint_markers_.resize(WaypointsPose.size());

        for (std::size_t i = 0; i < WaypointsPose.size(); ++i)
        {
            visualization_msgs::msg::Marker wayPointMarker;
            wayPointMarker.type = visualization_msgs::msg::Marker::ARROW;
            wayPointMarker.action = visualization_msgs::msg::Marker::ADD;
            wayPointMarker.scale.x = marker_size_property_->getFloat();
            wayPointMarker.scale.y = 0.2 * wayPointMarker.scale.x;
            wayPointMarker.scale.z = 0.2 * wayPointMarker.scale.x;
            Ogre::ColourValue color = marker_color_property_->getOgreColor();
            wayPointMarker.color.r = color.r;
            wayPointMarker.color.g = color.g;
            wayPointMarker.color.b = color.b;
            wayPointMarker.color.a = 1.0; // don't forget to set the alpha

            visualization_msgs::msg::InteractiveMarkerControl controlMove3d;
            controlMove3d.always_visible = true;
            controlMove3d.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE_3D;
            controlMove3d.name = "move";
            controlMove3d.markers.push_back(wayPointMarker);

            visualization_msgs::msg::InteractiveMarker imarker = i == InteractiveIndex ?
                make6DOFMarker("marker_scene_object", WaypointsPose[i], 2.1 * wayPointMarker.scale.x) :
                makeEmptyInteractiveMarker("marker_scene_object", WaypointsPose[i], wayPointMarker.scale.x);
            imarker.name = std::to_string(i + 1);
            imarker.description = imarker.name;
            imarker.controls.push_back(controlMove3d);
            interactive_markers::autoComplete(imarker);

            waypoint_markers_[i].reset(new rviz_default_plugins::displays::InteractiveMarker(getSceneNode(), context_));
            waypoint_markers_[i]->processMessage(imarker);
            waypoint_markers_[i]->setShowAxes(false);

            // connect signals
            connect(waypoint_markers_[i].get(), SIGNAL(userFeedback(visualization_msgs::msg::InteractiveMarkerFeedback&)),
                this, SLOT(interactiveMarkerProcessFeedback(visualization_msgs::msg::InteractiveMarkerFeedback&)));
        }
    }

    template <typename T>
    std::string toStringWithPrecision(const T Value, const int Digits = 6)
    {
        std::ostringstream out;
        out.precision(Digits);
        out << std::fixed << Value;
        return out.str();
    }

    void WaypointsDisplay::visualEta(const geometry_msgs::msg::Pose& Pose, double Eta)
    {
        std::string info(".");
        if (font_bool_property_->getBool())
        {
            if (Eta > 0.0)
            {
                info = "ETA in " + toStringWithPrecision(Eta, 2) + "s";
            }
            else if (fabs(Eta + 1.0) < 1e-5)
            {
                info = "Arrived";
            }
            else if (fabs(Eta + 2.0) < 1e-5)
            {
                info = "Aborted";
            }
            else
            {
                info = ".";
            }
        }

        eta_text_->setCaption(info);
        eta_text_->setCharacterHeight(font_size_property_->getFloat());
        eta_text_->setColor(font_color_property_->getOgreColor());
        eta_text_->setGlobalTranslation(Ogre::Vector3(Pose.position.x, Pose.position.y, 0.0));
    }

    void WaypointsDisplay::interactiveMarkerProcessFeedback(visualization_msgs::msg::InteractiveMarkerFeedback& Feedback)
    {
        panel_->updateWaypoint(std::stoi(Feedback.marker_name) - 1, Feedback.pose);
    }

    void WaypointsDisplay::updateUseStampedVel()
    {
        panel_->useStampedVel(use_stamped_vel_bool_property_->getBool());
    }

    void WaypointsDisplay::updateMarks()
    {
        panel_->updateHeight(marker_height_property_->getFloat());
    }

    void WaypointsDisplay::updateVisibility()
    {
        if (font_bool_property_->getBool())
        {
            text_display_scene_node_->attachObject(eta_text_.get());
        }
        else
        {
            text_display_scene_node_->detachObject(eta_text_.get());
        }
    }

	void WaypointsDisplay::updateSize()
	{
		eta_text_->setCharacterHeight(font_size_property_->getFloat());
	}

    void WaypointsDisplay::updateColor()
	{
		eta_text_->setColor(font_color_property_->getOgreColor());
	}

    void WaypointsDisplay::updateMode()
    {
        remote_mode_ = mode_property_->getOptionInt() == 0 ? false : true;
        panel_->setRemoteFlag(remote_mode_);
    }

    void WaypointsDisplay::updateBaselinkFrame()
    {
        panel_->setBaselinkFrame(frame_property_->getFrameStd());
    }

    void WaypointsDisplay::updateStuckTimeout()
    {
        panel_->setStuckTimeout(stuck_timeout_property_->getFloat());
    }

    void WaypointsDisplay::updateRecoveryMaxTryCount()
    {
        panel_->setRecoveryMaxTryCount(recovery_max_try_count_property_->getInt());
    }

    void WaypointsDisplay::updateTolerance()
    {
        panel_->setTolerance(xy_goal_tolerance_property_->getFloat(),
            yaw_goal_tolerance_property_->getFloat());
    }
} // end namespace whi_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(whi_rviz_plugins::WaypointsDisplay, rviz_common::Display)
