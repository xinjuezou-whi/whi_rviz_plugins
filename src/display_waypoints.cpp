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

#include <pluginlib/class_list_macros.h>
#include <rviz/window_manager_interface.h>
#include <rviz/display_context.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/tf_frame_property.h>
#include <rviz/ogre_helpers/movable_text.h>
#include <rviz/frame_manager.h>
#include <visualization_msgs/Marker.h>

#include <sstream>

namespace whi_rviz_plugins
{
    WaypointsDisplay::WaypointsDisplay()
        : Display()
    {
        std::cout << "\nWHI RViz plugin for navigation waypoints VERSION 00.21.6" << std::endl;
        std::cout << "Copyright @ 2022-2024 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n" << std::endl;

        marker_size_property_ = new rviz::FloatProperty("Marker Size", 1.0, "Arrow size of waypoint mark.",
            this, SLOT(updateMarks()));
        marker_height_property_ = new rviz::FloatProperty("Marker Height", 1.0, "Height of waypoint mark for the accessability.",
            this, SLOT(updateMarks()));
        marker_color_property_ = new rviz::ColorProperty("Marker Color", QColor(0, 255, 0), "Color of waypoints arrow.",
            this, SLOT(updateMarks()));
        font_bool_property_ = new rviz::BoolProperty("Show ETA", true, "Toggle the visibility of ETA info.",
            this, SLOT(updateVisibility()));
        font_size_property_ = new rviz::FloatProperty("ETA Font Size", 1.0, "Characters size of ETA info.",
            this, SLOT(updateSize()));
        font_color_property_ = new rviz::ColorProperty("ETA Font Color", QColor(255, 255, 255), "Characters color of ETA info.",
            this, SLOT(updateColor()));
        QStringList sourceList = { "Local", "Remote" };
        mode_property_ = new rviz::EnumProperty("Mode", sourceList[0], "Options of running mode",
            this, SLOT(updateMode()));
        for (int i = 0; i < sourceList.size(); ++i)
        {
            mode_property_->addOption(sourceList[i], i);
        }
        frame_manager_ = std::make_shared<rviz::FrameManager>();
        frame_property_ = new rviz::TfFrameProperty("base_frame", "base_link", "Base link frame of robot",
            this, frame_manager_.get(), false, SLOT(updateBaselinkFrame()));
        stuck_timeout_property_ = new rviz::FloatProperty("Stuck timeout(s)", 10.0, "Timeout for break robot from stuck",
            this, SLOT(updateStuckTimeout()));
    }

    WaypointsDisplay::~WaypointsDisplay()
    {
        delete frame_dock_;
    }

    void WaypointsDisplay::onInitialize()
    {
        Display::onInitialize();

        panel_ = new WaypointsPanel(
            std::bind(&WaypointsDisplay::visualizeWaypointsLocations, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&WaypointsDisplay::visualEta, this, std::placeholders::_1, std::placeholders::_2));
        rviz::WindowManagerInterface* windowContext = context_->getWindowManager();
        if (windowContext)
        {
            frame_dock_ = windowContext->addPane("Navi_waypoints", panel_); // getName() return "" ???
            frame_dock_->setIcon(getIcon()); // set the image name as same as the name of plugin
        }

        eta_text_.reset(new rviz::MovableText("."));
        eta_text_->setLineSpacing(1.0);

        frame_node_ = getSceneNode()->createChildSceneNode();

        updateMarks();
        updateSize();
        updateColor();
        updateVisibility();
        updateMode();
        updateBaselinkFrame();
        updateStuckTimeout();
    }

    void WaypointsDisplay::clearWaypointsLocationsDisplay()
    {
        for (auto& it : waypoints_marker_)
        {
            it.reset();
        }
        waypoints_marker_.clear();
    }

    void WaypointsDisplay::visualizeWaypointsLocations(int InteractiveIndex, const std::vector<geometry_msgs::PoseStamped>& WaypointsPose)
    {
        clearWaypointsLocationsDisplay();
        waypoints_marker_.resize(WaypointsPose.size());

        for (std::size_t i = 0; i < WaypointsPose.size(); ++i)
        {
            visualization_msgs::Marker wayPointMarker;
            wayPointMarker.type = visualization_msgs::Marker::ARROW;
            wayPointMarker.action = visualization_msgs::Marker::ADD;
            wayPointMarker.scale.x = marker_size_property_->getFloat();
            wayPointMarker.scale.y = 0.2 * wayPointMarker.scale.x;
            wayPointMarker.scale.z = 0.2 * wayPointMarker.scale.x;
            Ogre::ColourValue color = marker_color_property_->getOgreColor();
            wayPointMarker.color.r = color.r;
            wayPointMarker.color.g = color.g;
            wayPointMarker.color.b = color.b;
            wayPointMarker.color.a = 1.0; // don't forget to set the alpha

            visualization_msgs::InteractiveMarkerControl controlMove3d;
            controlMove3d.always_visible = true;
            controlMove3d.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;
            controlMove3d.name = "move";
            controlMove3d.markers.push_back(wayPointMarker);

            visualization_msgs::InteractiveMarker imarker = i == InteractiveIndex ? make6DOFMarker("marker_scene_object", WaypointsPose[i], 2.1 * wayPointMarker.scale.x) :
                makeEmptyInteractiveMarker("marker_scene_object", WaypointsPose[i], wayPointMarker.scale.x);
            imarker.name = std::to_string(i + 1);
            imarker.description = imarker.name;
            imarker.controls.push_back(controlMove3d);
            interactive_markers::autoComplete(imarker);

            waypoints_marker_[i].reset(new rviz::InteractiveMarker(getSceneNode(), context_));
            waypoints_marker_[i]->processMessage(imarker);
            waypoints_marker_[i]->setShowAxes(false);

            // connect signals
            connect(waypoints_marker_[i].get(), SIGNAL(userFeedback(visualization_msgs::InteractiveMarkerFeedback&)), this,
                SLOT(interactiveMarkerProcessFeedback(visualization_msgs::InteractiveMarkerFeedback&)));
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

    void WaypointsDisplay::visualEta(const geometry_msgs::Pose& Pose, double Eta)
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

    void WaypointsDisplay::interactiveMarkerProcessFeedback(visualization_msgs::InteractiveMarkerFeedback& Feedback)
    {
        panel_->updateWaypoint(std::stoi(Feedback.marker_name) - 1, Feedback.pose);
    }

    void WaypointsDisplay::updateMarks()
    {
        panel_->updateHeight(marker_height_property_->getFloat());
    }

    void WaypointsDisplay::updateVisibility()
    {
        if (font_bool_property_->getBool())
        {
            frame_node_->attachObject(eta_text_.get());
        }
        else
        {
            frame_node_->detachObject(eta_text_.get());
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
        panel_->setBaselinkFrame(frame_property_->getFrame().toStdString());
    }

    void WaypointsDisplay::updateStuckTimeout()
    {
        panel_->setStuckTimeout(stuck_timeout_property_->getFloat());
    }

    void WaypointsDisplay::addPositionControl(visualization_msgs::InteractiveMarker& IntMarker, bool OrientationFixed)
    {
        visualization_msgs::InteractiveMarkerControl control;

        if (OrientationFixed)
        {
            control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
        }

        control.orientation.w = 1.0;
        control.orientation.x = 1.0;
        control.orientation.y = 0.0;
        control.orientation.z = 0.0;
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
        IntMarker.controls.push_back(control);

        control.orientation.w = 1.0;
        control.orientation.x = 0.0;
        control.orientation.y = 1.0;
        control.orientation.z = 0;
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
        IntMarker.controls.push_back(control);

        control.orientation.w = 1.0;
        control.orientation.x = 0.0;
        control.orientation.y = 0.0;
        control.orientation.z = 1.0;
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
        IntMarker.controls.push_back(control);
    }

    void WaypointsDisplay::addOrientationControl(visualization_msgs::InteractiveMarker& IntMarker, bool OrientationFixed)
    {
        visualization_msgs::InteractiveMarkerControl control;

        if (OrientationFixed)
        {
            control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
        }
        
        control.orientation.w = 1.0;
        control.orientation.x = 1.0;
        control.orientation.y = 0;
        control.orientation.z = 0;
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
        IntMarker.controls.push_back(control);

        control.orientation.w = 1.0;
        control.orientation.x = 0.0;
        control.orientation.y = 1.0;
        control.orientation.z = 0.0;
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
        IntMarker.controls.push_back(control);

        control.orientation.w = 1.0;
        control.orientation.x = 0.0;
        control.orientation.y = 0.0;
        control.orientation.z = 1.0;
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
        IntMarker.controls.push_back(control);
    }

    visualization_msgs::InteractiveMarker WaypointsDisplay::make6DOFMarker(const std::string& Name,
        const geometry_msgs::PoseStamped& Stamped, double Scale)
    {
        visualization_msgs::InteractiveMarker intMarker = makeEmptyInteractiveMarker(Name, Stamped, Scale);
        addPositionControl(intMarker, false);
        addOrientationControl(intMarker, false);

        return intMarker;
    }

    visualization_msgs::InteractiveMarker WaypointsDisplay::makeEmptyInteractiveMarker(const std::string& Name,
        const geometry_msgs::PoseStamped& Stamped, double Scale)
    {
        visualization_msgs::InteractiveMarker intMarker;
        intMarker.header = Stamped.header;
        intMarker.name = Name;
        intMarker.scale = Scale;
        intMarker.pose = Stamped.pose;

        return intMarker;
    }

    PLUGINLIB_EXPORT_CLASS(whi_rviz_plugins::WaypointsDisplay, rviz::Display)
} // end namespace whi_rviz_plugins
