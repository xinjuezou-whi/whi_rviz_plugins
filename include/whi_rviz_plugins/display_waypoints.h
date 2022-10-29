/******************************************************************
rviz display for navigation waypoints

Features:
- waypoints
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2022-10-27: Initial version
2022-xx-xx: xxx
******************************************************************/
#pragma once
#include <rviz/display.h>
#include <rviz/panel_dock_widget.h>
#include <geometry_msgs/PoseStamped.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/tools.h>
#include <rviz/default_plugin/interactive_markers/interactive_marker.h>
#include <memory>

#include "panel_waypoints.h"

// forward declaration
namespace rviz
{
	class FloatProperty;
    class ColorProperty;
    class BoolProperty;
    class MovableText;
}

namespace whi_rviz_plugins
{
    class WaypointsDisplay : public rviz::Display 
    {
        Q_OBJECT
    public:
        WaypointsDisplay();
        ~WaypointsDisplay() override;

    public:
        // overrides from Display
        void onInitialize() override;
        void clearWaypointsLocationsDisplay();
        void visualizeWaypointsLocations(int InteractiveIndex, const std::vector<geometry_msgs::PoseStamped>& WaypointsPose); 
        void visualEta(const geometry_msgs::Pose& Pose, double Eta);       

    private Q_SLOTS:
        void interactiveMarkerProcessFeedback(visualization_msgs::InteractiveMarkerFeedback& Feedback);
		// these Qt slots get connected to signals indicating changes in the user-editable properties
		void updateMarks();
        void setVisibility(bool Visible);
        void setSize(float Size);
        void setColor(const Ogre::ColourValue& Color);

    private:
        static void addPositionControl(visualization_msgs::InteractiveMarker& IntMarker, bool OrientationFixed);
        static void addOrientationControl(visualization_msgs::InteractiveMarker& IntMarker, bool OrientationFixed);
        static visualization_msgs::InteractiveMarker makeEmptyInteractiveMarker(const std::string& Name,
            const geometry_msgs::PoseStamped& Stamped, double Scale);
        static visualization_msgs::InteractiveMarker make6DOFMarker(const std::string& Name,
            const geometry_msgs::PoseStamped& Stamped, double Scale);

    private:
        rviz::PanelDockWidget* frame_dock_{ nullptr };
        WaypointsPanel* panel_{ nullptr };
        std::vector<std::shared_ptr<rviz::InteractiveMarker>> waypoints_marker_;
        std::shared_ptr<rviz::MovableText> eta_text_{ nullptr };
        // user-editable property variables
        rviz::FloatProperty* marker_size_property_;
        rviz::FloatProperty* marker_height_property_;
        rviz::ColorProperty* marker_color_property_;
        rviz::BoolProperty* font_bool_property_;
        rviz::FloatProperty* font_size_property_;
        rviz::ColorProperty* font_color_property_;
        Ogre::SceneNode* frame_node_{ nullptr };
    };
} // end namespace whi_rviz_plugins
