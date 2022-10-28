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

    private Q_SLOTS:
        void interactiveMarkerProcessFeedback(visualization_msgs::InteractiveMarkerFeedback& Feedback);
		// these Qt slots get connected to signals indicating changes in the user-editable properties
		void updateMarks();

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
        // user-editable property variables
        rviz::FloatProperty* size_property_;
        rviz::FloatProperty* height_property_;
        rviz::ColorProperty* color_property_;
    };
} // end namespace whi_rviz_plugins
