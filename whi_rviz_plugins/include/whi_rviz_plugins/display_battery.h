/******************************************************************
rviz plugin for showing battery infomation

Features:
- power
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2022-07-2: Initial version
2022-xx-xx: xxx
******************************************************************/
#pragma once

#ifndef Q_MOC_RUN
#include <boost/circular_buffer.hpp>

#include <rviz/message_filter_display.h>
#include <sensor_msgs/Imu.h>
#endif

namespace Ogre
{
	class SceneNode;
}

namespace rviz
{
	class ColorProperty;
	class FloatProperty;
	class IntProperty;
}

namespace whi_rviz_plugins
{
	//class ImuVisual;

	class DisplayBat : public rviz::MessageFilterDisplay<sensor_msgs::Imu>
	{
		Q_OBJECT
	public:
		// pluginlib::ClassLoader creates instances by calling
		// the default constructor, so make sure you have one
		DisplayBat();
		virtual ~DisplayBat();

		// overrides of protected virtual functions from Display.  As much
		// as possible, when Displays are not enabled, they should not be
		// subscribed to incoming data and should not show anything in the
		// 3D view.  These functions are where these connections are made
		// and broken.
	protected:
		virtual void onInitialize();

		// helper to clear this display back to the initial state
		virtual void reset();

		// these Qt slots get connected to signals indicating changes in the user-editable properties
	private Q_SLOTS:
		void updateColorAndAlpha();
		void updateHistoryLength();

		// function to handle an incoming ROS message
	private:
		void processMessage(const sensor_msgs::Imu::ConstPtr& msg);

		// storage for the list of visuals.  It is a circular buffer where
		// data gets popped from the front (oldest) and pushed to the back (newest)
		//boost::circular_buffer<boost::shared_ptr<ImuVisual> > visuals_;

		// user-editable property variables
		rviz::ColorProperty* color_property_;
		rviz::FloatProperty* alpha_property_;
		rviz::IntProperty* history_length_property_;
	};
} // end namespace whi_rviz_plugins
