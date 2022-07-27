/******************************************************************
rviz plugin for showing battery infomation

Features:
- power
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "display_battery.h"
//#include "imu_visual.h"

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <pluginlib/class_list_macros.h>

namespace whi_rviz_plugins
{
    DisplayBat::DisplayBat()
    {
        color_property_ = new rviz::ColorProperty("Color", QColor(204, 51, 204),
            "Color to draw the acceleration arrows.",
            this, SLOT(updateColorAndAlpha()));

        alpha_property_ = new rviz::FloatProperty("Alpha", 1.0,
            "0 is fully transparent, 1.0 is fully opaque.",
            this, SLOT(updateColorAndAlpha()));

        history_length_property_ = new rviz::IntProperty("History Length", 1,
            "Number of prior measurements to display.",
            this, SLOT(updateHistoryLength()));
        history_length_property_->setMin(1);
        history_length_property_->setMax(100000);
    }

    DisplayBat::~DisplayBat() {}

    // after the top-level rviz::Display::initialize() does its own setup,
    // it calls the subclass's onInitialize() function.  This is where we
    // instantiate all the workings of the class.  We make sure to also
    // call our immediate super-class's onInitialize() function, since it
    // does important stuff setting up the message filter
    //
    //  note that "MFDClass" is a typedef of
    // `MessageFilterDisplay<message type>`, to save typing that long
    // templated class name every time you need to refer to the superclass
    void DisplayBat::onInitialize()
    {
        MFDClass::onInitialize();
        updateHistoryLength();
    }

    // clear the visuals by deleting their objects
    void DisplayBat::reset()
    {
        MFDClass::reset();
        //visuals_.clear();
    }

    // set the current color and alpha values for each visual
    void DisplayBat::updateColorAndAlpha()
    {
        float alpha = alpha_property_->getFloat();
        Ogre::ColourValue color = color_property_->getOgreColor();

        //for( size_t i = 0; i < visuals_.size(); i++ )
        //{
        //  visuals_[ i ]->setColor( color.r, color.g, color.b, alpha );
        //}
    }

    // set the number of past visuals to show
    void DisplayBat::updateHistoryLength()
    {
        //visuals_.rset_capacity(history_length_property_->getInt());
    }

    // this is our callback to handle an incoming message
    void DisplayBat::processMessage(const sensor_msgs::Imu::ConstPtr& msg)
    {
        // here we call the rviz::FrameManager to get the transform from the
        // fixed frame to the frame in the header of this Imu message
        // If it fails, we can't do anything else so we return
        Ogre::Quaternion orientation;
        Ogre::Vector3 position;
        if (!context_->getFrameManager()->getTransform(msg->header.frame_id,
            msg->header.stamp,
            position, orientation))
        {
            ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
                msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
            return;
        }

        // we are keeping a circular buffer of visual pointers.  This gets
        // the next one, or creates and stores it if the buffer is not full
        /*boost::shared_ptr<ImuVisual> visual;
        if( visuals_.full() )
        {
          visual = visuals_.front();
        }
        else
        {
          visual.reset(new ImuVisual( context_->getSceneManager(), scene_node_ ));
        }

        // Now set or update the contents of the chosen visual.
        visual->setMessage( msg );
        visual->setFramePosition( position );
        visual->setFrameOrientation( orientation );

        float alpha = alpha_property_->getFloat();
        Ogre::ColourValue color = color_property_->getOgreColor();
        visual->setColor( color.r, color.g, color.b, alpha );

        // And send it to the end of the circular buffer
        visuals_.push_back(visual);*/
    }

    PLUGINLIB_EXPORT_CLASS(whi_rviz_plugins::DisplayBat, rviz::Display)

} // end namespace whi_rviz_plugins
