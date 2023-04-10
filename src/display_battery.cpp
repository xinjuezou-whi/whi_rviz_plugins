/******************************************************************
rviz plugin for showing battery infomation

Features:
- power
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_rviz_plugins/display_battery.h"
#include "whi_rviz_plugins/battery_visual.h"

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/vector_property.h>
#include <pluginlib/class_list_macros.h>

namespace whi_rviz_plugins
{
    DisplayBat::DisplayBat()
    {
        std::cout << "\nWHI RViz plugin for battery VERSION 00.06" << std::endl;
        std::cout << "Copyright © 2022-2024 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n" << std::endl;

        color_red_ = std::make_shared<Ogre::ColourValue>(239.0 / 255.0, 41.0 / 255.0, 41.0 / 255.0);
        color_property_ = new rviz::ColorProperty("Color", QColor(138, 226, 52),
            "Color of battery info text.",
            this, SLOT(updateColorAndAlpha()));

        alpha_property_ = new rviz::FloatProperty("Alpha", 1.0,
            "0 is fully transparent, 1.0 is fully opaque.",
            this, SLOT(updateColorAndAlpha()));

        history_length_property_ = new rviz::IntProperty("History Length", 1,
            "Number of prior measurements to display.",
            this, SLOT(updateHistoryLength()));
        history_length_property_->setMin(1);
        history_length_property_->setMax(100000);

        size_property_ = new rviz::FloatProperty("Size", 1.0,
            "Character size of battery info text.",
            this, SLOT(updateSize()));

        offsets_property_ = new rviz::VectorProperty("Offsets", Ogre::Vector3::ZERO,
            "Offsets to frame",
            this, SLOT(updateOffsets()));

        orientation_property_ = new rviz::VectorProperty("Orientation", Ogre::Vector3(float(0.0), float(90.0), float(0.0)),
            "Orientation of battery symbol",
            this, SLOT(updateOrientation()));
    }

    DisplayBat::~DisplayBat() {}

    // after the top-level rviz::Display::initialize() does its own setup,
    // it calls the subclass's onInitialize() function
    // this is where all the workings of the class is instantiated
    // make sure to also call the immediate super-class's onInitialize() function,
    // since it does important stuff setting up the message filter
    //
    // note that "MFDClass" is a typedef of `MessageFilterDisplay<message type>`,
    // to save typing that long templated class name every time the superclass needs to be refered
    void DisplayBat::onInitialize()
    {
        MFDClass::onInitialize();
        updateColorAndAlpha();
        updateHistoryLength();
        updateSize();
        updateOffsets();
        updateOrientation();
    }

    // clear the visuals by deleting their objects
    void DisplayBat::reset()
    {
        MFDClass::reset();
        visuals_.clear();
    }

    // set the current color and alpha values for each visual
    void DisplayBat::updateColorAndAlpha()
    {
        float alpha = alpha_property_->getFloat();
        Ogre::ColourValue color = color_property_->getOgreColor();

        for (size_t i = 0; i < visuals_.size(); ++i)
        {
            visuals_[i]->setColor(color.r, color.g, color.b, alpha);
        }
    }

    // set the number of past visuals to show
    void DisplayBat::updateHistoryLength()
    {
        visuals_.rset_capacity(history_length_property_->getInt());
    }

    void DisplayBat::updateSize()
    {
        for (size_t i = 0; i < visuals_.size(); ++i)
        {
            visuals_[i]->setSize(size_property_->getFloat());
        }
    }

    void DisplayBat::updateOffsets()
    {
        for (size_t i = 0; i < visuals_.size(); ++i)
        {
            visuals_[i]->setOffsets(offsets_property_->getVector());
        }
    }

    void DisplayBat::updateOrientation()
    {
        for (size_t i = 0; i < visuals_.size(); ++i)
        {
            visuals_[i]->setOrientation(orientation_property_->getVector());
        }
    }

    void DisplayBat::processMessage(const whi_interfaces::WhiBattery::ConstPtr& Msg)
    {
        // call the rviz::FrameManager to get the transform from the fixed frame to the frame in the header of this battery message
        // if it fails, do nothing and return
        Ogre::Quaternion orientation;
        Ogre::Vector3 position;
        if (!context_->getFrameManager()->getTransform(Msg->header.frame_id,
            Msg->header.stamp, position, orientation))
        {
            ROS_DEBUG("error transforming from frame '%s' to frame '%s'",
                Msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
            return;
        }
#ifdef DEBUG
        std::cout << "x " << position.x << " y " << position.y << " z " << position.z << std::endl;

        Ogre::Matrix3 mat;
        orientation.ToRotationMatrix(mat);
        Ogre::Radian yaw, pitch, roll;
        mat.ToEulerAnglesXYZ(yaw, pitch, roll);
        std::cout << "yaw " << yaw.valueDegrees() << " pitch " << pitch.valueDegrees() << " roll " << roll.valueDegrees() << std::endl;
#endif

        // keeping a circular buffer of visual pointers
        // this gets the next one, or creates and stores it if the buffer is not full
        std::shared_ptr<BatteryVisual> visual;
        if (visuals_.full())
        {
            visual = visuals_.front();
        }
        else
        {
            visual.reset(new BatteryVisual(context_->getSceneManager(), scene_node_));
        }

        // set or update the contents of the chosen visual
        float alpha = alpha_property_->getFloat();
        Ogre::ColourValue color = Msg->state == whi_interfaces::WhiBattery::STA_NEED_CHARGING ?
            *color_red_ : color_property_->getOgreColor();
        visual->setColor(color.r, color.g, color.b, alpha);
        visual->createBatteryShape(Msg->soc / 100.0, Msg->state == whi_interfaces::WhiBattery::STA_NEED_CHARGING ?
            *color_red_ : Ogre::ColourValue(0.0, 1.0, 0.0, 0.8));
        visual->setMessage(Msg);
        visual->setFramePosition(position);
        visual->setFrameOrientation(orientation);

        // send it to the end of the circular buffer
        visuals_.push_back(visual);
    }

    PLUGINLIB_EXPORT_CLASS(whi_rviz_plugins::DisplayBat, rviz::Display)
} // end namespace whi_rviz_plugins
