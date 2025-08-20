/******************************************************************
TF frames handler

Features:
- tf frame handler
- xxx

Refactored by Xinjue Zou, xinjue.zou@outlook.com
Origin auther: Willow Garage, Inc., Bosch Software Innovations GmbH.

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_rviz_plugins/frame_selection_handler.h"

#include <string>

#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/quaternion_property.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "rviz_common/properties/vector_property.hpp"

using rviz_common::interaction::Picked;
using rviz_common::interaction::SelectionHandler;
using rviz_common::properties::BoolProperty;
using rviz_common::properties::FloatProperty;
using rviz_common::properties::Property;
using rviz_common::properties::QuaternionProperty;
using rviz_common::properties::StatusProperty;
using rviz_common::properties::StringProperty;
using rviz_common::properties::VectorProperty;

namespace whi_rviz_plugins
{
	FrameSelectionHandler::FrameSelectionHandler(
		FrameInfo *frame,
		TFDisplay *display,
		rviz_common::DisplayContext *context)
		: SelectionHandler(context),
		  frame_(frame),
		  display_(display)
	{
	}

	void FrameSelectionHandler::createProperties(const Picked &obj, Property *parent_property)
	{
		(void)obj;
		(void)display_;
		category_property_ = new Property(
			"Frame " + QString::fromStdString(frame_->name_),
			QVariant(), "", parent_property);

		enabled_property_ =
			new BoolProperty(
				"Enabled", true, "", category_property_, SLOT(updateVisibilityFromSelection()), frame_);

		parent_property_ = new StringProperty("Parent", "", "", category_property_);
		parent_property_->setReadOnly(true);

		position_property_ = new VectorProperty("Position", Ogre::Vector3::ZERO, "", category_property_);
		position_property_->setReadOnly(true);

		orientation_property_ = new QuaternionProperty(
			"Orientation", Ogre::Quaternion::IDENTITY, "",
			category_property_);
		orientation_property_->setReadOnly(true);

		euler_property_ = new VectorProperty("Euler", Ogre::Vector3::ZERO, "", category_property_);
		euler_property_->setReadOnly(true);
	}

	void FrameSelectionHandler::destroyProperties(const Picked &obj, Property *parent_property)
	{
		(void)obj;
		(void)parent_property;
		delete category_property_; // This deletes its children as well.
		category_property_ = nullptr;
		enabled_property_ = nullptr;
		parent_property_ = nullptr;
		position_property_ = nullptr;
		orientation_property_ = nullptr;
		euler_property_ = nullptr;
	}

	bool FrameSelectionHandler::getEnabled()
	{
		if (enabled_property_)
		{
			return enabled_property_->getBool();
		}
		return false; // should never happen, but don't want to crash if it does.
	}

	void FrameSelectionHandler::setEnabled(bool enabled)
	{
		if (enabled_property_)
		{
			enabled_property_->setBool(enabled);
		}
	}

	void FrameSelectionHandler::setParentName(std::string parent_name)
	{
		if (parent_property_)
		{
			parent_property_->setStdString(parent_name);
		}
	}

	void FrameSelectionHandler::setPosition(const Ogre::Vector3 &position)
	{
		if (position_property_)
		{
			position_property_->setVector(position);
		}
	}

	void FrameSelectionHandler::setOrientation(const Ogre::Quaternion &orientation)
	{
		if (orientation_property_)
		{
			orientation_property_->setQuaternion(orientation);
		}
		if (euler_property_)
		{
			Ogre::Matrix3 mat;
			orientation.ToRotationMatrix(mat);
			Ogre::Radian yaw, pitch, roll;
			mat.ToEulerAnglesXYZ(yaw, pitch, roll);
			euler_property_->setVector(Ogre::Vector3(roll.valueRadians(), pitch.valueRadians(), yaw.valueRadians()));
		}
	}
} // namespace whi_rviz_plugins
