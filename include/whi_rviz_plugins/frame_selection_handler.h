/******************************************************************
TF frames handler

Features:
- tf frame handler
- xxx

Refactored by Xinjue Zou, xinjue.zou@outlook.com
Origin auther: Willow Garage, Inc., Bosch Software Innovations GmbH.

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2025-08-20: Initial version
2025-xx-xx: xxx
******************************************************************/
#pragma once

#include <memory>
#include <string>

#include "rviz_common/interaction/selection_handler.hpp"
#include "display_tf.h"
#include "frame_info.h"
#include "visibility_control.hpp"

namespace rviz_common
{
	namespace properties
	{
		class Property;
		class BoolProperty;
		class StringProperty;
		class VectorProperty;
		class QuaternionProperty;
	} // namespace properties
} // namespace rviz_common

namespace whi_rviz_plugins
{
	class WHI_RVIZ_PLUGINS_PUBLIC FrameSelectionHandler : public rviz_common::interaction::SelectionHandler
	{
	public:
		~FrameSelectionHandler() override = default;

		void createProperties(
			const rviz_common::interaction::Picked &obj,
			rviz_common::properties::Property *parent_property) override;

		void destroyProperties(
			const rviz_common::interaction::Picked &obj,
			rviz_common::properties::Property *parent_property) override;

		bool getEnabled();

		void setEnabled(bool enabled);

		void setParentName(std::string parent_name);

		void setPosition(const Ogre::Vector3 &position);

		void setOrientation(const Ogre::Quaternion &orientation);

	private:
		FrameSelectionHandler(
			FrameInfo *frame,
			TFDisplay *display,
			rviz_common::DisplayContext *context);

		FrameInfo *frame_;
		TFDisplay *display_;
		rviz_common::properties::Property *category_property_{ nullptr };
		rviz_common::properties::BoolProperty *enabled_property_{ nullptr };
		rviz_common::properties::StringProperty *parent_property_{ nullptr };
		rviz_common::properties::VectorProperty *position_property_{ nullptr };
		rviz_common::properties::QuaternionProperty *orientation_property_{ nullptr };
		rviz_common::properties::VectorProperty* euler_property_{ nullptr };

		template <typename T, typename... Args>
		friend typename std::shared_ptr<T>
		rviz_common::interaction::createSelectionHandler(Args... arguments);
	};
} // namespace whi_rviz_plugins
