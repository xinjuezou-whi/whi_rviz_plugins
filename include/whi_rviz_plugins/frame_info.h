/******************************************************************
TF frames info

Features:
- tf info
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
#include <string>

#include "tf2/time.h"

#include "display_tf.h"
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
	/** @brief Internal class needed only by TFDisplay. */
	class WHI_RVIZ_PLUGINS_PUBLIC FrameInfo : public QObject
	{
		Q_OBJECT

	public:
		explicit FrameInfo(TFDisplay *display);

		static const Ogre::ColourValue ARROW_HEAD_COLOR;
		static const Ogre::ColourValue ARROW_SHAFT_COLOR;

		/** @brief Set this frame to be visible or invisible. */
		void setEnabled(bool enabled);

		void updatePositionAndOrientation(
			const Ogre::Vector3 &position, const Ogre::Quaternion &orientation, float scale);

		void setVisible(bool show_frame);
		void setNamesVisible(bool show_names);
		void setAxesVisible(bool show_axes);
		void setParentArrowVisible(bool show_parent_arrow);
		void setLastUpdate(const tf2::TimePoint &latest_time);

		void updateTreeProperty(rviz_common::properties::Property *parent);
		void updateColorForAge(double age, double frame_timeout) const;
		void updateParentArrow(
			const Ogre::Vector3 &position,
			const Ogre::Vector3 &parent_position,
			float scale);

	public Q_SLOTS:
		/** @brief Update whether the frame is visible or not, based on the enabled_property_
		 * in this FrameInfo. */
		void updateVisibilityFromFrame();

		/** @brief Update whether the frame is visible or not, based on the enabled_property_
		 * in the selection handler. */
		void updateVisibilityFromSelection();

	public:
		TFDisplay *display_;
		std::string name_;
		std::string parent_;
		rviz_rendering::Axes *axes_;
		rviz_common::interaction::CollObjectHandle axes_coll_;
		FrameSelectionHandlerPtr selection_handler_;
		rviz_rendering::Arrow *parent_arrow_;
		rviz_rendering::MovableText *name_text_;
		Ogre::SceneNode *name_node_;

		float distance_to_parent_;
		Ogre::Quaternion arrow_orientation_;

		tf2::TimePoint last_update_;
		tf2::TimePoint last_time_to_fixed_;

		rviz_common::properties::VectorProperty *rel_position_property_;
		rviz_common::properties::QuaternionProperty *rel_orientation_property_;
		rviz_common::properties::VectorProperty* rel_euler_property_;
		rviz_common::properties::VectorProperty *position_property_;
		rviz_common::properties::QuaternionProperty *orientation_property_;
		rviz_common::properties::VectorProperty* euler_property_;
		rviz_common::properties::StringProperty *parent_property_;
		rviz_common::properties::BoolProperty *enabled_property_;

		rviz_common::properties::Property *tree_property_;
	};
} // namespace whi_rviz_plugins
