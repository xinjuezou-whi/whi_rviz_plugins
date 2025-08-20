/******************************************************************
rviz display for TF frames

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
#include "visibility_control.hpp"

#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include <OgreQuaternion.h>
#include <OgreVector3.h>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2/buffer_core.h"
#include "tf2/time.h"

#include "rviz_common/interaction/forwards.hpp"
#include "rviz_common/display.hpp"

#include "rviz_default_plugins/transformation/transformer_guard.hpp"
#include "rviz_default_plugins/transformation/tf_frame_transformer.hpp"

namespace Ogre
{
	class SceneNode;
}

namespace rviz_rendering
{
	class Arrow;
	class Axes;
	class MovableText;
}

namespace rviz_common
{
	namespace properties
	{
		class BoolProperty;
		class FloatProperty;
		class QuaternionProperty;
		class StringProperty;
		class VectorProperty;
	} // namespace properties
} // namespace rviz_common

namespace whi_rviz_plugins
{
	class FrameInfo;

	class FrameSelectionHandler;

	typedef std::shared_ptr<FrameSelectionHandler> FrameSelectionHandlerPtr;
	typedef std::set<FrameInfo *> S_FrameInfo;

	/** @brief Displays a visual representation of the TF hierarchy. */
	class WHI_RVIZ_PLUGINS_PUBLIC TFDisplay : public rviz_common::Display
	{
		Q_OBJECT

	public:
		TFDisplay();

		~TFDisplay() override;

		void update(float wall_dt, float ros_dt) override;

	protected:
		void onInitialize() override;
		void load(const rviz_common::Config &config) override;
		void fixedFrameChanged() override;
		void reset() override;

	private Q_SLOTS:
		void updateShowAxes();
		void updateShowArrows();
		void updateShowNames();
		void allEnabledChanged();

	private:
		void updateFrames();
		FrameInfo *createFrame(const std::string &frame);
		void updateFrame(FrameInfo *frame);
		void deleteFrame(FrameInfo *frame, bool delete_properties);
		FrameInfo *getFrameInfo(const std::string &frame);
		void clear();

		void onEnable() override;
		void onDisable() override;

		Ogre::SceneNode *root_node_;
		Ogre::SceneNode *names_node_;
		Ogre::SceneNode *arrows_node_;
		Ogre::SceneNode *axes_node_;

		typedef std::map<std::string, FrameInfo *> M_FrameInfo;
		M_FrameInfo frames_;

		typedef std::map<std::string, bool> M_EnabledState;
		M_EnabledState frame_config_enabled_state_;

		float update_timer_;

		rviz_common::properties::BoolProperty *show_names_property_;
		rviz_common::properties::BoolProperty *show_arrows_property_;
		rviz_common::properties::BoolProperty *show_axes_property_;
		rviz_common::properties::FloatProperty *update_rate_property_;
		rviz_common::properties::FloatProperty *frame_timeout_property_;
		rviz_common::properties::BoolProperty *all_enabled_property_;

		rviz_common::properties::FloatProperty *scale_property_;

		rviz_common::properties::Property *frames_category_;
		rviz_common::properties::Property *tree_category_;

		bool changing_single_frame_enabled_state_;

		std::unique_ptr<rviz_default_plugins::transformation::TransformerGuard<
			rviz_default_plugins::transformation::TFFrameTransformer>>
			transformer_guard_;

		void updateRelativePositionAndOrientation(
			const FrameInfo *frame, std::shared_ptr<tf2::BufferCore> tf_buffer) const;

		void logTransformationException(
			const std::string &parent_frame,
			const std::string &child_frame,
			const std::string &message = "") const;

		void updateParentArrowIfTransformExists(FrameInfo *frame, const Ogre::Vector3 &position) const;

		bool hasNoTreePropertyOrParentChanged(
			const FrameInfo *frame, const std::string &old_parent) const;
		void updateParentTreeProperty(FrameInfo *frame) const;

		void deleteObsoleteFrames(std::set<FrameInfo *> &current_frames);
		S_FrameInfo createOrUpdateFrames(const std::vector<std::string> &frames);

		friend class FrameInfo;
	};
} // namespace whi_rviz_plugins
