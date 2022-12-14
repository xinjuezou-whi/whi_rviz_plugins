/******************************************************************
battery visual for showing battery infomation

Features:
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2022-07-29: Initial version
2022-xx-xx: xxx
******************************************************************/
#pragma once
#include "whi_interfaces/WhiBattery.h"

namespace Ogre
{
	class Vector3;
	class Quaternion;
	class BillboardSet;
	class SceneManager;
	class SceneNode;
	class ColourValue;
}

namespace rviz
{
	class MovableText;
	class Shape;
	class IntProperty;
}

namespace whi_rviz_plugins
{
	// each instance of BatteryVisual represents the visualization of a single whi_interfaces::WhiBattery message
	class BatteryVisual
	{
	public:
		// creates the visual stuff and puts it into the scene, but in an unconfigured state
		BatteryVisual(Ogre::SceneManager* SceneManager, Ogre::SceneNode* ParentNode);

		// removes the visual stuff from the scene
		virtual ~BatteryVisual();

		// configure the visual to show the data in the message
		void setMessage(const whi_interfaces::WhiBattery::ConstPtr& Msg);
		void createBatteryShape(double PowerRatio, const Ogre::ColourValue& Color);

		// set the pose of the coordinate frame the message refers to these could be done inside setMessage(),
		// but that would require calls to FrameManager and error handling inside setMessage(),
		// which doesn't seem as clean
		// this way BatteryVisual is only responsible for visualization
		void setFramePosition(const Ogre::Vector3& Position);
		void setFrameOrientation(const Ogre::Quaternion& Orientation);

		// set the color and alpha of the visual, which are user-editable parameters
		void setColor(const Ogre::ColourValue& Color);
		void setColor(float Red, float Green, float Blue, float Alpha);

		// set the size of text
		void setSize(float Size);

		// set the battery body offsets and orientation
		void setOffsets(const Ogre::Vector3& Offsets);
		void setOrientation(const Ogre::Vector3& Orientation);

	private:
		float size_{ 1.0 };
		std::shared_ptr<Ogre::Vector3> base_pose_{ nullptr };
		std::shared_ptr<Ogre::Vector3> offsets_{ nullptr };
		std::shared_ptr<Ogre::Vector3> orientation_{ nullptr };
		// the object implementing the actual text
		std::shared_ptr<rviz::MovableText> battery_info_{ nullptr };
		// the object of battery shape
		std::vector<std::shared_ptr<rviz::Shape>> battery_shape_;

		// a SceneNode whose pose is set to match the coordinate frame of the WhiBattery message header
		Ogre::SceneNode* frame_node_{ nullptr };

		// the SceneManager, kept here only so the destructor can ask it to destroy the `frame_node_`
		Ogre::SceneManager* scene_manager_{ nullptr };
	};
} // end namespace whi_rviz_plugins
