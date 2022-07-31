/******************************************************************
battery visual for showing battery infomation

Features:
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/\
#include "whi_rviz_plugins/battery_visual.h"

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreBillboard.h>
#include <OGRE/OgreBillboardSet.h>
#include <rviz/properties/string_property.h>
#include <rviz/ogre_helpers/movable_text.h>
#include <rviz/display_context.h>

namespace whi_rviz_plugins
{
	BatteryVisual::BatteryVisual(Ogre::SceneManager* SceneManager, Ogre::SceneNode* ParentNode)
	{
		scene_manager_ = SceneManager;

		// Ogre::SceneNode form a tree, with each node storing the
		// transform (position and orientation) of itself relative to its
		// parent.  Ogre does the math of combining those transforms when it
		// is time to render
		//
		// create a node to store the pose of the WhiBattery's header frame
		// relative to the RViz fixed frame
		frame_node_ = ParentNode->createChildSceneNode();

		// create the movable_text object within the frame node so that we can
		// set its position and direction relative to its header frame.
		battery_info_.reset(new rviz::MovableText("?\%"));

		// attach the movable_text to scene node
		frame_node_->attachObject(battery_info_.get());
	}

	BatteryVisual::~BatteryVisual()
	{
		scene_manager_->destroySceneNode(frame_node_);
	}

	void BatteryVisual::setMessage(const whi_interfaces::WhiBattery::ConstPtr& Msg)
	{
		int h = 0;
		int v = 0;
		battery_info_->setTextAlignment((rviz::MovableText::HorizontalAlignment)h, (rviz::MovableText::VerticalAlignment)v);
		rviz::StringProperty text("text", (std::to_string(Msg->percent) + "\%").c_str());
		battery_info_->setCaption(text.getStdString());
		battery_info_->setCharacterHeight(size_);
		battery_info_->setLineSpacing(1.0);
	}

	// position and orientation are passed through to the SceneNode
	void BatteryVisual::setFramePosition(const Ogre::Vector3& Position)
	{
		frame_node_->setPosition(Position);
	}

	void BatteryVisual::setFrameOrientation(const Ogre::Quaternion& Orientation)
	{
		frame_node_->setOrientation(Orientation);
	}

	// color is passed through to the object
	void BatteryVisual::setColor(const Ogre::ColourValue& Color)
	{
		battery_info_->setColor(Color);
	}

	void BatteryVisual::setColor(float Red, float Green, float Blue, float Alpha)
	{
		setColor(Ogre::ColourValue(Red, Green, Blue, Alpha));
	}

	void BatteryVisual::setSize(float Size)
	{
		size_ = Size;
		battery_info_->setCharacterHeight(size_);
	}
} // end namespace whi_rviz_plugins
