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
#include <rviz/ogre_helpers/shape.h>
#include <rviz/display_context.h>

namespace whi_rviz_plugins
{
	BatteryVisual::BatteryVisual(Ogre::SceneManager* SceneManager, Ogre::SceneNode* ParentNode)
	{
		scene_manager_ = SceneManager;

		// Ogre::SceneNode form a tree, 
		// with each node storing the transform (position and orientation) of itself relative to its parent
		// Ogre does the math of combining those transforms when it is time to render
		//
		// create a node to store the pose of the WhiBattery's header frame
		// relative to the RViz fixed frame
		frame_node_ = ParentNode->createChildSceneNode();

		// create the movable_text object within the frame node,
		// so that we can set its position and direction relative to its header frame
		battery_info_.reset(new rviz::MovableText("?\%"));
		battery_info_->setCharacterHeight(size_);
		// attach the movable_text to scene node
		frame_node_->attachObject(battery_info_.get());

		// create shapes of battery
		offsets_ = std::make_shared<Ogre::Vector3>(Ogre::Vector3::ZERO);
		createBatteryShape(1.0, Ogre::ColourValue(float(0.0), float(1.0), float(0.0), float(0.8)));
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

	void BatteryVisual::setOffsets(const Ogre::Vector3& Offsets)
	{
		*offsets_ = Offsets;
		battery_info_->setLocalTranslation(Ogre::Vector3(-offsets_->x, offsets_->z, offsets_->y));
	}

	void BatteryVisual::createBatteryShape(double PowerRatio, const Ogre::ColourValue& Color)
	{
		for (auto& it : battery_shape_)
		{
			it.reset();
		}
		battery_shape_.clear();
		battery_shape_.resize(4);

		Ogre::Matrix3 mat;
		mat.FromEulerAnglesXYZ(Ogre::Radian(float(1.57)), Ogre::Radian(float(0.0)), Ogre::Radian(float(0.0)));
		Ogre::Quaternion orientation;
		orientation.FromRotationMatrix(mat);

		float bodyHeight = float(0.7 * size_);
		float powerHeight = float(3.0 * bodyHeight / 4.0);
		// pole
		battery_shape_[0].reset(new rviz::Shape(rviz::Shape::Cylinder, scene_manager_));
		battery_shape_[0]->setScale(Ogre::Vector3(float(0.25 * size_), float(0.1 * size_), float(0.25 * size_)));
		battery_shape_[0]->setPosition(Ogre::Vector3(float(1.1 * 0.5 * size_ + offsets_->x), float(offsets_->y), float(0.5 * 0.1 * size_ + offsets_->z)));
		battery_shape_[0]->setOrientation(orientation);
		// up
		float bottomEdge = float(0.5 * (bodyHeight - powerHeight));
		float upperEdge = float(bottomEdge + (1.0 - PowerRatio) * powerHeight);
		float upEdgeOffset = float(-0.5 * upperEdge);
		battery_shape_[1].reset(new rviz::Shape(rviz::Shape::Cylinder, scene_manager_));
		battery_shape_[1]->setScale(Ogre::Vector3(float(0.5 * size_), upperEdge, float(0.5 * size_)));
		battery_shape_[1]->setPosition(Ogre::Vector3(float(1.1 * 0.5 * size_ + offsets_->x), offsets_->y, upEdgeOffset + offsets_->z));
		battery_shape_[1]->setOrientation(orientation);
		// power
		float leftPowerHeight = float(PowerRatio * powerHeight);
		float leftPowerOffset = float(-(upperEdge + 0.5 * leftPowerHeight));
		battery_shape_[2].reset(new rviz::Shape(rviz::Shape::Cylinder, scene_manager_));
		battery_shape_[2]->setScale(Ogre::Vector3(float(0.5 * size_), leftPowerHeight, float(0.5 * size_)));
		battery_shape_[2]->setPosition(Ogre::Vector3(float(1.1 * 0.5 * size_ + offsets_->x), offsets_->y, leftPowerOffset + offsets_->z));
		battery_shape_[2]->setColor(Color);
		battery_shape_[2]->setOrientation(orientation);
		// bottom
		float bottomEdgeOffset = float(-(upperEdge + leftPowerHeight + 0.5 * bottomEdge));
		battery_shape_[3].reset(new rviz::Shape(rviz::Shape::Cylinder, scene_manager_));
		battery_shape_[3]->setScale(Ogre::Vector3(float(0.5 * size_), bottomEdge, float(0.5 * size_)));
		battery_shape_[3]->setPosition(Ogre::Vector3(float(1.1 * 0.5 * size_ + offsets_->x), offsets_->y, bottomEdgeOffset + offsets_->z));
		battery_shape_[3]->setOrientation(orientation);
	}
} // end namespace whi_rviz_plugins
