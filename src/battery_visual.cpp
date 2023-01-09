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
		base_pose_ = std::make_shared<Ogre::Vector3>(Ogre::Vector3::ZERO);
		offsets_ = std::make_shared<Ogre::Vector3>(Ogre::Vector3::ZERO);
		orientation_ = std::make_shared<Ogre::Vector3>(Ogre::Vector3(float(0.0), float(90.0), float(0.0)));
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
		rviz::StringProperty text("text", (std::to_string(Msg->soc) + "\%").c_str());
		battery_info_->setCaption(text.getStdString());
		battery_info_->setLineSpacing(1.0);
	}

	// position and orientation are passed through to the SceneNode
	void BatteryVisual::setFramePosition(const Ogre::Vector3& Position)
	{
		*base_pose_ = Position;
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
		battery_info_->setLocalTranslation(Ogre::Vector3(-offsets_->y, offsets_->z, -offsets_->x));
	}

	void BatteryVisual::setOrientation(const Ogre::Vector3& Orientation)
	{
		*orientation_ = Orientation;
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
		mat.FromEulerAnglesXYZ(Ogre::Degree(orientation_->x), Ogre::Degree(orientation_->y), Ogre::Degree(orientation_->z));
		Ogre::Quaternion orientation;
		orientation.FromRotationMatrix(mat);

		// symbol size
		float bodyHeight = float(0.7 * size_);
		float powerHeight = float(3.0 * bodyHeight / 4.0);
		float poleHeight = float(0.1 * size_);
		float poleOffset = -float(0.5 * poleHeight);

		// pole
		Ogre::Vector3 polePose(0.0, poleOffset, 0.0);
		Ogre::Vector3 polePoseRotated = orientation * polePose;
		battery_shape_[0].reset(new rviz::Shape(rviz::Shape::Cylinder, scene_manager_));
		battery_shape_[0]->setScale(Ogre::Vector3(float(0.25 * size_), poleHeight, float(0.25 * size_)));
		battery_shape_[0]->setPosition(Ogre::Vector3(base_pose_->x + polePoseRotated.x + offsets_->x,
			base_pose_->y + polePoseRotated.y + offsets_->y,
			base_pose_->z + polePoseRotated.z + offsets_->z));
		battery_shape_[0]->setOrientation(orientation);

		// up
		float bottomHeight = float(0.5 * (bodyHeight - powerHeight));
		float upHeight = float(bottomHeight + (1.0 - PowerRatio) * powerHeight);
		float upOffset = -float(0.5 * upHeight + poleHeight);
		Ogre::Vector3 upPose(0.0, upOffset, 0.0);
		Ogre::Vector3 upPoseRotated = orientation * upPose;
		battery_shape_[1].reset(new rviz::Shape(rviz::Shape::Cylinder, scene_manager_));
		battery_shape_[1]->setScale(Ogre::Vector3(float(0.5 * size_), upHeight, float(0.5 * size_)));
		battery_shape_[1]->setPosition(Ogre::Vector3(base_pose_->x + upPoseRotated.x + offsets_->x,
			base_pose_->y + upPoseRotated.y + offsets_->y,
			base_pose_->z + upPoseRotated.z + offsets_->z));
		battery_shape_[1]->setOrientation(orientation);

		// power
		float leftPowerHeight = float(PowerRatio * powerHeight);
		float leftPowerOffset = -float(0.5 * leftPowerHeight + poleHeight + upHeight);
		Ogre::Vector3 leftPowerPose(0.0, leftPowerOffset, 0.0);
		Ogre::Vector3 leftPowerPoseRotated = orientation * leftPowerPose;
		battery_shape_[2].reset(new rviz::Shape(rviz::Shape::Cylinder, scene_manager_));
		battery_shape_[2]->setScale(Ogre::Vector3(float(0.5 * size_), leftPowerHeight, float(0.5 * size_)));
		battery_shape_[2]->setPosition(Ogre::Vector3(base_pose_->x + leftPowerPoseRotated.x + offsets_->x,
			base_pose_->y + leftPowerPoseRotated.y + offsets_->y,
			base_pose_->z + leftPowerPoseRotated.z + offsets_->z));
		battery_shape_[2]->setOrientation(orientation);
		battery_shape_[2]->setColor(Color);

		// bottom
		float bottomOffset = -float(0.5 * bottomHeight + poleHeight + upHeight + leftPowerHeight);
		Ogre::Vector3 bottomPose(0.0, bottomOffset, 0.0);
		Ogre::Vector3 bottomPoseRotated = orientation * bottomPose;
		battery_shape_[3].reset(new rviz::Shape(rviz::Shape::Cylinder, scene_manager_));
		battery_shape_[3]->setScale(Ogre::Vector3(float(0.5 * size_), bottomHeight, float(0.5 * size_)));
		battery_shape_[3]->setPosition(Ogre::Vector3(base_pose_->x + bottomPoseRotated.x + offsets_->x,
			base_pose_->y + bottomPoseRotated.y + offsets_->y,
			base_pose_->z + bottomPoseRotated.z + offsets_->z));
		battery_shape_[3]->setOrientation(orientation);
	}
} // end namespace whi_rviz_plugins
