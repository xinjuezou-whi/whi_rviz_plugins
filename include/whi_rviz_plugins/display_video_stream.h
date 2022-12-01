/******************************************************************
rviz display for video stream

Features:
- device like /dev/video0, net stream like rtsp, and sensor_msgs::Image
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2022-11-30: Initial version
2022-xx-xx: xxx
******************************************************************/
#pragma once
#include "rviz/image/image_display_base.h"
#include "rviz/image/ros_image_texture.h"
#include "rviz/render_panel.h"
#include "rviz/properties/bool_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/int_property.h"
#include "rviz/properties/enum_property.h"
#include "rviz/properties/string_property.h"
#include <OgreMaterial.h>
#include <OgreSharedPtr.h>
#include <opencv2/highgui.hpp>

#include <thread>
#include <memory>

namespace Ogre
{
    class SceneNode;
    class Rectangle2D;
} // namespace Ogre

namespace whi_rviz_plugins
{
    class VideoStreamDisplay : public rviz::ImageDisplayBase
    {
        Q_OBJECT
    public:
        VideoStreamDisplay();
        ~VideoStreamDisplay() override;

    public:
        // overrides from Display
        void onInitialize() override;
        void update(float WallDt, float RosDt) override;
        void reset() override;

    protected:
        // overrides from Display
        void onEnable() override;
        void onDisable() override;

    protected:
        // this is called by incomingMessage()
        void processMessage(const sensor_msgs::Image::ConstPtr& Msg) override;
        bool resetTexture();
        void stopSubscribe();
        void startCapture(const std::string& Stream);
        void stopCapture();
        void threadCapture(std::unique_ptr<cv::VideoCapture> Capture);

    protected Q_SLOTS:
        void updateNormalizeOptions();
        void updateStreamSource();
        void updateStreamDevice();
        void updateStreamUrl();

    private:
        Ogre::SceneManager* img_scene_manager_{ nullptr };
        rviz::ROSImageTexture texture_;
        rviz::RenderPanel* render_panel_{ nullptr };
        Ogre::SceneNode* img_scene_node_{ nullptr };
        Ogre::Rectangle2D* screen_rect_{ nullptr };
        Ogre::MaterialPtr material_;
        bool float_image_{ false };
        rviz::BoolProperty* normalize_property_{ nullptr };
        rviz::FloatProperty* min_property_{ nullptr };
        rviz::FloatProperty* max_property_{ nullptr };
        rviz::IntProperty* median_buffer_size_property_{ nullptr };
        rviz::EnumProperty* stream_source_{ nullptr };
        rviz::IntProperty* stream_device_{ nullptr };
        rviz::StringProperty* stream_url_{ nullptr };
        std::thread th_capture_;
        std::atomic_bool terminated_{ true };
    };
} // end namespace whi_rviz_plugins
