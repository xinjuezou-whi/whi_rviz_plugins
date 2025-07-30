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
2025-07-30: Migrate from ROS 1
2025-xx-xx: xxx
******************************************************************/
#pragma once
#include "whi_rviz_plugins/visibility_control.hpp"

#include "rviz_default_plugins/displays/image/image_display.hpp"
#include "rviz_default_plugins/displays/image/ros_image_texture.hpp"
#include "rviz_default_plugins/displays/image/ros_image_texture_iface.hpp"
#include "rviz_common/message_filter_display.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/int_property.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/string_property.hpp"
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
    class WHI_RVIZ_PLUGINS_PUBLIC VideoStreamDisplay : public
        rviz_common::MessageFilterDisplay<sensor_msgs::msg::Image>
    {
        Q_OBJECT
    public:
        explicit VideoStreamDisplay(std::unique_ptr<rviz_default_plugins::displays::ROSImageTextureIface> Texture);
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
        // this is called by incomingMessage()
        void processMessage(const sensor_msgs::msg::Image::ConstSharedPtr Msg) override;

    protected:
        void setupScreenRectangle();
        void setupRenderPanel();
        void clear();
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
        std::unique_ptr<rviz_default_plugins::displays::ROSImageTextureIface> texture_{ nullptr };
        std::unique_ptr<Ogre::Rectangle2D> screen_rect_;
        Ogre::MaterialPtr material_;
        std::unique_ptr<rviz_common::RenderPanel> render_panel_{ nullptr };
        bool got_float_image_{ false };
        rviz_common::properties::BoolProperty* normalize_property_{ nullptr };
        rviz_common::properties::FloatProperty* min_property_{ nullptr };
        rviz_common::properties::FloatProperty* max_property_{ nullptr };
        rviz_common::properties::IntProperty* median_buffer_size_property_{ nullptr };
        rviz_common::properties::EnumProperty* stream_source_{ nullptr };
        rviz_common::properties::IntProperty* stream_device_{ nullptr };
        rviz_common::properties::StringProperty* stream_url_{ nullptr };
        std::thread th_capture_;
        std::atomic_bool terminated_{ true };
    };
} // end namespace whi_rviz_plugins
