/******************************************************************
rviz display for video stream

Features:
- device like /dev/video0, net stream like rtsp, and sensor_msgs::Image
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_rviz_plugins/display_video_stream.h"
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreMaterialManager.h>
#include <OgreRoot.h>
#include <OgreRectangle2D.h>
#include <OgreTechnique.h>
#include <OgreTextureManager.h>
#include <OgreRenderWindow.h>
#include "rviz_common/display_context.hpp"
#include "rviz_rendering/render_window.hpp"
#include "rviz_rendering/material_manager.hpp"
#include "rviz_common/message_filter_display.hpp"
#include "rviz_common/uniform_string_stream.hpp"
#include "rviz_default_plugins/displays/image/ros_image_texture.hpp"
#include <sensor_msgs/image_encodings.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <cv_bridge/cv_bridge.h>

namespace whi_rviz_plugins
{
    VideoStreamDisplay::VideoStreamDisplay()
    : VideoStreamDisplay(std::make_unique<rviz_default_plugins::displays::ROSImageTexture>()) {}

    VideoStreamDisplay::VideoStreamDisplay(
        std::unique_ptr<rviz_default_plugins::displays::ROSImageTextureIface> Texture)
        : texture_(std::move(Texture))
    {
        std::cout << "\nWHI RViz plugin for video stream VERSION 02.04.1" << std::endl;
        std::cout << "Copyright @ 2022-2026 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n" << std::endl;

        normalize_property_ = new rviz_common::properties::BoolProperty("Normalize Range", true,
            "If set to true, will try to estimate the range of possible values from the received images",
            this, SLOT(updateNormalizeOptions()));
        min_property_ = new rviz_common::properties::FloatProperty("Min Value", 0.0,
            "Value which will be displayed as black",
            this, SLOT(updateNormalizeOptions()));
        max_property_ = new rviz_common::properties::FloatProperty("Max Value", 1.0,
            "Value which will be displayed as white", this, SLOT(updateNormalizeOptions()));
        median_buffer_size_property_ = new rviz_common::properties::IntProperty("Median window", 5,
            "Window size for median filter used for computin min/max",
            this, SLOT(updateNormalizeOptions()));
        QStringList sourceList = { "Message", "Device", "URL" };
        stream_source_ = new rviz_common::properties::EnumProperty("Stream source", sourceList[0],
            "Options of selecting stream source",
            this, SLOT(updateStreamSource()));
        for (int i = 0; i < sourceList.size(); ++i)
        {
            stream_source_->addOption(sourceList[i], i);
        }
        stream_device_ = new rviz_common::properties::IntProperty("Device address", 0,
            "Camera device address, just input 0 for /dev/video0 for an example",
            this, SLOT(updateStreamDevice()));
        stream_device_->setMin(0);
        stream_url_ = new rviz_common::properties::StringProperty("IP stream address", "",
            "Address of network stream, RTSP and HTTP are supported",
            this, SLOT(updateStreamUrl()));
    }

    VideoStreamDisplay::~VideoStreamDisplay()
    {
        stopCapture();

        if (initialized())
        {
            render_panel_.reset(nullptr);
            screen_rect_.reset(nullptr);
        }
    }

    void VideoStreamDisplay::onInitialize()
    {
        MFDClass::onInitialize();

        updateNormalizeOptions();
        setupScreenRectangle();
        setupRenderPanel();

        render_panel_->getRenderWindow()->setupSceneAfterInit(
            [this](Ogre::SceneNode * scene_node) { scene_node->attachObject(screen_rect_.get()); });

        updateNormalizeOptions();
        updateStreamDevice();
        updateStreamUrl();
        updateStreamSource();
        resetTexture();
    }

    void VideoStreamDisplay::onEnable()
    {
        MFDClass::subscribe();
    }

    void VideoStreamDisplay::onDisable()
    {
        stopSubscribe();
    }

    void VideoStreamDisplay::clear()
    {
        texture_->clear();
    }

    void VideoStreamDisplay::update(float WallDt, float RosDt)
    {
        (void)WallDt;
        (void)RosDt;
        try
        {
            texture_->update();

            // make sure the aspect ratio of the image is preserved
            float winWidth = render_panel_->width();
            float winHeight = render_panel_->height();

            float imgWidth = texture_->getWidth();
            float imgHeight = texture_->getHeight();

            if (imgWidth != 0 && imgHeight != 0 && winWidth != 0 && winHeight != 0)
            {
                float imgAspect = imgWidth / imgHeight;
                float winAspect = winWidth / winHeight;

                if (imgAspect > winAspect)
                {
                    screen_rect_->setCorners(-1.0f, 1.0f * winAspect / imgAspect, 1.0f,
                        -1.0f * winAspect / imgAspect, false);
                }
                else
                {
                    screen_rect_->setCorners(-1.0f * imgAspect / winAspect, 1.0f, 
                        1.0f * imgAspect / winAspect, -1.0f, false);
                }
            }
        }
        catch (rviz_default_plugins::displays::UnsupportedImageEncoding& e)
        {
            setStatus(rviz_common::properties::StatusProperty::Error, "Image", e.what());
        }
    }

    void VideoStreamDisplay::reset()
    {
        MFDClass::reset();
        clear();
        if (resetTexture())
        {
            texture_->clear();
        }
    }

    void VideoStreamDisplay::processMessage(const sensor_msgs::msg::Image::ConstSharedPtr Msg)
    {
        bool floatImage = Msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1 ||
            Msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
            Msg->encoding == sensor_msgs::image_encodings::TYPE_16SC1 ||
            Msg->encoding == sensor_msgs::image_encodings::MONO16;

        if (floatImage != got_float_image_)
        {
            got_float_image_ = floatImage;
            updateNormalizeOptions();
        }
        texture_->addMessage(Msg);
    }

    void VideoStreamDisplay::setupScreenRectangle()
    {
        static int count = 0;
        rviz_common::UniformStringStream ss;
        ss << "VideoStreamDisplayObject" << count++;

        screen_rect_ = std::make_unique<Ogre::Rectangle2D>(true);
        screen_rect_->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY - 1);
        screen_rect_->setCorners(-1.0f, 1.0f, 1.0f, -1.0f);

        ss << "Material";
        material_ = rviz_rendering::MaterialManager::createMaterialWithNoLighting(ss.str());
        material_->setSceneBlending(Ogre::SBT_REPLACE);
        material_->setDepthWriteEnabled(false);
        material_->setDepthCheckEnabled(false);

        Ogre::TextureUnitState* tu =
            material_->getTechnique(0)->getPass(0)->createTextureUnitState();
        tu->setTextureName(texture_->getName());
        tu->setTextureFiltering(Ogre::TFO_NONE);

        material_->setCullingMode(Ogre::CULL_NONE);
        Ogre::AxisAlignedBox aabInf;
        aabInf.setInfinite();
        screen_rect_->setBoundingBox(aabInf);
        screen_rect_->setMaterial(material_);
    }

    void VideoStreamDisplay::setupRenderPanel()
    {
        render_panel_ = std::make_unique<rviz_common::RenderPanel>();
        render_panel_->resize(640, 480);
        render_panel_->initialize(context_);
        setAssociatedWidget(render_panel_.get());

        static int count = 0;
        render_panel_->getRenderWindow()->setObjectName(
            "ImageDisplayRenderWindow" + QString::number(count++));
    }

    void VideoStreamDisplay::stopSubscribe()
    {
        MFDClass::onDisable();
        reset();
    }

    bool VideoStreamDisplay::resetTexture()
    {
        // set the empty image to WHI's logo
        std::string package_path = ament_index_cpp::get_package_share_directory("whi_rviz_plugins");
        std::string imgPath(package_path + "/icons/classes/whi_logo.png");
        cv::Mat img = cv::imread(imgPath);
        if (!img.empty())
        {
            processMessage(cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg());
            return false;
        }
        else
        {
            return true;
        }
    }

    void VideoStreamDisplay::startCapture(const std::string& Stream)
    {
        // spawn the capture thread
        terminated_.store(false);

        auto cap = std::make_unique<cv::VideoCapture>();
        // up to version 4.1.2, the open will be blocked if the device is occupied already
        // issue reference: https://github.com/opencv/opencv/issues/15782
        if (cap->open(Stream))
        {
            th_capture_ = std::thread(std::bind(&VideoStreamDisplay::threadCapture,
                this, std::placeholders::_1), std::move(cap));
        }
    }

    void VideoStreamDisplay::stopCapture()
    {
        terminated_.store(true);
        if (th_capture_.joinable())
        {
            th_capture_.join();
        }

        reset();
    }

    void VideoStreamDisplay::threadCapture(std::unique_ptr<cv::VideoCapture> Capture)
    {
        cv::Mat mat;
        while (!terminated_.load())
        {
            if (Capture->isOpened())
            {
                *Capture >> mat;
                if (!mat.empty())
                {
                    processMessage(cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", mat).toImageMsg());
                }
            }
        }

        Capture->release();
    }

    void VideoStreamDisplay::updateNormalizeOptions()
    {
        if (got_float_image_)
        {
            bool normalize = normalize_property_->getBool();

            normalize_property_->setHidden(false);
            min_property_->setHidden(normalize);
            max_property_->setHidden(normalize);
            median_buffer_size_property_->setHidden(!normalize);

            texture_->setNormalizeFloatImage(
            normalize, min_property_->getFloat(), max_property_->getFloat());
            texture_->setMedianFrames(median_buffer_size_property_->getInt());
        }
        else
        {
            normalize_property_->setHidden(true);
            min_property_->setHidden(true);
            max_property_->setHidden(true);
            median_buffer_size_property_->setHidden(true);
        }
    }

    void VideoStreamDisplay::updateStreamSource()
    {
        stopSubscribe();
        stopCapture();

        if (stream_source_->getOptionInt() == 0)
        {
            onEnable();
        }
        else
        {
            if (stream_source_->getOptionInt() == 1)
            {
                startCapture("/dev/video" + std::to_string(stream_device_->getInt()));
            }
            else
            {
                startCapture(stream_url_->getStdString());
            }
        }
    }

    void VideoStreamDisplay::updateStreamDevice()
    {
        if (stream_source_->getOptionInt() == 1)
        {
            stopCapture();
            startCapture("/dev/video" + std::to_string(stream_device_->getInt()));
        }
    }

    void VideoStreamDisplay::updateStreamUrl()
    {
        if (stream_source_->getOptionInt() == 2)
        {
            stopCapture();
            startCapture(stream_url_->getStdString());
        }
    }
} // end namespace whi_rviz_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(whi_rviz_plugins::VideoStreamDisplay, rviz_common::Display)
