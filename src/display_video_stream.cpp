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
#include <OgreRenderWindow.h>
#include "rviz/display_context.h"
#include <rviz/render_panel.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <boost/filesystem.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace whi_rviz_plugins
{
    VideoStreamDisplay::VideoStreamDisplay()
        : ImageDisplayBase(), texture_()
    {
        std::cout << "\nWHI RViz plugin for video stream VERSION 00.02" << std::endl;
        std::cout << "Copyright @ 2022-2023 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n" << std::endl;

        normalize_property_ = new rviz::BoolProperty("Normalize Range", true,
            "If set to true, will try to estimate the range of possible values from the received images",
            this, SLOT(updateNormalizeOptions()));
        min_property_ = new rviz::FloatProperty("Min Value", 0.0,
            "Value which will be displayed as black",
            this, SLOT(updateNormalizeOptions()));
        max_property_ = new rviz::FloatProperty("Max Value", 1.0,
            "Value which will be displayed as white", this, SLOT(updateNormalizeOptions()));
        median_buffer_size_property_ = new rviz::IntProperty("Median window", 5,
            "Window size for median filter used for computin min/max",
            this, SLOT(updateNormalizeOptions()));
        QStringList sourceList = { "Message", "Device", "URL" };
        stream_source_ = new rviz::EnumProperty("Stream source", sourceList[0],
            "Options of selecting stream source",
            this, SLOT(updateStreamSource()));
        for (int i = 0; i < sourceList.size(); ++i)
        {
            stream_source_->addOption(sourceList[i], i);
        }
        stream_device_ = new rviz::IntProperty("Device address", 0,
            "Camera device address, just input 0 for /dev/video0 for an example",
            this, SLOT(updateStreamDevice()));
        stream_device_->setMin(0);
        stream_url_ = new rviz::StringProperty("IP stream address", "",
            "Address of network stream, RTSP and HTTP are supported",
            this, SLOT(updateStreamUrl()));
    }

    VideoStreamDisplay::~VideoStreamDisplay()
    {
        stopCapture();

        if (initialized())
        {
            delete render_panel_;
            delete screen_rect_;
            img_scene_node_->getParentSceneNode()->removeAndDestroyChild(img_scene_node_->getName());
        }
    }

    void VideoStreamDisplay::onInitialize()
    {
        ImageDisplayBase::onInitialize();
        {
            static uint32_t count = 0;
            std::stringstream ss;
            ss << "ImageDisplay" << count++;
            img_scene_manager_ = Ogre::Root::getSingleton().createSceneManager(Ogre::ST_GENERIC, ss.str());
        }

        img_scene_node_ = img_scene_manager_->getRootSceneNode()->createChildSceneNode();

        {
            static int count = 0;
            std::stringstream ss;
            ss << "ImageDisplayObject" << count++;

            screen_rect_ = new Ogre::Rectangle2D(true);
            screen_rect_->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY - 1);
            screen_rect_->setCorners(-1.0f, 1.0f, 1.0f, -1.0f);

            ss << "Material";
            material_ = Ogre::MaterialManager::getSingleton().create(ss.str(),
                Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
            material_->setSceneBlending(Ogre::SBT_REPLACE);
            material_->setDepthWriteEnabled(false);
            material_->setReceiveShadows(false);
            material_->setDepthCheckEnabled(false);

            material_->getTechnique(0)->setLightingEnabled(false);
            Ogre::TextureUnitState* tu = material_->getTechnique(0)->getPass(0)->createTextureUnitState();
            tu->setTextureName(texture_.getTexture()->getName());
            tu->setTextureFiltering(Ogre::TFO_NONE);
            tu->setTextureAddressingMode(Ogre::TextureUnitState::TAM_CLAMP);

            material_->setCullingMode(Ogre::CULL_NONE);
            Ogre::AxisAlignedBox aabInf;
            aabInf.setInfinite();
            screen_rect_->setBoundingBox(aabInf);
            screen_rect_->setMaterial(material_->getName());
            img_scene_node_->attachObject(screen_rect_);
        }

        render_panel_ = new rviz::RenderPanel();
        render_panel_->getRenderWindow()->setAutoUpdated(false);
        render_panel_->getRenderWindow()->setActive(false);

        render_panel_->resize(640, 480);
        render_panel_->initialize(img_scene_manager_, context_);

        setAssociatedWidget(render_panel_);

        render_panel_->setAutoRender(false);
        render_panel_->setOverlaysEnabled(false);
        render_panel_->getCamera()->setNearClipDistance(0.01f);

        updateNormalizeOptions();
        resetTexture();
    }

    void VideoStreamDisplay::update(float WallDt, float RosDt)
    {
        Q_UNUSED(WallDt)
        Q_UNUSED(RosDt)
        try
        {
            texture_.update();

            // make sure the aspect ratio of the image is preserved
            float winWidth = render_panel_->width();
            float winHeight = render_panel_->height();

            float imgWidth = texture_.getWidth();
            float imgHeight = texture_.getHeight();

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
                    screen_rect_->setCorners(-1.0f * imgAspect / winAspect, 1.0f, 1.0f * imgAspect / winAspect,
                        -1.0f, false);
                }
            }

            render_panel_->getRenderWindow()->update();
        }
        catch (rviz::UnsupportedImageEncoding& e)
        {
            setStatus(rviz::StatusProperty::Error, "Image", e.what());
        }
    }

    void VideoStreamDisplay::reset()
    {
        ImageDisplayBase::reset();
        if (resetTexture())
        {
            texture_.clear();
        }
        render_panel_->getCamera()->setPosition(Ogre::Vector3(999999, 999999, 999999));
    }

    void VideoStreamDisplay::onEnable()
    {
        ImageDisplayBase::subscribe();
        render_panel_->getRenderWindow()->setActive(true);
    }

    void VideoStreamDisplay::onDisable()
    {
        render_panel_->getRenderWindow()->setActive(false);
        stopSubscribe();
    }

    void VideoStreamDisplay::processMessage(const sensor_msgs::Image::ConstPtr& Msg)
    {
        bool floatImage = Msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1 ||
            Msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
            Msg->encoding == sensor_msgs::image_encodings::TYPE_16SC1 ||
            Msg->encoding == sensor_msgs::image_encodings::MONO16;

        if (floatImage != float_image_)
        {
            float_image_ = floatImage;
            updateNormalizeOptions();
        }
        texture_.addMessage(Msg);
    }

    bool VideoStreamDisplay::resetTexture()
    {
        // set the empty image to WHI's logo
        boost::filesystem::path path(ros::package::getPath("whi_rviz_plugins"));
        std::string imgPath(path.string() + "/icons/classes/whi_logo.png");
        cv::Mat img = cv::imread(imgPath);
        if (!img.empty())
        {
            processMessage(cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg());
            return false;
        }
        else
        {
            return true;
        }
    }

    void VideoStreamDisplay::stopSubscribe()
    {
        ImageDisplayBase::unsubscribe();
        reset();
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
                    processMessage(cv_bridge::CvImage(std_msgs::Header(), "bgr8", mat).toImageMsg());
                }
            }
        }

        Capture->release();
    }

    void VideoStreamDisplay::updateNormalizeOptions()
    {
        if (float_image_)
        {
            bool normalize = normalize_property_->getBool();

            normalize_property_->setHidden(false);
            min_property_->setHidden(normalize);
            max_property_->setHidden(normalize);
            median_buffer_size_property_->setHidden(!normalize);

            texture_.setNormalizeFloatImage(normalize, min_property_->getFloat(), max_property_->getFloat());
            texture_.setMedianFrames(median_buffer_size_property_->getInt());
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
            ImageDisplayBase::subscribe();
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

    PLUGINLIB_EXPORT_CLASS(whi_rviz_plugins::VideoStreamDisplay, rviz::Display)
} // end namespace whi_rviz_plugins
