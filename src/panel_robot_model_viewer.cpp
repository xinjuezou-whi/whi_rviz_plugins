/******************************************************************
rviz plugin for robot model viewer

Features:
- OGRE view
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_rviz_plugins/panel_robot_model_viewer.h"
#include "whi_rviz_plugins/render_panel_custom.h"
#include "ui_navi_robot_model_viewer.h"

#include <ros/package.h>
#include <rviz/display.h>
#include <rviz/display_context.h>
#include <rviz/visualization_manager.h>
#include <rviz/selection/selection_manager.h>
#include <rviz/view_manager.h>
#include <rviz/render_panel.h>
#include <rviz/tool_manager.h>
#include <rviz/properties/parse_color.h>
#include <boost/filesystem.hpp>
#include <QTimer>

namespace whi_rviz_plugins
{
    RobotModelViewerPanel::RobotModelViewerPanel(rviz::DisplayContext* Context, Ogre::SceneNode* SceneNode,
        QWidget* Parent/* = nullptr*/)
        : context_(Context), scene_node_(SceneNode), QWidget(Parent), ui_(new Ui::NaviRobotModelViewer())
	{
		// set up the GUI
		ui_->setupUi(this);

		// WHI logo
        boost::filesystem::path path(ros::package::getPath("whi_rviz_plugins"));
		QImage logo;
		if (logo.load(QString(path.string().c_str()) + "/icons/classes/whi_logo.png"))
		{
			QImage scaled = logo.scaledToHeight(48);
			ui_->label_logo->setPixmap(QPixmap::fromImage(scaled));
		}
        // other properties
        ui_->comboBox_view->addItems(QStringList({ "rviz/Orbit", "rviz/TopDownOrtho" }));
        ui_->comboBox_view->setCurrentIndex(0);
        // signals
        connect(ui_->comboBox_view, QOverload<int>::of(&QComboBox::activated), this,
			[=](int Index) { onViewIndexChanged(Index, this); });

        // construct and lay out render panel.
        render_panel_ = new rviz::RenderPanelCustom();
        ((rviz::RenderPanelCustom*)render_panel_)->setEventFilters(rviz::RenderPanelCustom::MOUSE_PRESS_EVENT,
            false, Qt::NoModifier, Qt::RightButton);
        ((rviz::RenderPanelCustom*)render_panel_)->setEventFilters(rviz::RenderPanelCustom::MOUSE_RELEASE_EVENT,
            false, Qt::NoModifier, Qt::RightButton);
        ((rviz::RenderPanelCustom*)render_panel_)->setEventFilters(rviz::RenderPanelCustom::MOUSE_MOVE_EVENT,
            false, Qt::NoModifier, Qt::RightButton);

        ui_->horizontalLayout_main->insertWidget(0, render_panel_);

        // next we initialize the main RViz classes
        //
        // the VisualizationManager is the container for Display objects,
        // holds the main Ogre scene, holds the ViewController, etc.  It is
        // very central and we will probably need one in every usage of
        // librviz.
        manager_ = new rviz::VisualizationManager(render_panel_);
        render_panel_->initialize(manager_->getSceneManager(), manager_);
        
        Ogre::SceneNode* lightSceneNode = NULL;
        Ogre::Light* light = manager_->getSceneManager()->createLight();

        // set some attributes of the light
        // the basic light type can be:
        //   pointlight (like a candle?)
        //   spotlight (kind of 'conic' light)
        //   directional light (like the sun in an outdoor scene)
        // directional light is like parallel rays coming from 1 direction
        light->setType(Ogre::Light::LT_DIRECTIONAL);

        // choose the color of the light
        // the diffuse color is the main color of the light
        // the specular color is its color when reflected on an imperfect surface
        // for example, when my bald head skin reflect the sun, it makes a bright round of specular color
        //
        // the final color of an object also depends on its material
        // color values vary between 0.0(minimum) to 1.0 (maximum)
        light->setDiffuseColour(0.25f, 0.25f, 0.25f); // this will be a red light
        light->setSpecularColour(1.0f, 1.0f, 1.0f);// color of 'reflected' light

        lightSceneNode = manager_->getSceneManager()->getRootSceneNode()->createChildSceneNode();
        lightSceneNode->attachObject(light);

        // set reference frame
        // IMPORTANT: WITHOUT THIS, ALL THE DIFFERENT PARTS OF THE ROBOT MODEL WILL BE DISPLAYED AT 0,0,0
        manager_->setFixedFrame("base_link");

        manager_->initialize();
        manager_->startUpdate();

        // create a Grid display
        grid_ = manager_->createDisplay("rviz/Grid", "Grid", true);
        ROS_ASSERT(grid_ != NULL);
        grid_->subProp("Plane Cell Count")->setValue(10);
        grid_->subProp("Cell Size")->setValue(1.0);
        grid_->subProp("Alpha")->setValue(0.5);
        // create a RobotModel display
        robot_model_ = manager_->createDisplay("rviz/RobotModel", "Robot model", true );
        ROS_ASSERT(robot_model_ != NULL);
        robot_model_->subProp("Robot Description")->setValue("robot_description");

        // handles mouse events without rviz::tool
        mouse_event_handler_ = new MouseEventHandler();
        QObject::connect(render_panel_, SIGNAL(signalMousePressEvent(QMouseEvent*)), mouse_event_handler_, SLOT(mousePressEvent(QMouseEvent*)));
        QObject::connect(render_panel_, SIGNAL(signalMouseReleaseEvent(QMouseEvent*)), mouse_event_handler_, SLOT(mouseReleaseEvent(QMouseEvent*)));
        QObject::connect(render_panel_, SIGNAL(signalMouseDoubleClickEvent(QMouseEvent*)), mouse_event_handler_, SLOT(mouseDoubleClick(QMouseEvent*)));

        // set background color to rviz default
        setBackgroundColor(QColor(48, 48, 48));
    }

    RobotModelViewerPanel::~RobotModelViewerPanel()
	{
		delete ui_;
        delete manager_;
	}

    void RobotModelViewerPanel::setBackgroundColor(const QColor& Color)
    {
        if (render_panel_)
        {
            render_panel_->getViewport()->setBackgroundColour(rviz::qtToOgre(Color));
        }
    }

    void RobotModelViewerPanel::setFixedFrame(const QString& Frame)
    {
        if (manager_)
        {
            manager_->setFixedFrame(Frame.toStdString().c_str());
        }
    }

    void RobotModelViewerPanel::setRobotDescription(const QString& Description)
    {
        if (robot_model_)
        {
            robot_model_->subProp("Robot Description")->setValue(Description.toStdString().c_str());
        }
    }

    void RobotModelViewerPanel::updateCameraParams()
    {
        QPoint mousePanel = render_panel_->mapFromGlobal(QCursor::pos());
        Ogre::Vector3 pointWorld;
        context_->getSelectionManager()->get3DPoint(render_panel_->getViewport(), mousePanel.x(),
                                                    mousePanel.y(), pointWorld);
        auto camera = render_panel_->getCamera();
        Ogre::Vector3 position = camera->getPosition();

        if (ui_->comboBox_view->currentIndex() == 0)
        {
            Ogre::Quaternion orientation = camera->getOrientation();
            focal_point_ = orientation.Inverse() * (pointWorld - scene_node_->getPosition());
            distance_ = focal_point_.distance(position);
            Ogre::Vector3 diff = position - focal_point_;
            pitch_ = std::asin(diff.z / distance_);
            yaw_ = atan2(diff.y, diff.x);
#ifdef DEBUG
            std::cout << "focal x:" << focal_point_.x << ",y:" << focal_point_.y << ",z:" << focal_point_.z << std::endl;
            std::cout << "distance: " << distance_ << ", pitch: " << pitch_ << ", yaw: " << yaw_ << std::endl;
#endif
        }
        else if (ui_->comboBox_view->currentIndex() == 1)
        {

        }
    }

    void RobotModelViewerPanel::onViewIndexChanged(int Index, QWidget* Group)
    {
        manager_->getViewManager()->setCurrentViewControllerType(ui_->comboBox_view->itemText(Index));
    }

    void RobotModelViewerPanel::load(const rviz::Config& Config)
    {
        // not an override from rviz::Panel
        //rviz::Panel::load(Config);

        auto viewer = Config.mapGetChild("robot_model_viewer");
        auto view = viewer.mapGetChild("view").getValue().toString();
        QTimer::singleShot(500, this, [=]()
		{
            ui_->comboBox_view->setCurrentText(view);
            if (manager_)
            {
                manager_->getViewManager()->setCurrentViewControllerType(view);
            }
		});
        if (ui_->comboBox_view->currentIndex() == 0)
        {
            distance_ = viewer.mapGetChild("distance").getValue().toDouble();
            pitch_ = viewer.mapGetChild("pitch").getValue().toDouble();
            yaw_ = viewer.mapGetChild("yaw").getValue().toDouble();
            auto focal = viewer.mapGetChild("focal point");
            focal_point_.x = focal.mapGetChild("x").getValue().toDouble();
            focal_point_.y = focal.mapGetChild("y").getValue().toDouble();
            focal_point_.z = focal.mapGetChild("z").getValue().toDouble();

            // Ogre::Vector3 pos;
            // pos.x = distance_ * std::cos(yaw_) * std::cos(pitch_) + focal_point_.x;
            // pos.y = distance_ * std::sin(yaw_) * std::cos(pitch_) + focal_point_.y;
            // pos.z = distance_ * std::sin(pitch_) + focal_point_.z;
            // auto camera = render_panel_->getCamera();
            // camera->setPosition(pos);
            // Ogre::Vector3 cameraZ = Ogre::Vector3::UNIT_Z;
            // camera->setFixedYawAxis(true, scene_node_->getOrientation() * cameraZ);
            // camera->setDirection(scene_node_->getOrientation() * (focal_point_ - pos));
            // camera->setFOVy(Ogre::Radian(0.05));
        }
        else if (ui_->comboBox_view->currentIndex() == 1)
        {
            scale_ = viewer.mapGetChild("scale").getValue().toDouble();
            angle_ = viewer.mapGetChild("angle").getValue().toDouble();
            x_ = viewer.mapGetChild("x").getValue().toDouble();
            y_ = viewer.mapGetChild("y").getValue().toDouble();
        }
    }

    void RobotModelViewerPanel::save(rviz::Config Config) const
    {
        // not an override from rviz::Panel
        //rviz::Panel::save(Config);

        auto viewer = Config.mapMakeChild("robot_model_viewer");
        viewer.mapMakeChild("view").setValue(ui_->comboBox_view->currentText());
        if (ui_->comboBox_view->currentIndex() == 0)
        {
            viewer.mapMakeChild("distance").setValue(distance_);
            viewer.mapMakeChild("pitch").setValue(pitch_);
            viewer.mapMakeChild("yaw").setValue(yaw_);
            auto childFocal = viewer.mapMakeChild("focal point");
            childFocal.mapMakeChild("x").setValue(focal_point_.x);
            childFocal.mapMakeChild("y").setValue(focal_point_.y);
            childFocal.mapMakeChild("z").setValue(focal_point_.z);
        }
        else if (ui_->comboBox_view->currentIndex() == 1)
        {
            viewer.mapMakeChild("scale").setValue(scale_);
            viewer.mapMakeChild("angle").setValue(angle_);
            viewer.mapMakeChild("x").setValue(x_);
            viewer.mapMakeChild("y").setValue(y_);
        }
    }
} // end namespace whi_rviz_plugins
