/******************************************************************
rviz plugin for navigation initial and goal controll under namespace

Features:
- 2D initial pose
- 2D navigation goal
- add namespace
- record namespace

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_rviz_plugins/panel_nav_namespace.h"
#include "ui_navi_multiple_ns.h"

#include <ros/package.h>
#include <ros/service.h>
#include <iostream>
#include <memory>
#include <boost/filesystem.hpp>

#include <QButtonGroup>
#include <QIcon>

#include <pluginlib/class_list_macros.h>

namespace whi_rviz_plugins
{
    NavNs::NavNs(QWidget* Parent/* = nullptr*/)
        : rviz::Panel(Parent), ui_(new Ui::NaviMultipleNs())
        , node_handle_(std::make_unique<ros::NodeHandle>())
    {
        std::cout << "\nWHI RViz plugin for navigation goal with namespace VERSION 00.01.ing" << std::endl;
        std::cout << "Copyright @ 2023-2024 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n" << std::endl;

        /// set up the GUI
		ui_->setupUi(this);
        // exclusive toggling behavior
        QButtonGroup* buttonGroup = new QButtonGroup;
        buttonGroup->addButton(ui_->pushButton_initial, 0);
        buttonGroup->addButton(ui_->pushButton_goal, 1);
        buttonGroup->setExclusive(true);
        // WHI logo
		boost::filesystem::path packagePath(ros::package::getPath("whi_rviz_plugins"));
		QImage logo;
		if (logo.load(QString(packagePath.string().c_str()) + "/icons/classes/whi_logo.png"))
		{
			QImage scaled = logo.scaledToHeight(48);
			ui_->label_logo->setPixmap(QPixmap::fromImage(scaled));
		}
        // icons on buttons
        QIcon icon;
        icon.addFile(QString(packagePath.string().c_str()) + "/icons/classes/SetInitialPose.png",
            QSize(), QIcon::Normal, QIcon::Off);
        ui_->pushButton_initial->setIcon(icon);
        icon.addFile(QString(packagePath.string().c_str()) + "/icons/classes/SetGoal.png",
            QSize(), QIcon::Normal, QIcon::Off);
        ui_->pushButton_goal->setIcon(icon);
    }

    void NavNs::save(rviz::Config Config) const
    {
        rviz::Panel::save(Config);
        QStringList ns = { "testwhi01", "testwhi02" };
        Config.mapSetValue("whi_navi_ns", ns);
    }

    PLUGINLIB_EXPORT_CLASS(whi_rviz_plugins::NavNs, rviz::Panel)
} // end namespace whi_rviz_plugins
