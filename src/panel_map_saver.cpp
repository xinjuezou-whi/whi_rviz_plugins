/******************************************************************
rviz plugin for saving map with map_server

Features:
- map_server map_saver -f <map>
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_rviz_plugins/panel_map_saver.h"

#include <ros/package.h>
#include <ros/service.h>
#include <iostream>
#include <boost/filesystem.hpp>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QSpacerItem>
#include <QLabel>
#include <QFileDialog>
#include <QMessageBox>

#include <pluginlib/class_list_macros.h>

namespace whi_rviz_plugins
{
    MapSaverPanel::MapSaverPanel(QWidget* Parent/* = nullptr*/)
        : rviz::Panel(Parent)
    {
        std::cout << "\nWHI RViz plugin for saving map VERSION 00.01" << std::endl;
        std::cout << "Copyright @ 2022-2024 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n" << std::endl;

        initLayout();

        node_handle_ = std::make_unique<ros::NodeHandle>();
        map_sub_ = std::make_unique<ros::Subscriber>(node_handle_->subscribe<nav_msgs::OccupancyGrid>("map", 10,
            std::bind(&MapSaverPanel::subCallbackMap, this, std::placeholders::_1)));
    }

    void MapSaverPanel::initLayout()
    {
        QVBoxLayout* layoutMain = new QVBoxLayout(this);
        // line 1
        QHBoxLayout* hBox = new QHBoxLayout();
        QPushButton* buttonSave = new QPushButton("Save");
        hBox->addWidget(buttonSave);
        QSpacerItem* horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);
        hBox->addItem(horizontalSpacer);
        QLabel* labelLogo = new QLabel("");
        hBox->addWidget(labelLogo);
        boost::filesystem::path path(ros::package::getPath("whi_rviz_plugins"));
		QImage logo;
		if (logo.load(QString(path.string().c_str()) + "/icons/classes/whi_logo.png"))
		{
			QImage scaled = logo.scaledToHeight(48);
			labelLogo->setPixmap(QPixmap::fromImage(scaled));
		}
        layoutMain->addLayout(hBox);
        // line 2
        hBox = new QHBoxLayout();
        QLabel* labelSaved = new QLabel("no saved map yet");
        hBox->addWidget(labelSaved);
        layoutMain->addLayout(hBox);

        // signal
        connect(buttonSave, &QPushButton::clicked, this, [=]()
        {
            ros::Duration duration = ros::Time::now() - map_received_;
            if (duration.toSec() < 5)
            {
                QString fileName = QFileDialog::getSaveFileName(this, tr("Save map"), "/home/whi/untitled", tr("Map Files (*.pgm *.yaml)"));
			    if (!fileName.isNull())
			    {
                    if (fileName.contains(".pgm"))
                    {
                        fileName = fileName.remove(".pgm");
                    }
                    if (fileName.contains(".yaml"))
                    {
                        fileName = fileName.remove(".yaml");
                    }

                    save(fileName.toStdString());
                    labelSaved->setText(fileName);
			    }
            }
            else
            {
                QMessageBox::information(this, tr("Info"),
                tr("There is no published map.\n"
                   "Please start mapping function first"));
            }
        });
    }

    bool MapSaverPanel::mapServerValid()
    {
        if (ros::service::exists("static_map", false))
        {
            return true;
        }
        else
        {
            QMessageBox::information(this, tr("Info"),
                tr("There is no active map_server.\n"
                   "Please start map_server first"));
            return false;
        }
    }

    void MapSaverPanel::save(std::string File)
    {
        system((std::string("rosrun map_server map_saver -f ") + File).c_str());
    }

    void MapSaverPanel::subCallbackMap(const nav_msgs::OccupancyGridConstPtr& Map)
    {
        map_received_ = ros::Time::now();
    }

    PLUGINLIB_EXPORT_CLASS(whi_rviz_plugins::MapSaverPanel, rviz::Panel)
} // end namespace whi_rviz_plugins
