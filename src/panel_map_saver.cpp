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

#include <nav_msgs/srv/get_map.hpp>
#include "rviz_common/visualization_manager.hpp"

#include <iostream>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QSpacerItem>
#include <QLabel>
#include <QFileDialog>
#include <QMessageBox>

namespace whi_rviz_plugins
{
    MapSaverPanel::MapSaverPanel(QWidget* Parent/* = nullptr*/)
        : rviz_common::Panel(Parent)
    {
        std::cout << "\nWHI RViz plugin for saving map VERSION 02.02.0" << std::endl;
        std::cout << "Copyright @ 2022-2026 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n" << std::endl;

        initLayout();

        node_handle_ = std::make_unique<rclcpp::Node>("MapSaverPanel");
        map_sub_ = node_handle_->create_subscription<nav_msgs::msg::OccupancyGrid>(
	        "map", 10, std::bind(&MapSaverPanel::subCallbackMap, this, std::placeholders::_1));
        map_received_ = node_handle_->get_clock()->now();
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
        std::string package_path = ament_index_cpp::get_package_share_directory("whi_rviz_plugins");
		QImage logo;
		if (logo.load(QString((package_path + "/icons/classes/whi_logo.png").c_str())))
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
            rclcpp::Duration duration = node_handle_->get_clock()->now() - map_received_;
            if (duration.seconds() < 5)
            {
                // vis_manager_->stopUpdate();
                QString fileName = QFileDialog::getSaveFileName(this, tr("Save map"),
                    "/home/whi/untitled", tr("Map Files (*.pgm *.yaml)"));
			    // vis_manager_->startUpdate();
                if (!fileName.isEmpty())
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
        auto client = node_handle_->create_client<nav_msgs::srv::GetMap>("static_map");
        if (client->wait_for_service(std::chrono::seconds(2)))
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
        system((std::string("ros2 run nav2_map_server map_saver_cli -f ") + File).c_str());
    }

    void MapSaverPanel::subCallbackMap(const nav_msgs::msg::OccupancyGrid::SharedPtr Msg)
    {
        map_received_ = node_handle_->get_clock()->now();
    }
} // end namespace whi_rviz_plugins

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(whi_rviz_plugins::MapSaverPanel, rviz_common::Panel)
