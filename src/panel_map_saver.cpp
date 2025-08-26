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

#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_rendering/render_window.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <nav_msgs/srv/get_map.hpp>

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QSpacerItem>
#include <QLabel>
#include <QFileDialog>
#include <QMessageBox>

#include <iostream>
#include <algorithm>

namespace whi_rviz_plugins
{
    static std::vector<std::string> pipeExecute(const char* Cmd)
    {
        std::vector<std::string> results;

        const size_t BUF_LEN = 512;
        char buf[BUF_LEN] = {0};

        // Force line-buffered output and capture stderr too
        std::string fullCmd = std::string("stdbuf -oL ") + Cmd + " 2>&1";

        FILE* pipe = popen(fullCmd.c_str(), "r");
        if (!pipe)
        {
            perror("popen failed");
            return results;
        }

        while (fgets(buf, BUF_LEN, pipe) != NULL)
        {
            std::string line(buf);

            // remove trailing newline safely
            if (!line.empty() && line.back() == '\n')
            {
                line.pop_back();
            }

            std::cout << "pipe read line: " << line << std::endl;
            results.push_back(line);
        }
        pclose(pipe);

        return results;
    }

    MapSaverPanel::MapSaverPanel(QWidget* Parent/* = nullptr*/)
        : rviz_common::Panel(Parent)
    {
        std::cout << "\nWHI RViz plugin for saving map VERSION 02.02.3" << std::endl;
        std::cout << "Copyright @ 2022-2026 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n" << std::endl;

        initLayout();
    }

    void MapSaverPanel::onInitialize()
    {
        // Access the abstract ROS Node and
        // in the process lock it for exclusive use until the method is done.
        // Get a pointer to the familiar rclcpp::Node for making subscriptions/publishers
        // (as per normal rclcpp code)
        node_handle_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

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
                // Create a QFileDialog object
                QFileDialog dialog(this);
                // Use the non-native dialog option to avoid blocking the main thread
                dialog.setOption(QFileDialog::DontUseNativeDialog);
                dialog.setAcceptMode(QFileDialog::AcceptSave);
                // Set any other options, like filters or the file mode
                dialog.setNameFilter("Map Files (*.pgm *.yaml)");
                dialog.setFileMode(QFileDialog::AnyFile);
                // Open the dialog. exec() is blocking, but because it's non-native,
                // it doesn't freeze the entire application.
                if (dialog.exec())
                {
                    QStringList selectedFiles = dialog.selectedFiles();
                    if (!selectedFiles.isEmpty())
                    {
                        if (selectedFiles.first().contains(".pgm"))
                        {
                            selectedFiles.first() = selectedFiles.first().remove(".pgm");
                        }
                        if (selectedFiles.first().contains(".yaml"))
                        {
                            selectedFiles.first() = selectedFiles.first().remove(".yaml");
                        }
                        if (save(selectedFiles.first().toStdString()))
                        {
                            labelSaved->setText(selectedFiles.first());
                        }
                        else
                        {
                            QMessageBox::critical(this, tr("Error"), tr("Failed to save map"));
                        }
                    }
                }
            }
            else
            {
                QMessageBox::warning(this, tr("Info"),
                tr("There is no published map.\n"
                   "Please start mapping function first"));
            }

            // rclcpp::Duration duration = node_handle_->get_clock()->now() - map_received_;
            // if (duration.seconds() < 5)
            // {
            //     // manager_->stopUpdate();
            //     QString fileName = QFileDialog::getSaveFileName(this, tr("Save map"),
            //         "/home/whi/untitled", tr("Map Files (*.pgm *.yaml)"));
			//     // manager_->startUpdate();
            //     if (!fileName.isEmpty())
			//     {
            //         if (fileName.contains(".pgm"))
            //         {
            //             fileName = fileName.remove(".pgm");
            //         }
            //         if (fileName.contains(".yaml"))
            //         {
            //             fileName = fileName.remove(".yaml");
            //         }

            //         save(fileName.toStdString());
            //         labelSaved->setText(fileName);
			//     }
            // }
            // else
            // {
            //     QMessageBox::information(this, tr("Info"),
            //     tr("There is no published map.\n"
            //        "Please start mapping function first"));
            // }
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
            QMessageBox::warning(this, tr("Info"),
                tr("There is no active map_server.\n"
                   "Please start map_server first"));
            return false;
        }
    }

    bool MapSaverPanel::save(std::string File)
    {
        auto res = pipeExecute((std::string("ros2 run nav2_map_server map_saver_cli -f ") + File).c_str());
        const auto it = std::find_if(res.begin(), res.end(), [](const std::string Item)
        {
            return Item.find("Failed to save the map") != std::string::npos;
        });

        return it == res.end();
    }

    void MapSaverPanel::subCallbackMap(const nav_msgs::msg::OccupancyGrid::SharedPtr Msg)
    {
        map_received_ = node_handle_->get_clock()->now();
    }
} // end namespace whi_rviz_plugins

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(whi_rviz_plugins::MapSaverPanel, rviz_common::Panel)
