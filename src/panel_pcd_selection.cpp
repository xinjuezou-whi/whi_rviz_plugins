/******************************************************************
rviz plugin for selecte point cloud in rviz and save the selected into pcd file

Features:
- mouse selection in rviz
- xxx

Dependency:
- pcl-tools: sudo apt install pcl-tools

Written by Xinjue Zou, xinjue.zou@outlook.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_rviz_plugins/panel_pcd_selection.h"
#include "whi_rviz_plugins/utility.h"

#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/properties/property_tree_widget.hpp>
#include <rviz_common/interaction/selection_manager.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

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
    PcdSelectionPanel::PcdSelectionPanel(QWidget* Parent/* = nullptr*/)
        : rviz_common::Panel(Parent)
    {
        std::cout << "\nWHI RViz plugin for pcd selection and saving VERSION 00.02.2" << std::endl;
        std::cout << "Copyright@2026 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n" << std::endl;

        initLayout();
    }

    void PcdSelectionPanel::onInitialize()
    {
        tree_widget_->setModel(getDisplayContext()->getSelectionManager()->getPropertyModel());
    }

    void traverseProperty(rviz_common::properties::Property* Prop)
    {
        if (!Prop)
        {
            return;
        }

        std::cout << Prop->getName().toStdString() << ": " << Prop->getValue().toString().toStdString() << std::endl;

        int numChildren = Prop->numChildren();
        for (int i = 0; i < numChildren; ++i)
        {
            traverseProperty(Prop->childAt(i));
        }
    }

    void PcdSelectionPanel::initLayout()
    {
        tree_widget_ = new rviz_common::properties::PropertyTreeWidget();

        QVBoxLayout* layoutMain = new QVBoxLayout(this);
        // line 1
        QHBoxLayout* hBox = new QHBoxLayout();
        QPushButton* buttonSave = new QPushButton("Save");
        hBox->addWidget(buttonSave);
        QPushButton* buttonView = new QPushButton("View");
        hBox->addWidget(buttonView);
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
        QVBoxLayout* vBox = new QVBoxLayout();
        QLabel* labelSaved = new QLabel("no saved PCD yet");
        vBox->addWidget(labelSaved);
        QLabel* labelNum = new QLabel("");
        vBox->addWidget(labelNum);
        hBox->addLayout(vBox);
        layoutMain->addLayout(hBox);

        // signal
        connect(buttonSave, &QPushButton::clicked, this, [=]()
        {
            rviz_common::properties::PropertyTreeModel* treeModel = tree_widget_->getModel();

            std::vector<PointXYZI> points;
            extractPoints(treeModel->getRoot(), points);

            if (!points.empty())
            {
                // Create a QFileDialog object
                QFileDialog dialog(this);
                // Use the non-native dialog option to avoid blocking the main thread
                dialog.setOption(QFileDialog::DontUseNativeDialog);
                dialog.setAcceptMode(QFileDialog::AcceptSave);
                // Set any other options, like filters or the file mode
                dialog.setNameFilter("PCD Files (*.pcd)");
                dialog.setFileMode(QFileDialog::AnyFile);
                // Open the dialog. exec() is blocking, but because it's non-native,
                // it doesn't freeze the entire application.
                if (dialog.exec())
                {
                    QStringList selectedFiles = dialog.selectedFiles();
                    if (!selectedFiles.isEmpty())
                    {
                        if (selectedFiles.first().contains(".pcd"))
                        {
                            selectedFiles.first() = selectedFiles.first().remove(".pcd");
                        }

                        if (save(selectedFiles.first().toStdString(), points))
                        {
                            labelSaved->setText(selectedFiles.first() + ".pcd");
                            labelNum->setText(QString::number(points.size()) + " points");
                        }
                        else
                        {
                            QMessageBox::critical(this, tr("Error"), tr("Failed to save PCD"));
                        }
                    }
                }
            }
            else
            {
                QMessageBox::warning(this, "No Points Selected", "Please select some points in RViz before saving.");
                return;
            }
        });

        connect(buttonView, &QPushButton::clicked, this, [=]()
        {
            auto text = labelSaved->text();
            if (text.contains(".pcd"))
            {
                pipeExecute((std::string("pcl_viewer ") + text.toStdString()).c_str());
            }
            else   
            {
                QMessageBox::warning(this, "No PCD Saved", "Please save a PCD file first before viewing.");
            }
        });
    }

    void PcdSelectionPanel::extractPoints(rviz_common::properties::Property* Prop, std::vector<PointXYZI>& Points)
    {
        if (!Prop)
        {
            return;
        }

        std::string name = Prop->getName().toStdString();
        if (name.find("Point ") != std::string::npos)
        {
            PointXYZI pt{0, 0, 0, 0};
            for (int i = 0; i < Prop->numChildren(); ++i)
            {
                auto child = Prop->childAt(i);
                std::string cname = child->getName().toStdString();
                std::string val = child->getValue().toString().toStdString();
#ifndef DEBUG
                std::cout << "Processing property: " << cname << " with value: " << val << std::endl;
#endif

                if (cname == "Position")
                {
                    std::stringstream ss(val);
                    std::string item;

                    std::getline(ss, item, ';'); pt.x = std::stof(item);
                    std::getline(ss, item, ';'); pt.y = std::stof(item);
                    std::getline(ss, item, ';'); pt.z = std::stof(item);
                }
                else if (cname.find("intensity") != std::string::npos)
                {
                    pt.intensity = std::stof(val);
                }
            }

#ifndef DEBUG
            std::cout << "Extracted point: (" << pt.x << ", " << pt.y << ", " << pt.z << "), intensity: " << pt.intensity << std::endl;
#endif
            Points.push_back(pt);
        }

        // recurse
        for (int i = 0; i < Prop->numChildren(); ++i)
        {
            extractPoints(Prop->childAt(i), Points);
        }
    }

    bool PcdSelectionPanel::save(const std::string& File, const std::vector<PointXYZI>& Points)
    {
        pcl::PointCloud<pcl::PointXYZI> cloud;

        cloud.width = Points.size();
        cloud.height = 1;
        cloud.is_dense = true;
        cloud.points.resize(Points.size());

        for (size_t i = 0; i < Points.size(); ++i)
        {
            cloud.points[i].x = Points[i].x;
            cloud.points[i].y = Points[i].y;
            cloud.points[i].z = 0.0;
            cloud.points[i].intensity = Points[i].intensity;
        }

        auto res = pcl::io::savePCDFileASCII(File + ".pcd", cloud);

        return res == 0 ? true : false;
    }
} // end namespace whi_rviz_plugins

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(whi_rviz_plugins::PcdSelectionPanel, rviz_common::Panel)
