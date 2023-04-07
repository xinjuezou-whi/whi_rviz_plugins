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
#include "whi_rviz_plugins/panel_navi_namespace.h"
#include "ui_navi_multiple_ns.h"

#include <ros/package.h>
#include <iostream>
#include <memory>
#include <boost/filesystem.hpp>

#include <QButtonGroup>
#include <QIcon>
#include <QMessageBox>

namespace whi_rviz_plugins
{
    NaviNsPanel::NaviNsPanel(QWidget* Parent/* = nullptr*/)
        : rviz::Panel(Parent), ui_(new Ui::NaviMultipleNs())
        , node_handle_(std::make_shared<ros::NodeHandle>())
    {
        /// set up the GUI
		ui_->setupUi(this);
        ui_->comboBox_ns->setInsertPolicy(QComboBox::NoInsert);
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

        // signals
        connect(ui_->pushButton_add_ns, &QPushButton::clicked, this, [=]()
		{
			if (ui_->comboBox_ns->findText(ui_->comboBox_ns->currentText()) < 0)
			{
                ui_->comboBox_ns->addItem(ui_->comboBox_ns->currentText());
			}
			else
			{
				QMessageBox::information(this, tr("Info"), tr("namespace is present already"));
			}
		});
        connect(buttonGroup, QOverload<QAbstractButton *>::of(&QButtonGroup::buttonClicked), this,
            [=](QAbstractButton* Button)
            {
                if (ui_->pushButton_initial->isChecked())
                {
                    std::cout << "iiiiiiiiiiiiiiiiiinitial button " << std::endl;
                }
                else if (ui_->pushButton_goal->isChecked())
                {
                    std::cout << "ggggggggggggggggggoal button " << std::endl;
                }
            });
    }

    void NaviNsPanel::load(const rviz::Config& Config)
    {
        //Panel::load(Config);

        auto ns = Config.mapGetChild("whi_navi_ns");
        int countNs = ns.listLength();
        for (int i = 0; i < countNs; ++i)
        {
            ui_->comboBox_ns->addItem(ns.listChildAt(i).getValue().toString());
        }
    }

    void NaviNsPanel::save(rviz::Config Config) const
    {
        //rviz::Panel::save(Config);

        auto child = Config.mapMakeChild("whi_navi_ns");
        for (int i = 0; i < ui_->comboBox_ns->count(); ++i)
        {
            child.listAppendNew().setValue(ui_->comboBox_ns->itemText(i));
        }
    }
} // end namespace whi_rviz_plugins
