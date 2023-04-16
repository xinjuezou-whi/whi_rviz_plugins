/******************************************************************
rviz plugin for navigation waypoints

Features:
- waypoints
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_rviz_plugins/panel_waypoints.h"
#include "ui_navi_waypoints.h"

#include <iostream>
#include <cmath>
#include <boost/filesystem.hpp>
#include <ros/package.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <angles/angles.h>
#include <yaml-cpp/yaml.h>

#include <QFileDialog>
#include <QMessageBox>
#include <QTimer>

namespace whi_rviz_plugins
{
	WaypointsPanel::WaypointsPanel(VisualizeWaypoints FuncWaypoints, VisualizeEta FuncEta,
		QWidget* Parent/* = nullptr*/)
		: QWidget(Parent), ui_(new Ui::NaviWaypoints())
		, func_visualize_waypoints_(FuncWaypoints), func_visualize_eta_(FuncEta)
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

		// widget behaviour
		ui_->label_state->setText("Standby");
		ui_->pushButton_execute->setEnabled(false);
		ui_->doubleSpinBox_point_span->setRange(-10800.0, 10800.0); // 3 hours
		ui_->doubleSpinBox_point_span->setValue(-2.0);
		ui_->doubleSpinBox_point_span->setSingleStep(0.1);
		ui_->doubleSpinBox_stop_span->setRange(-10800.0, 10800.0);
		ui_->doubleSpinBox_stop_span->setValue(2.0);
		ui_->doubleSpinBox_stop_span->setSingleStep(0.1);
		QStringList header = { "x", "y", "yaw" };
		ui_->tableWidget_waypoints->setColumnCount(header.size());
		ui_->tableWidget_waypoints->setHorizontalHeaderLabels(header);
		ui_->tableWidget_waypoints->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
		ui_->tableWidget_waypoints->setContextMenuPolicy(Qt::CustomContextMenu);
		ui_->groupBox_waypoints->setTitle(QString("Waypoints (0)"));
		ui_->comboBox_ns->setInsertPolicy(QComboBox::NoInsert);
		// signals
		connect(ui_->pushButton_add, &QPushButton::clicked, this, [=]() { addButtonClicked(); });
		connect(ui_->pushButton_insert, &QPushButton::clicked, this, [=]() { insertButtonClicked(); });
		connect(ui_->pushButton_remove, &QPushButton::clicked, this, [=]() { removeButtonClicked(); });
		connect(ui_->tableWidget_waypoints, &QTableWidget::cellChanged, this, [=](int Row, int Column) { visualizeWaypoints(Row); });
    	connect(ui_->tableWidget_waypoints, &QTableWidget::currentCellChanged, this, [=](int Row, int Column) { visualizeWaypoints(Row); });
		connect(ui_->pushButton_load, &QPushButton::clicked, this, [=]()
		{
			QString fileName = QFileDialog::getOpenFileName(this, tr("Open Waypoints"), "/home/whi", tr("Waypoints Files (*.yaml)"));
			if (!fileName.isNull())
			{
				loadWaypointsNs(fileName.toStdString());
			}
		});
		connect(ui_->pushButton_save, &QPushButton::clicked, this, [=]()
		{
			if (ui_->tableWidget_waypoints->rowCount() > 0)
			{
				QString fileName = QFileDialog::getSaveFileName(this, tr("Save Waypoints"), "/home/whi/untitled.yaml", tr("Waypoints Files (*.yaml)"));
				if (!fileName.isNull())
				{
					if (!fileName.contains(".yaml"))
					{
						fileName += ".yaml";
					}
					saveWaypoints(fileName.toStdString());
				}
			}
		});
		connect(ui_->pushButton_execute, &QPushButton::clicked, this, [=]()
		{
			// re-configure namespace
			goals_->setNamespace(ui_->comboBox_ns->currentText().toStdString(), is_remote_);

			std::vector<geometry_msgs::PoseStamped> waypoints;
			retrieveWaypoints(waypoints);
			std::vector<geometry_msgs::Pose> points;
			for (const auto& it : waypoints)
			{
				points.push_back(it.pose);
			}
			if (goals_->execute(points, ui_->doubleSpinBox_point_span->value(), 
				ui_->doubleSpinBox_stop_span->value(), ui_->checkBox_loop->isChecked()))
			{
				ui_->label_state->setText("Executing...");
				ui_->pushButton_execute->setEnabled(false);
				enableUi(false);
			}
			else
			{
				QMessageBox::critical(this, tr("Error"), tr("failed to send goal"));
				ui_->label_state->setText("Failed to send goal");
			}
		});
		connect(ui_->pushButton_abort, &QPushButton::clicked, this, [=]()
		{
			goals_->cancel();

			ui_->label_state->setText("Standby");
			if (ui_->tableWidget_waypoints->rowCount() > 0)
			{
				ui_->pushButton_execute->setEnabled(true);
			}
			enableUi(true);
		});
		connect(ui_->checkBox_loop, &QCheckBox::stateChanged, this, [=](int State) { goals_->setLooping(State); });
		connect(ui_->doubleSpinBox_point_span, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
			[=](double Value) { goals_->setPointSpan(Value); });
		connect(ui_->doubleSpinBox_stop_span, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
			[=](double Value) { goals_->setStopSpan(Value); });
		connect(ui_->pushButton_add_ns, &QPushButton::clicked, this, [=]()
		{
			if (ui_->comboBox_ns->findText(ui_->comboBox_ns->currentText()) < 0)
			{
				// re-configure namespace
				goals_->setNamespace(ui_->comboBox_ns->currentText().toStdString(), is_remote_);
				// add to map
				storeAll2Map(ui_->comboBox_ns->currentText().toStdString());
				// add to combox
				ui_->comboBox_ns->addItem(ui_->comboBox_ns->currentText().toStdString().c_str());
			}
			else
			{
				QMessageBox::information(this, tr("Info"), tr("namespace is present already"));
			}
		});
		connect(ui_->comboBox_ns, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [=](int Index)
		{
			// re-configure namespace
			goals_->setNamespace(ui_->comboBox_ns->currentText().toStdString(), is_remote_);
			// remove all rows from table
			ui_->tableWidget_waypoints->setRowCount(0);
			// fill all from map to table
			const auto& poseList = waypoints_map_[ui_->comboBox_ns->currentText().toStdString()];
			for (size_t i = 0; i < poseList.size(); ++i)
			{
				std::vector<double> poseVector;
				poseVector.push_back(poseList[i].position.x);
				poseVector.push_back(poseList[i].position.y);
				poseVector.push_back(angles::to_degrees(getYawFromPose(poseList[i])));

				ui_->tableWidget_waypoints->insertRow(i);
				fillWaypoint(i, false, &poseVector);
			}
			visualizeWaypoints(0);
		});

		// multiple goals
		goals_ = std::make_unique<GoalsHandle>();
		goals_->registerEatUpdater(func_visualize_eta_);
		goals_->registerExecutionUpdater(std::bind(&WaypointsPanel::executionState,
			this, std::placeholders::_1, std::placeholders::_2));
		goals_->registerMapReceived(std::bind(&WaypointsPanel::mapTrigger, this));
		
		timer_map_ = new QTimer(this);
		timer_map_->setSingleShot(true);
		connect(timer_map_, &QTimer::timeout, this, [=]()
		{
			QMessageBox::warning(this, tr("Warning"), tr("failed to get current position"));
		});
	}

	WaypointsPanel::~WaypointsPanel()
	{
		delete ui_;
	}

	void WaypointsPanel::setRemoteFlag(bool Flag)
	{
		is_remote_ = Flag;
	}

	void WaypointsPanel::updateWaypoint(int Index, const geometry_msgs::Pose& Pose)
	{
		ui_->tableWidget_waypoints->blockSignals(true);
		ui_->tableWidget_waypoints->setItem(Index, 0, new QTableWidgetItem(QString::number(Pose.position.x)));
		ui_->tableWidget_waypoints->setItem(Index, 1, new QTableWidgetItem(QString::number(Pose.position.y)));
		ui_->tableWidget_waypoints->setItem(Index, 2,
			new QTableWidgetItem(QString::number(angles::to_degrees(getYawFromPose(Pose)))));
		ui_->tableWidget_waypoints->blockSignals(false);

		// synchronize the map
		geometry_msgs::PoseStamped pose;
		retrieveWaypoint(Index, pose);
		waypoints_map_[ui_->comboBox_ns->currentText().toStdString()][Index] = pose.pose;
	}

	void WaypointsPanel::updateHeight(double Height)
	{
		std::vector<geometry_msgs::PoseStamped> waypoints;
		retrieveWaypoints(waypoints);
		for (auto& it : waypoints)
        {
            it.pose.position.z = Height;
        }

		if (func_visualize_waypoints_)
		{
			func_visualize_waypoints_(ui_->tableWidget_waypoints->currentRow(), waypoints);
		}
	}

	void WaypointsPanel::fillWaypoint(int RowIndex, bool WithCurrent/* = false*/, const std::vector<double>* Point/* = nullptr*/)
	{
		ui_->tableWidget_waypoints->blockSignals(true);
		if (WithCurrent)
		{
			geometry_msgs::Pose currentPose = goals_->getCurrentPose();;
			ui_->tableWidget_waypoints->setItem(RowIndex, 0, new QTableWidgetItem(QString::number(currentPose.position.x)));
			ui_->tableWidget_waypoints->setItem(RowIndex, 1, new QTableWidgetItem(QString::number(currentPose.position.y)));
			double yaw = getYawFromPose(currentPose);
			ui_->tableWidget_waypoints->setItem(RowIndex, 2, new QTableWidgetItem(QString::number(angles::to_degrees(yaw))));
		}
		else
		{
			if (Point)
			{
				for (int i = 0; i < std::min(ui_->tableWidget_waypoints->columnCount(), int(Point->size())); ++i)
				{
					ui_->tableWidget_waypoints->setItem(RowIndex, i, new QTableWidgetItem(std::to_string(Point->at(i)).c_str()));
				}
			}
			else
			{
				for (int i = 0; i < ui_->tableWidget_waypoints->columnCount(); ++i)
				{
					ui_->tableWidget_waypoints->setItem(RowIndex, i, new QTableWidgetItem("0.0"));
				}
			}
		}
		ui_->tableWidget_waypoints->blockSignals(false);
		ui_->pushButton_execute->setEnabled(true);
	}

	void WaypointsPanel::retrieveWaypoints(std::vector<geometry_msgs::PoseStamped>& Waypoints) const
	{
  		for (int i = 0; i < ui_->tableWidget_waypoints->rowCount(); ++i)
		{
			geometry_msgs::PoseStamped pose;
			retrieveWaypoint(i, pose);

			Waypoints.push_back(pose);
		}
	}

	void WaypointsPanel::retrieveWaypoint(int Index, geometry_msgs::PoseStamped& Waypoint) const
	{
		Waypoint.pose.position.x = ui_->tableWidget_waypoints->item(Index, 0)->text().toDouble();
		Waypoint.pose.position.y = ui_->tableWidget_waypoints->item(Index, 1)->text().toDouble();
		Waypoint.pose.position.z = 1.0;// TODO

		tf2::Quaternion orientation;
		orientation.setRPY(0.0, 0.0, angles::from_degrees(ui_->tableWidget_waypoints->item(Index, 2)->text().toDouble()));

		Waypoint.pose.orientation = tf2::toMsg(orientation);
	}

	void WaypointsPanel::visualizeWaypoints(int Row) const
	{
		std::vector<geometry_msgs::PoseStamped> waypoints;
		retrieveWaypoints(waypoints);

		if (func_visualize_waypoints_)
		{
			func_visualize_waypoints_(Row, waypoints);
		}
	}

	void WaypointsPanel::addButtonClicked()
	{
		if (ui_->checkBox_current->isChecked())
		{
			trigger_state_ = TRIGGER_ADD;
			// re-configure namespace
			goals_->setNamespace(ui_->comboBox_ns->currentText().toStdString(), is_remote_);

			if (!timer_map_->isActive())
			{
				timer_map_->start(1000);
			}
		}
		else
		{
			addWaypoint();
		}
	}

	void WaypointsPanel::insertButtonClicked()
	{
		if (ui_->checkBox_current->isChecked())
		{
			trigger_state_ = TRIGGER_INSERT;
			// re-configure namespace
			goals_->setNamespace(ui_->comboBox_ns->currentText().toStdString(), is_remote_);

			if (!timer_map_->isActive())
			{
				timer_map_->start(1000);
			}
		}
		else
		{
			insertWaypoint();
		}
	}

	void WaypointsPanel::removeButtonClicked()
	{
		// remove item in map first
		if (auto search = waypoints_map_.find(ui_->comboBox_ns->currentText().toStdString());
			search != waypoints_map_.end())
		{
			auto& poseList = waypoints_map_[ui_->comboBox_ns->currentText().toStdString()];
			poseList.erase(poseList.begin() + ui_->tableWidget_waypoints->currentRow());
		}

		// then remove item in table
		int highlightRow = ui_->tableWidget_waypoints->currentRow() == ui_->tableWidget_waypoints->rowCount() - 1 ?
			ui_->tableWidget_waypoints->rowCount() - 2 : ui_->tableWidget_waypoints->currentRow();
		ui_->tableWidget_waypoints->removeRow(ui_->tableWidget_waypoints->currentRow());
		ui_->groupBox_waypoints->setTitle(QString("Waypoints (") + QString::number(ui_->tableWidget_waypoints->rowCount())
			+ QString(")"));

		ui_->tableWidget_waypoints->setCurrentCell(highlightRow, 0);

		visualizeWaypoints(highlightRow);

		if (ui_->tableWidget_waypoints->rowCount() == 0)
		{
			ui_->pushButton_execute->setEnabled(false);
		}
	}

	void WaypointsPanel::executionState(int State, std::shared_ptr<std::string> Info)
	{
		if (State == GoalsHandle::STA_DONE)
		{
			ui_->pushButton_execute->setEnabled(true);
			enableUi(true);
			ui_->label_state->setText("Standby");
		}
		else if (State == GoalsHandle::STA_POINT_APPROACHED)
		{
			ui_->label_state->setText("Executing..." + QString(Info != nullptr ? Info->c_str() : ""));
		}
	}

	bool WaypointsPanel::loadWaypoints(std::string File)
	{
		try
		{
			YAML::Node node = YAML::LoadFile(File);

			const auto& mapOrigin = node["map"];
			double originX = mapOrigin[0].as<double>();
			double originY = mapOrigin[1].as<double>();

			int ret = QMessageBox::Yes;
			geometry_msgs::Pose origin = goals_->getMapOrigin();
			if (fabs(originX - origin.position.x) > 1e-2 || fabs(originY - origin.position.y) > 1e-2)
			{
				ret = QMessageBox::question(this, tr("Your call"),
                	tr("Map origin is different.\n"
                       "Do you intend to ignore and load?"),
                    QMessageBox::Yes | QMessageBox::Cancel, QMessageBox::Cancel);
			}
			if (ret == QMessageBox::Yes)
			{
				// remove all rows from table
				ui_->tableWidget_waypoints->setRowCount(0);
				// load from yaml
				const auto& points = node["waypoints"];
				for (size_t i = 0; i < points.size(); ++i)
				{
					int index = points[i]["index"].as<int>();

					std::vector<double> pose;
					for (const auto& it : points[i]["pose"])
					{
						pose.push_back(it.as<double>());
					}

					ui_->tableWidget_waypoints->insertRow(i);
					fillWaypoint(i, false, &pose);
					// add to map
					storeItem2Map(i, false);
				}

				ui_->groupBox_waypoints->setTitle(QString("Waypoints (") + QString::number(ui_->tableWidget_waypoints->rowCount())
					+ QString(")"));
				ui_->tableWidget_waypoints->setCurrentCell(0, 0);

				visualizeWaypoints(0);
			}

			return true;
		}
		catch(const std::exception& e)
		{
			std::cout << "failed to load waypoints file " << File << std::endl;
			return false;
		}
	}

	void WaypointsPanel::saveWaypoints(std::string File)
	{
		std::ofstream ofs(File, std::ios::out | std::ios::trunc);
		if (ofs.good())
		{
			// namespace
			if (!ui_->comboBox_ns->currentText().isEmpty())
			{
				std::string line("namespace: " + ui_->comboBox_ns->currentText().toStdString() + "\n");
				ofs.write(line.c_str(), line.length());
			}
			// use the map origin to check if the loaded points meet current map
			geometry_msgs::Pose origin = goals_->getMapOrigin();
			std::string line("map: [" + std::to_string(origin.position.x) + ", " +
				std::to_string(origin.position.y) + "]\nwaypoints:\n");
			ofs.write(line.c_str(), line.length());

			for (int i = 0; i < ui_->tableWidget_waypoints->rowCount(); ++i)
			{
				line.assign("  - index: " + std::to_string(i) + "\n");
				line += "    pose: [" + ui_->tableWidget_waypoints->item(i, 0)->text().toStdString() + ", ";
				line += ui_->tableWidget_waypoints->item(i, 1)->text().toStdString() + ", ";
				line += ui_->tableWidget_waypoints->item(i, 2)->text().toStdString() + "]\n";
				ofs.write(line.c_str(), line.length());
			}

			ofs.close();
		}
	}

	bool WaypointsPanel::loadWaypointsNs(std::string File)
	{
		waypoints_file_ = File;
		try
		{
			YAML::Node node = YAML::LoadFile(waypoints_file_);
			const auto& ns = node["namespace"];
			ns_from_load_ = ns ? ns.as<std::string>() : "";

			int ret = QMessageBox::Yes;
			if (ui_->comboBox_ns->findText(ns_from_load_.c_str()) >= 0)
			{
				if (ui_->tableWidget_waypoints->rowCount() > 0)
				{
					ret = QMessageBox::question(this, tr("Your call"),
               			tr("Waypoints under namespace ") + QString(ns_from_load_.c_str()) + tr(" present.\n") +
						tr("Do you intend to overwrite current ones?"),
                    	QMessageBox::Yes | QMessageBox::Cancel, QMessageBox::Cancel);
				}
			}
			if (ret == QMessageBox::Yes)
			{
				trigger_state_ = TRIGGER_LOAD;
				// re-configure namespace
				goals_->setNamespace(ns_from_load_, is_remote_);

				// wait for map subscriber
				QTimer::singleShot(1000, this, [=]()
				{
					if (trigger_state_ == TRIGGER_LOAD)
					{
						ui_->comboBox_ns->addItem(ns_from_load_.c_str());
						ui_->comboBox_ns->setCurrentText(ns_from_load_.c_str());
						ui_->comboBox_ns->setCurrentIndex(ui_->comboBox_ns->findText(ns_from_load_.c_str()));
						
						loadWaypoints(waypoints_file_);

						trigger_state_ = TRIGGER_NA;
					}
				});
			}

			return true;
		}
		catch(const std::exception& e)
		{
			std::cout << "failed to load waypoints file " << waypoints_file_ << std::endl;
			return false;
		}
	}

	void WaypointsPanel::storeItem2Map(int RowIndex, bool Insert/* = true*/)
	{
		if (auto search = waypoints_map_.find(ui_->comboBox_ns->currentText().toStdString());
			search != waypoints_map_.end())
		{
			// add single item
			geometry_msgs::PoseStamped added;
			retrieveWaypoint(RowIndex, added);
			geometry_msgs::Pose pose;
			pose = added.pose;
			auto& poseList = waypoints_map_[ui_->comboBox_ns->currentText().toStdString()];
			if (Insert)
			{
				poseList.insert(poseList.begin() + RowIndex, pose);
			}
			else
			{
				poseList.push_back(pose);
			}
		}
		else
		{
			// add all
			storeAll2Map(ui_->comboBox_ns->currentText().toStdString());
		}
	}

	void WaypointsPanel::storeAll2Map(const std::string& Namespace)
	{
		std::vector<geometry_msgs::PoseStamped> waypoints;
		retrieveWaypoints(waypoints);
		for (const auto& it : waypoints)
		{
			waypoints_map_[Namespace].push_back(it.pose);
		}
	}

	void WaypointsPanel::addWaypoint()
	{
		ui_->tableWidget_waypoints->insertRow(ui_->tableWidget_waypoints->rowCount());
		fillWaypoint(ui_->tableWidget_waypoints->rowCount() - 1, ui_->checkBox_current->isChecked());
		// add to map
		storeItem2Map(ui_->tableWidget_waypoints->rowCount() - 1, false);
		ui_->groupBox_waypoints->setTitle(QString("Waypoints (") + QString::number(ui_->tableWidget_waypoints->rowCount())
			+ QString(")"));

		ui_->tableWidget_waypoints->setCurrentCell(ui_->tableWidget_waypoints->rowCount() - 1, 0);
	}

	void WaypointsPanel::insertWaypoint()
	{
		ui_->tableWidget_waypoints->insertRow(ui_->tableWidget_waypoints->currentRow());
		fillWaypoint(ui_->tableWidget_waypoints->currentRow() - 1, ui_->checkBox_current->isChecked());
		// add to map
		storeItem2Map(ui_->tableWidget_waypoints->currentRow() - 1);
		ui_->groupBox_waypoints->setTitle(QString("Waypoints (") + QString::number(ui_->tableWidget_waypoints->rowCount())
			+ QString(")"));

		ui_->tableWidget_waypoints->setCurrentCell(ui_->tableWidget_waypoints->currentRow() - 1, 0);
	}

	void WaypointsPanel::mapTrigger()
	{
		switch (trigger_state_)
		{
		case TRIGGER_LOAD:
			ui_->comboBox_ns->addItem(ns_from_load_.c_str());
			ui_->comboBox_ns->setCurrentText(ns_from_load_.c_str());
			loadWaypoints(waypoints_file_);
			break;
		case TRIGGER_ADD:
			if (timer_map_->isActive())
			{
				timer_map_->stop();
			}
			addWaypoint();
			break;
		case TRIGGER_INSERT:
			if (timer_map_->isActive())
			{
				timer_map_->stop();
			}
			insertWaypoint();
			break;
		default:
			break;
		}

		trigger_state_ = TRIGGER_NA;
	}

	double WaypointsPanel::getYawFromPose(const geometry_msgs::Pose& Pose) const
	{
		tf::Quaternion quat(Pose.orientation.x, Pose.orientation.y, Pose.orientation.z, Pose.orientation.w);
		double roll = 0.0, pitch = 0.0, yaw = 0.0;
  		tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

		return std::isnan(yaw) ? 0.0 : yaw;
	}

	void WaypointsPanel::enableUi(bool Flag)
	{
		ui_->comboBox_ns->setEnabled(Flag);
		ui_->pushButton_add_ns->setEnabled(Flag);
		ui_->pushButton_add->setEnabled(Flag);
		ui_->pushButton_insert->setEnabled(Flag);
		ui_->pushButton_remove->setEnabled(Flag);
		ui_->pushButton_load->setEnabled(Flag);
	}
} // end namespace whi_rviz_plugins
