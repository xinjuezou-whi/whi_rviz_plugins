/******************************************************************
rviz plugin for navigation waypoints

Features:
- waypoints
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

Apache License Version 2.0, check LICENSE for more information.
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

		// plugins
		loadPlugin(path.string() + "/config/config.yaml", ui_->comboBox_ns->currentText().toStdString());

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
		if (plugins_map_[task_plugin_name_])
		{
			header.push_back("task");
		}
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
#ifdef DEBUG
			fileName = "/home/whi/detection.yaml";
#endif
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
#ifdef DEBUG
				fileName = "/home/whi/detection.yaml";
#endif
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
			if (isBypassed())
			{
				return;
			}

			// re-configure namespace
			std::string ns = ui_->comboBox_ns->currentText().toStdString();
			configureNs(ns);

			std::vector<geometry_msgs::PoseStamped> waypoints;
			retrieveWaypoints(waypoints);
			std::vector<geometry_msgs::Pose> points;
			for (const auto& it : waypoints)
			{
				points.push_back(it.pose);
			}

			if (tasks_map_[ns].empty())
			{
				if (goals_map_[ns]->execute(points,
					ui_->doubleSpinBox_point_span->value(), ui_->doubleSpinBox_stop_span->value(),
					ui_->checkBox_loop->isChecked()))
				{
					ui_->label_state->setText("Executing...");
					enableUi(false);
				}
				else
				{
					QMessageBox::critical(this, tr("Error"), tr("failed to send goal"));
					ui_->label_state->setText("Failed to send goal");
				}
			}
			else
			{
				if (goals_map_[ns]->execute(points, tasks_map_[ns],
					ui_->doubleSpinBox_point_span->value(), ui_->doubleSpinBox_stop_span->value(),
					ui_->checkBox_loop->isChecked()))
				{
					ui_->label_state->setText("Executing...");
					enableUi(false);
				}
				else
				{
					QMessageBox::critical(this, tr("Error"), tr("failed to send goal"));
					ui_->label_state->setText("Failed to send goal");
				}
			}
		});
		connect(ui_->pushButton_abort, &QPushButton::clicked, this, [=]()
		{
			if (!isBypassed())
			{
				abort();
			}
		});
		connect(ui_->checkBox_loop, &QCheckBox::stateChanged, this, [=](int State)
		{
			if (goals_map_[ui_->comboBox_ns->currentText().toStdString()])
			{
				goals_map_[ui_->comboBox_ns->currentText().toStdString()]->setLooping(State);
			}
		});
		connect(ui_->doubleSpinBox_point_span, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
			[=](double Value)
			{
				if (goals_map_[ui_->comboBox_ns->currentText().toStdString()])
				{
					goals_map_[ui_->comboBox_ns->currentText().toStdString()]->setPointSpan(Value);
				}
			});
		connect(ui_->doubleSpinBox_stop_span, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
			[=](double Value)
			{
				if (goals_map_[ui_->comboBox_ns->currentText().toStdString()])
				{
					goals_map_[ui_->comboBox_ns->currentText().toStdString()]->setStopSpan(Value);
				}
			});
		connect(ui_->pushButton_add_ns, &QPushButton::clicked, this, [=]()
		{
			if (ui_->comboBox_ns->findText(ui_->comboBox_ns->currentText()) < 0)
			{
				// re-configure namespace
				configureNs(ui_->comboBox_ns->currentText().toStdString());
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
			configureNs(ui_->comboBox_ns->currentText().toStdString());
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
				bindTaskPlugin(i);
				fillWaypoint(i, false, &poseVector);
			}
			visualizeWaypoints(0);

			enableUi(!goals_map_[ui_->comboBox_ns->currentText().toStdString()]->isActive());
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

	void WaypointsPanel::setBaselinkFrame(const std::string& Frame)
	{
		baselink_frame_ = Frame;
		for (auto& it : goals_map_)
		{
			if (it.second)
			{
				it.second->setBaselinkFrame(baselink_frame_);
			}
		}
	}

	void WaypointsPanel::setStuckTimeout(double Timeout)
	{
		stuck_timeout_ = Timeout;
		for (auto& it : goals_map_)
		{
			if (it.second)
			{
				it.second->setStuckTimeout(stuck_timeout_);
			}
		}
	}

	void WaypointsPanel::setRecoveryMaxTryCount(int Count)
	{
		recovery_max_try_count_ = Count;
		for (auto& it : goals_map_)
		{
			if (it.second)
			{
				it.second->setRecoveryMaxTryCount(recovery_max_try_count_);
			}
		}
	}

	void WaypointsPanel::setTolerance(double XyTolerance, double YawTolerance)
	{
		xy_goal_tolerance_ = XyTolerance;
		yaw_goal_tolerance_ = YawTolerance;
		for (auto& it : goals_map_)
		{
			if (it.second)
			{
				it.second->setTolerance(xy_goal_tolerance_, yaw_goal_tolerance_);
			}
		}		
	}

	void WaypointsPanel::setMotionStateTopic(const std::string& Topic)
	{
		if (!Topic.empty())
		{
			node_handle_ = std::make_unique<ros::NodeHandle>();
        	sub_motion_state_ = std::make_unique<ros::Subscriber>(
            	node_handle_->subscribe<whi_interfaces::WhiMotionState>(Topic, 10,
            	std::bind(&WaypointsPanel::subCallbackMotionState, this, std::placeholders::_1)));
		}
	}

	void WaypointsPanel::setRcStateTopic(const std::string& Topic)
	{
		if (!Topic.empty())
		{
			node_handle_ = std::make_unique<ros::NodeHandle>();
        	sub_rc_state_ = std::make_unique<ros::Subscriber>(
            	node_handle_->subscribe<whi_interfaces::WhiRcState>(Topic, 10,
            	std::bind(&WaypointsPanel::subCallbackRcState, this, std::placeholders::_1)));
		}
	}

	void WaypointsPanel::configureNs(const std::string& Namespace)
	{
		if (pre_ns_ != Namespace)
		{
			// only visualize the info of one namespace
			if (goals_map_[pre_ns_])
			{
				goals_map_[pre_ns_]->unbindCallback();
			}
			pre_ns_ = Namespace;
		}

		if (!goals_map_[Namespace])
		{
			goals_map_[Namespace] = std::make_unique<GoalsHandle>(Namespace, is_remote_);
			goals_map_[Namespace]->setBaselinkFrame(baselink_frame_);
			goals_map_[Namespace]->setStuckTimeout(stuck_timeout_);
		}
		else
		{
			goals_map_[Namespace]->unbindCallback();
		}
		goals_map_[Namespace]->registerEatUpdater(func_visualize_eta_);
		goals_map_[Namespace]->registerExecutionUpdater(std::bind(&WaypointsPanel::executionState,
			this, std::placeholders::_1, std::placeholders::_2));
		if (plugins_map_[task_plugin_name_])
		{
			plugins_map_[task_plugin_name_]->updateNamespace(Namespace);
			goals_map_[Namespace]->setTaskPlugin(plugins_map_[task_plugin_name_]);
		}
	}

	void WaypointsPanel::fillWaypoint(int RowIndex, bool WithCurrent/* = false*/, const std::vector<double>* Point/* = nullptr*/)
	{
		ui_->tableWidget_waypoints->blockSignals(true);
		if (WithCurrent)
		{
			geometry_msgs::Pose currentPose = goals_map_[ui_->comboBox_ns->currentText().toStdString()]->getCurrentPose();
			ui_->tableWidget_waypoints->setItem(RowIndex, 0, new QTableWidgetItem(QString::number(currentPose.position.x)));
			ui_->tableWidget_waypoints->setItem(RowIndex, 1, new QTableWidgetItem(QString::number(currentPose.position.y)));
			double yaw = getYawFromPose(currentPose);
			ui_->tableWidget_waypoints->setItem(RowIndex, 2, new QTableWidgetItem(QString::number(angles::to_degrees(yaw))));
		}
		else
		{
			int colCount = plugins_map_[task_plugin_name_] ?
				ui_->tableWidget_waypoints->columnCount() - 1 : ui_->tableWidget_waypoints->columnCount();
			if (Point)
			{
				for (int i = 0; i < std::min(colCount, int(Point->size())); ++i)
				{
					ui_->tableWidget_waypoints->setItem(RowIndex, i, new QTableWidgetItem(std::to_string(Point->at(i)).c_str()));
				}
			}
			else
			{
				for (int i = 0; i < colCount; ++i)
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
			// re-configure namespace
			std::string ns = ui_->comboBox_ns->currentText().toStdString();
			configureNs(ns);
			// wait for map subscriber
			QTimer::singleShot(500, this, [=]()
			{
				if (!goals_map_[ns]->isMapReceived())
				{
					QMessageBox::warning(this, tr("Warning"), tr("failed to get current position"));
				}

				addWaypoint();
			});
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
			// re-configure namespace
			configureNs(ui_->comboBox_ns->currentText().toStdString());
			// wait for map subscriber
			QTimer::singleShot(500, this, [=]()
			{
				if (!goals_map_[ui_->comboBox_ns->currentText().toStdString()]->isMapReceived())
				{
					QMessageBox::warning(this, tr("Warning"), tr("failed to get current position"));
				}

				insertWaypoint();
			});
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

		refreshTasksMap();

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
			std::string ns = ui_->comboBox_ns->currentText().toStdString();
			geometry_msgs::Pose origin = goals_map_[ns]->getMapOrigin();
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

					if (plugins_map_[task_plugin_name_])
					{
						auto btnTask = bindTaskPlugin(index);

						const auto& taskFile = points[i]["task_file"];
						if (taskFile)
						{
							tasks_map_[ns][index] = taskFile.as<std::string>();
						}
						if (btnTask && !tasks_map_[ns][index].empty())
						{
							plugins_map_[task_plugin_name_]->addTask(tasks_map_[ns][index]);
							btnTask->setToolTip(tasks_map_[ns][index].c_str());
							btnTask->setText("Remove");
						}
					}
				}

				ui_->groupBox_waypoints->setTitle(QString("Waypoints (") + QString::number(ui_->tableWidget_waypoints->rowCount())
					+ QString(")"));
				ui_->tableWidget_waypoints->setCurrentCell(0, 0);

				visualizeWaypoints(0);

				if (goals_map_[ns])
				{
					goals_map_[ns]->reset();
				}
			}

			return true;
		}
		catch (const std::exception& e)
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
			std::string ns = ui_->comboBox_ns->currentText().toStdString();
			if (!ui_->comboBox_ns->currentText().isEmpty())
			{
				std::string line("namespace: " + ns + "\n");
				ofs.write(line.c_str(), line.length());
			}
			// use the map origin to check if the loaded points meet current map
			geometry_msgs::Pose origin;
			if (goals_map_[ns])
			{
				origin = goals_map_[ns]->getMapOrigin();
			}
			std::string line("map: [" + std::to_string(origin.position.x) + ", " +
				std::to_string(origin.position.y) + "]\nwaypoints:\n");
			ofs.write(line.c_str(), line.length());

			for (int i = 0; i < ui_->tableWidget_waypoints->rowCount(); ++i)
			{
				line.assign("  - index: " + std::to_string(i) + "\n");
				line += "    pose: [" + ui_->tableWidget_waypoints->item(i, 0)->text().toStdString() + ", ";
				line += ui_->tableWidget_waypoints->item(i, 1)->text().toStdString() + ", ";
				line += ui_->tableWidget_waypoints->item(i, 2)->text().toStdString() + "]\n";
				if (!tasks_map_[ns][i].empty())
				{
					line += "    task_file: " + tasks_map_[ns][i] + "\n";
				}
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
			if (nsExisted(ns_from_load_))
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
				// re-configure namespace
				configureNs(ns_from_load_);

				// wait for map subscriber
				QTimer::singleShot(500, this, [=]()
				{
					if (!this->nsExisted(ns_from_load_))
					{
						ui_->comboBox_ns->addItem(ns_from_load_.c_str());
					}
					ui_->comboBox_ns->setCurrentText(ns_from_load_.c_str());
						
					loadWaypoints(waypoints_file_);
				});
			}

			return true;
		}
		catch (const std::exception& e)
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
		bindTaskPlugin(ui_->tableWidget_waypoints->rowCount() - 1);
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
		bindTaskPlugin(ui_->tableWidget_waypoints->currentRow() - 1);
		refreshTasksMap();
		fillWaypoint(ui_->tableWidget_waypoints->currentRow() - 1, ui_->checkBox_current->isChecked());
	
		// add to map
		storeItem2Map(ui_->tableWidget_waypoints->currentRow() - 1);
		ui_->groupBox_waypoints->setTitle(QString("Waypoints (") + QString::number(ui_->tableWidget_waypoints->rowCount())
			+ QString(")"));

		ui_->tableWidget_waypoints->setCurrentCell(ui_->tableWidget_waypoints->currentRow() - 1, 0);
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
		if (Flag)
		{
			if (ui_->tableWidget_waypoints->rowCount() > 0)
			{
				ui_->pushButton_execute->setEnabled(Flag);
			}
		}
		else
		{
			ui_->pushButton_execute->setEnabled(Flag);
		}
		ui_->pushButton_add_ns->setEnabled(Flag);
		ui_->pushButton_add->setEnabled(Flag);
		ui_->pushButton_insert->setEnabled(Flag);
		ui_->pushButton_remove->setEnabled(Flag);
		ui_->pushButton_load->setEnabled(Flag);
	}

	bool WaypointsPanel::nsExisted(const std::string& Namespace) const
	{
		return ui_->comboBox_ns->findText(Namespace.c_str()) >= 0;
	}

	bool WaypointsPanel::loadPlugin(const std::string& Config, const std::string& Namespace)
	{
		try
    	{
			bool res = true;
        	YAML::Node node = YAML::LoadFile(Config);
			const auto& taskPlugin = node["tasks_plugin"];
			if (taskPlugin)
			{
				const auto& name = taskPlugin["name"];
				if (name)
				{
					task_plugin_name_ = name.as<std::string>();
					createTaskPlugin(taskPlugin, Namespace);
				}
				else
				{
					res = false;
				}
			}

        	return res;
    	}
    	catch (const std::exception& e)
    	{
        	ROS_ERROR_STREAM("failed to load config file " << Config);
        	return false;
    	}
	}

	bool WaypointsPanel::createTaskPlugin(const YAML::Node& Node, const std::string& Namespace)
	{
        try
        {
			if (!plugins_map_[task_plugin_name_])
			{
				plugin_loader_ = std::make_unique<pluginlib::ClassLoader<BasePlugin>>
					("whi_rviz_plugins", "whi_rviz_plugins::BasePlugin");
				plugins_map_[task_plugin_name_] = plugin_loader_->createInstance(task_plugin_name_);
				plugins_map_[task_plugin_name_]->initialize(Node, Namespace);
			}
            
            ROS_INFO_STREAM("created plugin " << task_plugin_name_);

			return true;
        }
        catch (const pluginlib::PluginlibException& ex)
        {
            ROS_FATAL_STREAM("failed to create the " << task_plugin_name_ <<
                "are you sure it is properly registered and that the containing library is built? Exception: " <<
				ex.what());

			return false;
        }
	}

	QPushButton* WaypointsPanel::bindTaskPlugin(int Row)
	{
		// add task button
		if (plugins_map_[task_plugin_name_])
		{
			QWidget* general = new QWidget();
			QPushButton* btnTask = new QPushButton();
			btnTask->setText("Load");
			QHBoxLayout* hbox = new QHBoxLayout(general);
			hbox->addWidget(btnTask);
			hbox->setAlignment(Qt::AlignCenter);
			hbox->setContentsMargins(0, 0, 0, 0);
			general->setLayout(hbox);
			ui_->tableWidget_waypoints->setCellWidget(Row, 3, general);

			connect(btnTask, &QPushButton::clicked, this, [=]()
			{
				std::string ns = ui_->comboBox_ns->currentText().toStdString();

				if (btnTask->text() == "Load")
				{
					QString fileName = QFileDialog::getOpenFileName(this, tr("Open tasks"), "/home/whi",
						tr("Tasks Files (*.yaml)"));
					if (!fileName.isNull() && plugins_map_[task_plugin_name_]->addTask(fileName.toStdString()))
					{
						tasks_map_[ns][Row] = fileName.toStdString();
						btnTask->setToolTip(tasks_map_[ns][Row].c_str());
						btnTask->setText("Remove");
					}
				}
				else if (btnTask->text() == "Remove")
				{
					tasks_map_[ns][Row].clear();
					btnTask->setToolTip("");
					btnTask->setText("Load");
				}
			});

			return btnTask;
		}

		return nullptr;
	}

	void WaypointsPanel::refreshTasksMap()
	{
		for (int i = 0; i < ui_->tableWidget_waypoints->rowCount(); ++i)
		{
			auto widget = ui_->tableWidget_waypoints->cellWidget(i, 3);
			auto btn = widget->layout()->itemAt(0)->widget();
			tasks_map_[ui_->comboBox_ns->currentText().toStdString()][i] = btn->toolTip().toStdString();
		}
	}

	void WaypointsPanel::subCallbackMotionState(const whi_interfaces::WhiMotionState::ConstPtr& MotionState)
    {
        if (MotionState->state == whi_interfaces::WhiMotionState::STA_ESTOP)
	    {
		    if (!toggle_estop_.load())
		    {
			    abort();
		    }
		    toggle_estop_.store(true);
	    }
	    else if (MotionState->state == whi_interfaces::WhiMotionState::STA_ESTOP_CLEAR)
	    {
		    toggle_estop_.store(false);
	    }

        if (MotionState->state == whi_interfaces::WhiMotionState::STA_CRITICAL_COLLISION)
	    {
		    if (!toggle_collision_.load())
		    {
			    abort();
		    }
		    toggle_collision_.store(true);
	    }
	    else if (MotionState->state == whi_interfaces::WhiMotionState::STA_CRITICAL_COLLISION_CLEAR)
	    {
		    toggle_collision_.store(false);
	    }
    }

	void WaypointsPanel::subCallbackRcState(const whi_interfaces::WhiRcState::ConstPtr& RcState)
	{
        if (RcState->state == whi_interfaces::WhiRcState::STA_REMOTE)
        {
            if (!remote_mode_.load())
            {
                abort();
            }
            remote_mode_.store(true);
        }
        else if (RcState->state == whi_interfaces::WhiRcState::STA_AUTO)
        {
            remote_mode_.store(false);
        }
	}

	void WaypointsPanel::abort()
	{
		if (goals_map_[ui_->comboBox_ns->currentText().toStdString()])
		{
			goals_map_[ui_->comboBox_ns->currentText().toStdString()]->cancel();
		}

		ui_->label_state->setText("Standby");
		enableUi(true);
	}

	bool WaypointsPanel::isBypassed()
    {
        if (toggle_estop_.load())
        {
            QMessageBox::information(this, tr("Info"), tr("E-Stop detected, command is ignored"));
        }
        else if (toggle_collision_.load())
        {
            QMessageBox::information(this, tr("Info"), tr("critical collision detected, command is ignored"));
        }
        else if (remote_mode_.load())
        {
            QMessageBox::information(this, tr("Info"), tr("vehicle is in remote mode, command is ignored"));
        }
        else
        {
            return false;
        }

        return true;
    }
} // end namespace whi_rviz_plugins
