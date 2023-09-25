/******************************************************************
navigation goals logic under ROS 1

Features:
- multiple navigation goals logic and process
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_rviz_plugins/goals_handle.h"

#include <tf/tf.h>

#include <thread>

GoalsHandle::GoalsHandle(const std::string& Namespace, bool Remote/* = false*/)
	: node_handle_(std::make_unique<ros::NodeHandle>())
{
	setNamespace(Namespace);
	init(Remote);

	// tf listening
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(buffer_);
}

bool GoalsHandle::execute(const std::vector<geometry_msgs::Pose>& Waypoints, double PointSpan, double StopSpan,
	bool Loop/* = false*/)
{
	std::map<int, std::string> tasks;
	bool res = execute(Waypoints, tasks, PointSpan, StopSpan, Loop);

	return res;
}

bool GoalsHandle::execute(const std::vector<geometry_msgs::Pose>& Waypoints, const std::map<int, std::string>& Tasks,
	double PointSpan, double StopSpan, bool Loop/* = false*/)
{
	goals_list_.clear();

	for (std::size_t i = findBeginIndex(Waypoints); ; i = (i + 1) % Waypoints.size())
	{
		std::string config;
		if (auto search = Tasks.find(i); search != Tasks.end())
		{
			config = Tasks.at(i);
		}
		goals_list_.push_back(std::make_tuple(Waypoints[i], config));

		if (goals_list_.size() == Waypoints.size())
		{
			break;
		}
	}

	point_span_ = PointSpan;
	stop_span_ = StopSpan;
	looping_ = Loop;
	waypoints_num_ = goals_list_.size();
	loop_count_ = 0;

	if (!goals_list_.empty())
	{
		final_goal_ = std::get<0>(goals_list_.back());
		return setGoal(std::get<0>(goals_list_.front()));
	}
	else
	{
		return false;
	}
}

void GoalsHandle::cancel()
{
	cancelGoal();
	goals_list_.clear();
}

void GoalsHandle::reset()
{
	// neutralize the active goal
	active_goal_ = getCurrentPose();
}

void GoalsHandle::setLooping(bool Looping)
{
	looping_ = Looping;
}

void GoalsHandle::setPointSpan(double Span)
{
	point_span_ = Span;
}

void GoalsHandle::setStopSpan(double Span)
{
	stop_span_ = Span;
}

geometry_msgs::Pose GoalsHandle::getMapOrigin() const
{
	return map_origin_;
}

geometry_msgs::Pose GoalsHandle::getCurrentPose()
{
	auto trans = listenTf("map", baselink_frame_, ros::Time(0));
	geometry_msgs::Pose pose;
	pose.position.x = trans.transform.translation.x;
	pose.position.y = trans.transform.translation.y;
	pose.position.z = trans.transform.translation.z;
	pose.orientation = trans.transform.rotation;

	return pose;
}

void GoalsHandle::registerEatUpdater(VisualizeEta Func)
{
	func_eta_ = Func;
}

void GoalsHandle::registerExecutionUpdater(ExecutionState Func)
{
	func_execution_state_ = Func;
}

bool GoalsHandle::isActive() const
{
	return !goals_list_.empty();
}

void GoalsHandle::unbindCallback()
{
	func_eta_ = nullptr;
	func_execution_state_ = nullptr;
}

bool GoalsHandle::isMapReceived()
{
	return map_received_;
}

void GoalsHandle::setNamespace(const std::string& Namespace)
{
	if (!Namespace.empty())
	{
		namespace_ = "/" + Namespace + "/";
	}
	else
	{
		namespace_ = Namespace;
	}
}

void GoalsHandle::init(bool IsRemote/* = false*/)
{
	std::string topicMap = IsRemote ? "map_metadata" : namespace_ + "map_metadata";
	sub_map_data_ = std::make_unique<ros::Subscriber>(node_handle_->subscribe<nav_msgs::MapMetaData>(
		topicMap, 10, std::bind(&GoalsHandle::subCallbackMapData, this, std::placeholders::_1)));
	sub_cmd_vel_ = std::make_unique<ros::Subscriber>(node_handle_->subscribe<geometry_msgs::Twist>(
		namespace_ + "cmd_vel", 10, std::bind(&GoalsHandle::subCallbackCmdVel, this, std::placeholders::_1)));

	movebase_client_ = std::make_unique<MoveBaseClient>(namespace_ + "move_base", true);
}

bool GoalsHandle::setGoal(const geometry_msgs::Pose& Goal)
{
	// wait for the action server to come up
	static size_t waitingCount = 0;
	while (!movebase_client_->waitForServer(ros::Duration(5.0)))
	{
		if (++waitingCount > 3)
		{
			printf("can't set the goal, please check if the action server is on\n");
			return false;
		}
		ROS_INFO("Waiting for the move_base action server to come up");
	}

	move_base_msgs::MoveBaseGoal goalMsg;
	goalMsg.target_pose.header.frame_id = "map";
	goalMsg.target_pose.header.stamp = ros::Time::now();
	goalMsg.target_pose.pose = Goal;

	movebase_client_->sendGoal(goalMsg,
		std::bind(&GoalsHandle::callbackGoalDone, this, std::placeholders::_1, std::placeholders::_2),
		std::bind(&GoalsHandle::callbackGoalActive, this),
		std::bind(&GoalsHandle::callbackGoalFeedback, this, std::placeholders::_1));

	return true;
}

void GoalsHandle::cancelGoal() const
{
	// wait for the action server to come up
	while (!movebase_client_->waitForServer(ros::Duration(5.0)))
	{
		ROS_INFO("Waiting for the move_base action server to come up");
	}
	if (!movebase_client_->getState().isDone())
	{
		movebase_client_->cancelGoal();
	}

	if (func_eta_)
	{
		func_eta_(active_goal_, -2.0);
	}
}

geometry_msgs::TransformStamped GoalsHandle::listenTf(const std::string& DstFrame, const std::string& SrcFrame,
    const ros::Time& Time)
{
    try
    {
        return buffer_.lookupTransform(DstFrame, SrcFrame, Time, ros::Duration(1.0));
    }
    catch (tf2::TransformException &e)
    {
        ROS_ERROR("%s", e.what());
        return geometry_msgs::TransformStamped();
    }
}

void GoalsHandle::handleGoalAndState(const geometry_msgs::Pose& Pose)
{
	double dist = distance(Pose, active_goal_);
	if (goals_list_.empty())
	{
		if (dist < 0.2)
		{
			if (func_execution_state_)
			{
				func_execution_state_(STA_DONE, nullptr);
			}
			if (func_eta_)
			{
				func_eta_(active_goal_, -1.0);
			}
			std::cout << "all goals traversed. remained goals " << goals_list_.size() << std::endl;
		}
	}
	else
	{
		if (active_goal_task_.empty())
		{
			bool isFinalOne = metDistance(active_goal_, final_goal_, 1e-3);
			double tolerance = isFinalOne ? -stop_span_ * current_linear_ : -point_span_ * current_linear_;
			if (dist < tolerance)
			{
				setGoal(std::get<0>(goals_list_.front()));
				updateStateInfo(isFinalOne);

				std::cout << "tolerace reached, proceeding the next. remained goals " << goals_list_.size() << "  " << tolerance << std::endl;
			}
		}
	}

	// eta info shows during running state
	if (current_linear_ > 1e-2 && dist > 0.2)
	{
		if (func_eta_)
		{
			func_eta_(active_goal_, dist / current_linear_);
		}
	}
}

void GoalsHandle::updateStateInfo(bool IsFinalOne)
{
	if (func_execution_state_)
	{
		if (looping_)
		{
			if (IsFinalOne)
			{
				std::shared_ptr<std::string> info = std::make_shared<std::string>(
					std::to_string(++loop_count_) + (loop_count_ > 1 ? " loops proceed" : " loop proceed"));
				func_execution_state_(STA_POINT_APPROACHED, info);
			}
		}
		else
		{
			std::shared_ptr<std::string> info = std::make_shared<std::string>(
				std::to_string(waypoints_num_ - goals_list_.size()) + " approached " +
				std::to_string(goals_list_.size()) + " left");
			func_execution_state_(STA_POINT_APPROACHED, info);
		}
	}
}

void GoalsHandle::subCallbackMapData(const nav_msgs::MapMetaData::ConstPtr& MapData)
{
	map_origin_ = MapData->origin;
	map_received_ = true;
}

void GoalsHandle::subCallbackCmdVel(const geometry_msgs::Twist::ConstPtr& CmdVel)
{
	current_linear_ = CmdVel->linear.x;
}

void GoalsHandle::callbackGoalDone(const actionlib::SimpleClientGoalState& State, const move_base_msgs::MoveBaseResultConstPtr& Result)
{
#ifdef DEBUG
	std::cout << "goal state " << std::to_string(State.state_) << " goal left " << goals_list_.size() << std::endl;
#endif
	if (goals_list_.empty())
	{
		// execute task if there is
		if (movebase_client_->getState().state_ == actionlib::SimpleClientGoalState::SUCCEEDED &&
			!active_goal_task_.empty())
		{
			if (task_plugin_)
			{
				task_plugin_->process(active_goal_task_);
			}
		}
		// then set finish state
		if (func_execution_state_)
		{
			func_execution_state_(STA_DONE, nullptr);
		}
	}
	else
	{
		if (active_goal_task_.empty())
		{
			if (int(point_span_) >= 0 || int(stop_span_) >= 0)
			{
				double pointSpan = point_span_ < 0.0 ? 0.1 : point_span_;
				double stopSpan = stop_span_ < 0.0 ? 0.1 : stop_span_;
				bool isFinalOne = metDistance(active_goal_, final_goal_, 1e-3);
				ros::Duration duration = isFinalOne ? ros::Duration(stopSpan) : ros::Duration(pointSpan);
				non_realtime_loop_ = std::make_unique<ros::Timer>(
					node_handle_->createTimer(duration,
					std::bind(&GoalsHandle::callbackTimer, this, std::placeholders::_1)));
			}
		}
		else
		{
			// execute task then to approach the next waypoint
			// IMPORTANT: DO NOT CALL ACTION in its own callback
			std::thread{ &GoalsHandle::executeTask, this }.detach();
		}
	}
}

void GoalsHandle::callbackGoalActive()
{
	active_goal_ = std::get<0>(goals_list_.front());
	active_goal_task_ = std::get<1>(goals_list_.front());
	goals_list_.pop_front();
	if (looping_)
	{
		goals_list_.push_back(std::make_tuple(active_goal_, active_goal_task_));
	}
	else
	{
		bool isFinalOne = metDistance(active_goal_, final_goal_, 1e-3);
		if (isFinalOne)
		{
			goals_list_.clear();
		}
	}
#ifdef DEBUG
	std::cout << "goal left count " << goals_list_.size() << std::endl;
#endif
}

void GoalsHandle::callbackGoalFeedback(const move_base_msgs::MoveBaseFeedbackConstPtr& Feedback)
{
	handleGoalAndState(Feedback->base_position.pose);
}

void GoalsHandle::callbackTimer(const ros::TimerEvent& Event)
{
	if (!goals_list_.empty())
	{
		setGoal(std::get<0>(goals_list_.front()));
		std::cout << "span timeout, proceeding the next" << std::endl;	
	}

	non_realtime_loop_->stop();
	non_realtime_loop_ = nullptr;
}

int GoalsHandle::findBeginIndex(const std::vector<geometry_msgs::Pose>& Waypoints)
{
	int beginIndex = 0;
	for (std::size_t i = 0; i < Waypoints.size(); ++i)
	{
		if (distance(getCurrentPose(), Waypoints[i]) > 1.0)
		{
			// otherwise check the aborted goal if there is
			if (metDistance(active_goal_, Waypoints[i], 1e-3))
			{
				beginIndex = i;
				break;
			}
		}
		else
		{
			// choose the next one if current pose overlays the waypoint
			beginIndex = (i + 1) % Waypoints.size();
			break;
		}
	}

	return beginIndex;
}

void GoalsHandle::executeTask()
{
	if (task_plugin_)
	{
		task_plugin_->process(active_goal_task_);
	}
	if (!goals_list_.empty())
	{
		setGoal(std::get<0>(goals_list_.front()));
	}
}

void GoalsHandle::setTaskPlugin(boost::shared_ptr<whi_rviz_plugins::BasePlugin> Plugin)
{
	task_plugin_ = Plugin;
}

void GoalsHandle::setBaselinkFrame(const std::string& Frame)
{
	baselink_frame_ = Frame;
}

bool GoalsHandle::metDistance(const geometry_msgs::Pose& Pose1, const geometry_msgs::Pose& Pose2, double Tolerance)
{
	return fabs(Pose1.position.x - Pose2.position.x) < Tolerance && fabs(Pose1.position.y - Pose2.position.y) < Tolerance;
}

double GoalsHandle::distance(const geometry_msgs::Pose& Pose1, const geometry_msgs::Pose& Pose2)
{
	return sqrt(pow(Pose1.position.x - Pose2.position.x, 2.0) + pow(Pose1.position.y - Pose2.position.y, 2.0));
}
