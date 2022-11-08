/******************************************************************
navigation goals logic under ROS 1

Features:
- multiple navigation goals logic and process
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_rviz_plugins/goals_handle.h"

#include <tf/tf.h>

GoalsHandle::GoalsHandle()
	: node_handle_(std::make_unique<ros::NodeHandle>())
{
	std::string model;
	with_ux_ = node_handle_->getParam("/whi_navigation_ux/robot_model", model);

    sub_planned_path_ = std::make_unique<ros::Subscriber>(node_handle_->subscribe<nav_msgs::Path>(
		"/move_base/NavfnROS/plan", 10, std::bind(&GoalsHandle::subCallbackPlanPath, this, std::placeholders::_1)));
	sub_map_data_ = std::make_unique<ros::Subscriber>(node_handle_->subscribe<nav_msgs::MapMetaData>(
		"map_metadata", 10, std::bind(&GoalsHandle::subCallbackMapData, this, std::placeholders::_1)));
	sub_estimate_ = std::make_unique<ros::Subscriber>(node_handle_->subscribe<geometry_msgs::PoseWithCovarianceStamped>(
			"/amcl_pose", 10, std::bind(&GoalsHandle::subCallbackEstimated, this, std::placeholders::_1)));
	sub_cmd_vel_ = std::make_unique<ros::Subscriber>(node_handle_->subscribe<geometry_msgs::Twist>(
			"/cmd_vel", 10, std::bind(&GoalsHandle::subCallbackCmdVel, this, std::placeholders::_1)));
}

void GoalsHandle::execute(std::vector<geometry_msgs::Pose> Waypoints, double PointSpan, double StopSpan, bool Loop/* = false*/)
{
	goals_list_.clear();
	std::size_t metIndex = 0;
	for (std::size_t i = 0; i < Waypoints.size(); ++i)
	{
		if (metDistance(active_goal_, Waypoints[i], 1e-3))
		{
			metIndex = i;
			break;
		}
	}
	for (std::size_t i = metIndex; ; i = (i + 1) % Waypoints.size())
	{
		goals_list_.push_back(Waypoints[i]);
		if (goals_list_.size() == Waypoints.size())
		{
			break;
		}
	}
	point_span_ = PointSpan;
	stop_span_ = StopSpan;
	looping_ = Loop;

	if (!goals_list_.empty())
	{
		final_goal_ = goals_list_.back();
		setGoal(goals_list_.front());
	}
}

void GoalsHandle::cancel()
{
	cancelGoal();
	goals_list_.clear();
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
	mtx_estimated_.lock();
	geometry_msgs::Pose estimated = estimated_;
	mtx_estimated_.unlock();

	return estimated;
}

void GoalsHandle::registerEatUpdater(VisualizeEta Func)
{
	func_eta_ = Func;
}

void GoalsHandle::registerExecutionUpdater(ExecutionState Func)
{
	func_execution_state_ = Func;
}

void GoalsHandle::setGoal(const geometry_msgs::Pose& Goal)
{
	static size_t waitingCount = 0;
	if (movebase_client_ == nullptr)
	{
		movebase_client_ = std::make_unique<MoveBaseClient>("move_base", true);
	}
	// wait for the action server to come up
	while (!movebase_client_->waitForServer(ros::Duration(5.0)))
	{
		if (++waitingCount > 3)
		{
			printf("can't set the goal, please check if the action server is on\n");
			break;
		}
		ROS_INFO("Waiting for the move_base action server to come up");
	}

	move_base_msgs::MoveBaseGoal goalMsg;
	goalMsg.target_pose.header.frame_id = "map";
	goalMsg.target_pose.header.stamp = ros::Time::now();
	goalMsg.target_pose.pose  = Goal;

	movebase_client_->sendGoal(goalMsg, std::bind(&GoalsHandle::callbackGoalDone, this, std::placeholders::_1, std::placeholders::_2),
		std::bind(&GoalsHandle::callbackGoalActive, this),
		std::bind(&GoalsHandle::callbackGoalFeedback, this, std::placeholders::_1));

	// waitForResult will block until the move_base action is done processing the goal we sent it
	// refer to http://wiki.ros.org/actionlib
	// and http://wiki.ros.org/actionlib/Tutorials
	//movebase_client_->waitForResult();
	//if (movebase_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	//{
	//	ROS_INFO("Hooray, the base moved 1 meter forward");
	//}
	//else
	//{
	//	ROS_INFO("The base failed to move forward 1 meter for some reason");
	//}
}

void GoalsHandle::cancelGoal() const
{
	auto pubCancel = std::make_unique<ros::Publisher>(node_handle_->advertise<actionlib_msgs::GoalID>("move_base/cancel", 10));
	actionlib_msgs::GoalID cancelID; // must be an empty goal msg

	pubCancel->publish(cancelID);
}

void GoalsHandle::handleGoalAndState(const geometry_msgs::Pose& Pose)
{
	bool isFinalOne = metDistance(active_goal_, final_goal_, 1e-3);
	double tolerance = isFinalOne ? -stop_span_ * current_linear_ : -point_span_ * current_linear_;
	double dist = distance(Pose, active_goal_);
	if (tolerance > 0.0)
	{
		if (dist < tolerance && !goals_list_.empty())
		{
			setGoal(goals_list_.front());
			std::cout << "tolerace reached, proceeding the next. remained goals " << goals_list_.size() << "  " << tolerance << std::endl;
		}
	}
	else
	{
		if (with_ux_ && !goals_list_.empty())
		{
			bool isFinalOne = metDistance(active_goal_, final_goal_, 1e-3);
			ros::Duration duration = isFinalOne ? ros::Duration(stop_span_) : ros::Duration(point_span_);
			non_realtime_loop_ = std::make_unique<ros::Timer>(
				node_handle_->createTimer(duration, std::bind(&GoalsHandle::callbackTimer, this, std::placeholders::_1)));
		}
	}
	if (current_linear_ > 1e-3 && dist > 0.2)
	{
		if (func_eta_)
		{
			func_eta_(active_goal_, dist / current_linear_);
		}
	}
	if (goals_list_.empty() && dist < 0.2)
	{
		if (func_execution_state_)
		{
			func_execution_state_(STA_DONE);
		}
		if (func_eta_)
		{
			double eta = with_ux_ ? -2.0 : -1.0;
			func_eta_(active_goal_, eta);
		}
		std::cout << "all goals traversed. remained goals " << goals_list_.size() << std::endl;
	}
}

void GoalsHandle::subCallbackPlanPath(const nav_msgs::Path::ConstPtr& PlanPath)
{
	traj_poses_ = PlanPath->poses;
}

void GoalsHandle::subCallbackMapData(const nav_msgs::MapMetaData::ConstPtr& MapData)
{
	map_origin_ = MapData->origin;
}

void GoalsHandle::subCallbackEstimated(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& Estimated)
{
	mtx_estimated_.lock();
	estimated_ = Estimated->pose.pose;
	mtx_estimated_.unlock();

	tf::Quaternion quat(estimated_.orientation.x, estimated_.orientation.y, estimated_.orientation.z, estimated_.orientation.w);
  	double roll = 0.0, pitch = 0.0, yaw = 0.0;
  	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

	if (with_ux_)
	{
		handleGoalAndState(estimated_);
	}
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
	if (!goals_list_.empty() && (point_span_ > 0.0 || stop_span_ > 0.0))
	{
		if (!with_ux_)
		{
			bool isFinalOne = metDistance(active_goal_, final_goal_, 1e-3);
			ros::Duration duration = isFinalOne ? ros::Duration(stop_span_) : ros::Duration(point_span_);
			non_realtime_loop_ = std::make_unique<ros::Timer>(
				node_handle_->createTimer(duration, std::bind(&GoalsHandle::callbackTimer, this, std::placeholders::_1)));
		}
	}
	else if (goals_list_.empty())
	{
		if (func_execution_state_)
		{
			func_execution_state_(STA_DONE);
		}
	}
}

void GoalsHandle::callbackGoalActive()
{
	active_goal_ = goals_list_.front();
	goals_list_.pop_front();
	if (looping_)
	{
		goals_list_.push_back(active_goal_);
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
	if (!with_ux_)
	{
		handleGoalAndState(Feedback->base_position.pose);
	}
}

void GoalsHandle::callbackTimer(const ros::TimerEvent& Event)
{
	setGoal(goals_list_.front());
	std::cout << "span timeout, proceeding the next" << std::endl;	

	non_realtime_loop_->stop();
	non_realtime_loop_ = nullptr;
}

bool GoalsHandle::metDistance(const geometry_msgs::Pose& Pose1, const geometry_msgs::Pose& Pose2, double Tolerance)
{
	return fabs(Pose1.position.x - Pose2.position.x) < Tolerance && fabs(Pose1.position.y - Pose2.position.y) < Tolerance;
}

double GoalsHandle::distance(const geometry_msgs::Pose& Pose1, const geometry_msgs::Pose& Pose2)
{
	return sqrt(pow(Pose1.position.x - Pose2.position.x, 2.0) + pow(Pose1.position.y - Pose2.position.y, 2.0));
}
