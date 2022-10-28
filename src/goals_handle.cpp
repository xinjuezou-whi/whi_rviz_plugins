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
    sub_planned_path_ = std::make_unique<ros::Subscriber>(node_handle_->subscribe<nav_msgs::Path>(
		"/move_base/NavfnROS/plan", 10, std::bind(&GoalsHandle::subCallbackPlanPath, this, std::placeholders::_1)));
	sub_map_data_ = std::make_unique<ros::Subscriber>(node_handle_->subscribe<nav_msgs::MapMetaData>(
		"map_metadata", 10, std::bind(&GoalsHandle::subCallbackMapData, this, std::placeholders::_1)));
	sub_odom_ = std::make_unique<ros::Subscriber>(node_handle_->subscribe<geometry_msgs::PoseWithCovarianceStamped>(
			"/amcl_pose", 10, std::bind(&GoalsHandle::subCallbackEstimated, this, std::placeholders::_1)));
}

void GoalsHandle::execute(std::vector<geometry_msgs::Pose> Waypoints, double PointSpan, double StopSpan, bool Loop/* = false*/)
{
	goals_list_.clear();
	for (const auto& it : Waypoints)
	{
		goals_list_.push_back(it);
	}
	point_span_ = PointSpan;
	stop_span_ = StopSpan;
	looping_ = Loop;

	if (!goals_list_.empty())
	{
		setGoal(goals_list_.front());
	}
}

geometry_msgs::Pose GoalsHandle::getMapOrigin() const
{
	return map_origin_;
}

geometry_msgs::Pose GoalsHandle::getCurrentPose() const
{
	return estimated_;
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
	estimated_ = Estimated->pose.pose;

	tf::Quaternion quat(estimated_.orientation.x, estimated_.orientation.y, estimated_.orientation.z, estimated_.orientation.w);
  	double roll = 0.0, pitch = 0.0, yaw = 0.0;
  	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
}

void GoalsHandle::callbackGoalDone(const actionlib::SimpleClientGoalState& State, const move_base_msgs::MoveBaseResultConstPtr& Result)
{

}

void GoalsHandle::callbackGoalActive()
{
	active_goal_ = goals_list_.front();
	goals_list_.pop_front();
	if (looping_)
	{
		goals_list_.push_back(active_goal_);
	}
#ifdef DEBUG
	std::cout << "goal left count " << goals_list_.size() << std::endl;
#endif
}

void GoalsHandle::callbackGoalFeedback(const move_base_msgs::MoveBaseFeedbackConstPtr& Feedback)
{
	if (fabs(Feedback->base_position.pose.position.x - active_goal_.position.x) < point_span_ &&
		fabs(Feedback->base_position.pose.position.y - active_goal_.position.y) < point_span_)
	{
		if (!goals_list_.empty())
		{
			setGoal(goals_list_.front());
			printf("tolerace reached, proceeding the next\n");	
		}
	}
#ifndef DEBUG
	std::cout << "delta " << fabs(Feedback->base_position.pose.position.x - active_goal_.position.x) << " " << 
		fabs(Feedback->base_position.pose.position.y - active_goal_.position.y) << " tolerance " << point_span_ <<
		" left point count " << std::to_string(goals_list_.size()) << std::endl;
	std::cout << "goal feedback " << Feedback->base_position.pose.position.x << " " << Feedback->base_position.pose.position.y <<
		" active goal " << active_goal_.position.x << " " << active_goal_.position.y << std::endl;
#endif
}
