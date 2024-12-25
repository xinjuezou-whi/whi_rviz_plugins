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

bool GoalsHandle::execute(const std::vector<WaypointPack>& WaypointPacks, double PointSpan, double StopSpan,
	bool Loop/* = false*/)
{
	std::map<int, std::string> tasks;
	bool res = execute(WaypointPacks, tasks, PointSpan, StopSpan, Loop);

	return res;
}

bool GoalsHandle::execute(const std::vector<WaypointPack>& WaypointPacks, const std::map<int, std::string>& Tasks,
	double PointSpan, double StopSpan, bool Loop/* = false*/)
{
	goals_list_.clear();

	for (std::size_t i = findBeginIndex(WaypointPacks); ; i = (i + 1) % WaypointPacks.size())
	{
		std::string config;
		if (auto search = Tasks.find(i); search != Tasks.end())
		{
			config = Tasks.at(i);
		}
		goals_list_.emplace_back(WaypointPacks[i].first.pose, WaypointPacks[i].second, config);

		if (goals_list_.size() == WaypointPacks.size())
		{
			goals_list_.back().is_last_ = true;
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
		if (non_realtime_loop_)
		{
			non_realtime_loop_->stop();
			non_realtime_loop_ = nullptr;
		}

		bool res = false;
		if (goals_list_.front().is_relative_)
		{
			goals_list_.front().absolute_pose_ = constructAbsGoal(goals_list_.front().request_pose_);
			res = setGoal(goals_list_.front().absolute_pose_);
		}
		else
		{
			res = setGoal(goals_list_.front().request_pose_);
		}

		return res;
	}
	else
	{
		return false;
	}
}

void GoalsHandle::cancel()
{
	task_plugin_->abort();

	cancelGoal();
	goals_list_.clear();
}

void GoalsHandle::reset()
{
	// neutralize the active goal
	active_goal_.request_pose_ = getCurrentPose();
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

geometry_msgs::Pose GoalsHandle::getCurrentPose() const
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

bool GoalsHandle::setGoal(const geometry_msgs::Pose& Goal, bool Recovery/* = false*/)
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

	state_last_ = ros::Time::now();
	is_recovery_ = Recovery;
	stuck_relocate_count_ = is_recovery_ ? stuck_relocate_count_ : 0;

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
		func_eta_(active_goal_.absolute_pose_, -2.0);
	}
}

geometry_msgs::TransformStamped GoalsHandle::listenTf(const std::string& DstFrame, const std::string& SrcFrame,
    const ros::Time& Time) const
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

void GoalsHandle::handleGoalAndState(const geometry_msgs::PoseStamped& Pose)
{
	ros::Time current = ros::Time::now();
	double dist = distance(Pose.pose, active_goal_.absolute_pose_);

	if (isStill())
	{
		// mechanics of break from stuck
		if ((current - state_last_).toSec() > stuck_timeout_)
		{
			if (metTolerance(getCurrentPose(), active_goal_.absolute_pose_) ||
				stuck_relocate_count_++ >= recovery_max_try_count_)
			{
				if (goals_list_.empty())
				{
					if (active_goal_.task_.empty())
					{
						if (func_execution_state_)
						{
							func_execution_state_(STA_DONE, nullptr);
						}
						if (func_eta_)
						{
							func_eta_(active_goal_.absolute_pose_, -1.0);
						}
						std::cout << "all goals traversed. remained goals " << goals_list_.size() << std::endl;
					}
					else
					{
						if (!state_task_)
						{
							// execute waypoint task then to approach the next waypoint
							// IMPORTANT: DO NOT CALL ACTION in its own callback
							std::thread{ &GoalsHandle::executeTask, this, true }.detach();
						}
					}
				}
				else
				{
					if (active_goal_.task_.empty())
					{
						double span = active_goal_.is_last_ ? -stop_span_ : -point_span_;
						if (span < stuck_timeout_)
						{
							// to approach the next waypoint
							if (goals_list_.front().is_relative_)
							{
								goals_list_.front().absolute_pose_ = constructAbsGoal(goals_list_.front().request_pose_);
								setGoal(goals_list_.front().absolute_pose_);
							}
							else
							{
								setGoal(goals_list_.front().request_pose_);
							}
							std::cout << "break from stuck. remained goals " << goals_list_.size() << std::endl;
						}
					}
					else
					{
						if (!state_task_)
						{
							// execute waypoint task then to approach the next waypoint
							// IMPORTANT: DO NOT CALL ACTION in its own callback
							std::thread{ &GoalsHandle::executeTask, this, true }.detach();
						}
					}	
				}
			}
			else
			{
				// to re-execute the current goal
				goals_list_.insert(goals_list_.begin(), active_goal_);
				if (goals_list_.front().is_relative_)
				{
					goals_list_.front().absolute_pose_ = constructAbsGoal(goals_list_.front().request_pose_);
					setGoal(goals_list_.front().absolute_pose_, true);
				}
				else
				{
					setGoal(goals_list_.front().request_pose_, true);
				}
			}
		}
	}
	else
	{
		if (active_goal_.task_.empty() && !goals_list_.front().is_relative_ &&
			((!active_goal_.is_last_ && point_span_ < 0.0) || (active_goal_.is_last_ && stop_span_ < 0.0)))
		{
			double tolerance = active_goal_.is_last_ ? -stop_span_ * current_linear_ : -point_span_ * current_linear_;
			if (dist < tolerance)
			{
				setGoal(goals_list_.front().request_pose_, true);
				std::cout << "tolerance within " << tolerance << " reached, proceeding the next. remained goals " <<
					goals_list_.size() << std::endl;
			}
		}

		state_last_ = current;
	}

	// eta info shows during running state
	if (current_linear_ > 1e-2 && dist > 0.2)
	{
		if (func_eta_)
		{
			func_eta_(active_goal_.absolute_pose_, dist / current_linear_);
		}
	}
}

void GoalsHandle::updateStateInfo(const GoalPack& Goal)
{
	if (func_execution_state_)
	{
		if (looping_)
		{
			// is final one?
			if (Goal.is_last_)
			{
				loop_count_ = is_recovery_ ? loop_count_ : loop_count_ + 1;
				std::shared_ptr<std::string> info = std::make_shared<std::string>(
					std::to_string(loop_count_) + (loop_count_ > 1 ? " loops proceed" : " loop proceed"));
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
	current_angular_ = CmdVel->angular.z;
}

void GoalsHandle::callbackGoalDone(const actionlib::SimpleClientGoalState& State,
	const move_base_msgs::MoveBaseResultConstPtr& Result)
{
#ifdef DEBUG
	std::cout << "goal state " << std::to_string(State.state_) << " goal left " << goals_list_.size() << std::endl;
#endif
	if (active_goal_.task_.empty())
	{
		double pointSpan = point_span_ < 0.0 ? 0.1 : point_span_;
		double stopSpan = stop_span_ < 0.0 ? 0.1 : stop_span_;
		ros::Duration duration = active_goal_.is_last_ ? ros::Duration(stopSpan) : ros::Duration(pointSpan);
		non_realtime_loop_ = std::make_unique<ros::Timer>(
			node_handle_->createTimer(duration,
			std::bind(&GoalsHandle::callbackTimer, this, std::placeholders::_1)));
	}
	else
	{
		if (movebase_client_->getState().state_ == actionlib::SimpleClientGoalState::SUCCEEDED &&
			metDistance(active_goal_.absolute_pose_, getCurrentPose(), 0.2))
		{
			// execute task then to approach the next waypoint
			// IMPORTANT: DO NOT CALL ACTION in its own callback
			std::thread{ &GoalsHandle::executeTask, this, false }.detach();
		}
		else
		{
			if (goals_list_.empty())
			{
				// set finish state
				if (func_execution_state_)
				{
					func_execution_state_(STA_DONE, nullptr);
				}
			}
		}
	}
}

void GoalsHandle::callbackGoalActive()
{
	static GoalPack lastGoal;

	active_goal_ = goals_list_.front();
	goals_list_.pop_front();
	if (looping_)
	{
		goals_list_.push_back(active_goal_);
	}
	else
	{
		if (active_goal_.is_last_)
		{
			goals_list_.clear();
		}
	}

	updateStateInfo(lastGoal);
	lastGoal = active_goal_;

#ifdef DEBUG
	std::cout << "goal left count " << goals_list_.size() << std::endl;
#endif
}

void GoalsHandle::callbackGoalFeedback(const move_base_msgs::MoveBaseFeedbackConstPtr& Feedback)
{
	handleGoalAndState(Feedback->base_position);
}

void GoalsHandle::callbackTimer(const ros::TimerEvent& Event)
{
	if (!goals_list_.empty())
	{
		if (goals_list_.front().is_relative_)
		{
			goals_list_.front().absolute_pose_ = constructAbsGoal(goals_list_.front().request_pose_);
			setGoal(goals_list_.front().absolute_pose_);
		}
		else
		{
			setGoal(goals_list_.front().request_pose_);
		}

		std::cout << "span timeout, proceeding the next" << std::endl;
	}
	else
	{
		// set finish state
		if (func_execution_state_)
		{
			func_execution_state_(STA_DONE, nullptr);
		}
	}

	non_realtime_loop_->stop();
	non_realtime_loop_ = nullptr;
}

int GoalsHandle::findBeginIndex(const std::vector<WaypointPack>& WaypointPacks)
{
	int beginIndex = 0;
	double minDistance = std::numeric_limits<double>::max();
	for (std::size_t i = 0; i < WaypointPacks.size(); ++i)
	{
		// only consider the absolute pose
		auto dist = WaypointPacks[i].second ?
			std::numeric_limits<double>::max() : distance(getCurrentPose(), WaypointPacks[i].first.pose);
		if (dist < minDistance)
		{
			beginIndex = i;
			minDistance = dist;
		}
	}
	// choose the next one if current pose overlays the waypoint
	if (minDistance < 0.5)
	{
		beginIndex = (beginIndex + 1) % WaypointPacks.size();
	}

	return beginIndex;
}

void GoalsHandle::executeTask(bool ForceClean/* = false*/)
{
	if (ForceClean)
	{
		cancelGoal();
	}

	state_task_ = true;

	if (task_plugin_)
	{
		auto delta = locationDelta();
		task_plugin_->process(active_goal_.task_, delta.data(), delta.size());
	}
	if (!goals_list_.empty())
	{
		if (goals_list_.front().is_relative_)
		{
			goals_list_.front().absolute_pose_ = constructAbsGoal(goals_list_.front().request_pose_);
			setGoal(goals_list_.front().absolute_pose_);
		}
		else
		{
			setGoal(goals_list_.front().request_pose_);
		}

		std::cout << "task executed, proceeding the next" << std::endl;
	}
	else
	{
		// set finish state
		if (func_execution_state_)
		{
			func_execution_state_(STA_DONE, nullptr);
		}
	}

	state_task_ = false;
}

std::array<double, 3> GoalsHandle::locationDelta()
{
	auto current = getCurrentPose();
    tf2::Quaternion curQ(current.orientation.x, current.orientation.y, current.orientation.z,
		current.orientation.w);
    double curRoll = 0.0, curPitch = 0.0, curYaw = 0.0;
	tf2::Matrix3x3(curQ).getRPY(curRoll, curPitch, curYaw);

	tf2::Quaternion goalQ(active_goal_.absolute_pose_.orientation.x, active_goal_.absolute_pose_.orientation.y,
		active_goal_.absolute_pose_.orientation.z, active_goal_.absolute_pose_.orientation.w);
    double goalRoll = 0.0, goalPitch = 0.0, goalYaw = 0.0;
	tf2::Matrix3x3(goalQ).getRPY(goalRoll, goalPitch, goalYaw);

	std::array<double, 3> delta;
	delta[0] = active_goal_.absolute_pose_.position.x - current.position.x;
	delta[1] = active_goal_.absolute_pose_.position.y - current.position.y;
	delta[2] = goalYaw - curYaw;
	delta[2] = fabs(delta[2]) > M_PI ? delta[2] - 2.0 * M_PI : delta[2];

	return delta;
}

bool GoalsHandle::isStill() const
{
	return fabs(current_linear_) < 1e-4 && fabs(current_angular_) < 1e-4;
}

bool GoalsHandle::metTolerance(const geometry_msgs::Pose& PoseA, const geometry_msgs::Pose& PoseB) const
{
	tf2::Quaternion curQ(PoseA.orientation.x, PoseA.orientation.y, PoseA.orientation.z, PoseA.orientation.w);
    double curRoll = 0.0, curPitch = 0.0, curYaw = 0.0;
	tf2::Matrix3x3(curQ).getRPY(curRoll, curPitch, curYaw);

	tf2::Quaternion activeQ(PoseB.orientation.x, PoseB.orientation.y, PoseB.orientation.z, PoseB.orientation.w);
    double activeRoll = 0.0, activePitch = 0.0, activeYaw = 0.0;
	tf2::Matrix3x3(activeQ).getRPY(activeRoll, activePitch, activeYaw);

	return fabs(PoseA.position.x - active_goal_.absolute_pose_.position.x) < xy_goal_tolerance_ &&
		fabs(PoseA.position.y - active_goal_.absolute_pose_.position.y) < xy_goal_tolerance_ &&
		fabs(curYaw - activeYaw) < yaw_goal_tolerance_;
}

static void rangeRadians(double& Radians)
{
	if (Radians > M_PI)
	{
		Radians -= 2.0 * M_PI;
	}
	if (Radians < -M_PI)
	{
		Radians += 2.0 * M_PI;
	}
}

static std::array<double, 3> toEuler(const tf2::Quaternion& Quaternion)
{
	double roll = 0.0, pitch = 0.0, yaw = 0.0;
	tf2::Matrix3x3(Quaternion).getRPY(roll, pitch, yaw);

	return { roll, pitch, yaw };
}

static std::array<double, 3> toEuler(const geometry_msgs::Quaternion& Orientation)
{
	tf2::Quaternion quaternion(Orientation.x, Orientation.y, Orientation.z, Orientation.w);

	return toEuler(quaternion);
}

static geometry_msgs::Quaternion fromEuler(double Roll, double Pitch, double Yaw)
{
	tf2::Quaternion orientation;
	orientation.setRPY(Roll, Pitch, Yaw);

	return tf2::toMsg(orientation);
}

geometry_msgs::Pose GoalsHandle::constructAbsGoal(const geometry_msgs::Pose& Relative) const
{
	geometry_msgs::Pose absolute = getCurrentPose();
	// position
	absolute.position.x += Relative.position.x;
	absolute.position.y += Relative.position.y;
	absolute.position.z += Relative.position.z;
	// orientation
	auto absEulers = toEuler(absolute.orientation);
	auto relativeEulers = toEuler(Relative.orientation);
	for (int i = 0; i < absEulers.size(); ++i)
	{
		absEulers[i] += relativeEulers[i];
		rangeRadians(absEulers[i]);
	}
	absolute.orientation = fromEuler(absEulers[0], absEulers[1], absEulers[2]);

	return absolute;
}

void GoalsHandle::setTaskPlugin(boost::shared_ptr<whi_rviz_plugins::BasePlugin> Plugin)
{
	task_plugin_ = Plugin;
}

void GoalsHandle::setBaselinkFrame(const std::string& Frame)
{
	baselink_frame_ = Frame;
}

void GoalsHandle::setStuckTimeout(double Timeout)
{
	stuck_timeout_ = Timeout;
}

void GoalsHandle::setRecoveryMaxTryCount(int Count)
{
	recovery_max_try_count_ = Count;
}

void GoalsHandle::setTolerance(double XyTolerance, double YawTolerance)
{
	xy_goal_tolerance_ = XyTolerance;
	yaw_goal_tolerance_ = YawTolerance;
}

bool GoalsHandle::metDistance(const geometry_msgs::Pose& Pose1, const geometry_msgs::Pose& Pose2, double Tolerance)
{
	return fabs(Pose1.position.x - Pose2.position.x) < Tolerance && fabs(Pose1.position.y - Pose2.position.y) < Tolerance;
}

double GoalsHandle::distance(const geometry_msgs::Pose& Pose1, const geometry_msgs::Pose& Pose2)
{
	return sqrt(pow(Pose1.position.x - Pose2.position.x, 2.0) + pow(Pose1.position.y - Pose2.position.y, 2.0));
}
