/**\file wait_for_integer_action_server.cpp
 * \brief File with WaitForIntegerActionServer class implementation
 *
 * @version 1.0
 * @author carloscosta
 */

#include <wait_for_input_actions/wait_for_integer_action_server.h>

namespace wait_for_input_actions {


WaitForIntegerActionServer::WaitForIntegerActionServer() :
		last_received_integer_(-1),
		goal_integer_(-2),
		pending_goal_(false) {}
WaitForIntegerActionServer::~WaitForIntegerActionServer() {}


bool WaitForIntegerActionServer::loadConfigurationFromParameterServer(ros::NodeHandlePtr& _node_handle, ros::NodeHandlePtr& _private_node_handle) {
	node_handle_ = _node_handle;
	private_node_handle_ = _private_node_handle;
	private_node_handle_->param("integer_topic", subscriber_integer_topic_, std::string("integer"));
	return true;
}


void WaitForIntegerActionServer::start() {
	subscriber_integer_ = node_handle_->subscribe(subscriber_integer_topic_, 1, &WaitForIntegerActionServer::integerCallback, this);
	wait_for_input_skill_action_server_ = std::make_shared<WaitForInputSkillActionServer>(*node_handle_, "WaitForInputSkillActionServer", false);
	wait_for_input_skill_action_server_->registerGoalCallback(boost::bind(&WaitForIntegerActionServer::goalCallback, this));
	wait_for_input_skill_action_server_->registerPreemptCallback(boost::bind(&WaitForIntegerActionServer::preemptCallback, this));
	wait_for_input_skill_action_server_->start();

	ROS_DEBUG("WaitForInputSkillActionServer started...");

	ros::spin();
}


void WaitForIntegerActionServer::integerCallback(const std_msgs::Int32::ConstPtr& msg) {
	last_received_integer_ = msg->data;

	ROS_DEBUG_STREAM("Received new integer: " << last_received_integer_);

	if (pending_goal_
			&& wait_for_input_skill_action_server_->isActive()
			&& !checkIfPreempted()
			&& last_received_integer_ == goal_integer_)
		publishSuccess();
}


void WaitForIntegerActionServer::goalCallback() {
	task_manager_msgs::WaitForInputSkillGoalConstPtr goal = wait_for_input_skill_action_server_->acceptNewGoal();
	goal_integer_ = goal->inputId;

	ROS_DEBUG_STREAM("Received new goal with ID: " << goal_integer_);

	if (!wait_for_input_skill_action_server_->isActive() || checkIfPreempted())
		return;

	if (last_received_integer_ == goal_integer_) {
		publishSuccess();
	} else {
		pending_goal_ = true;
	}
}


void WaitForIntegerActionServer::preemptCallback() {
	pending_goal_ = false;
	ROS_DEBUG_STREAM("Preempting goal with ID: " << goal_integer_);
	if (wait_for_input_skill_action_server_->isActive()) {
		task_manager_msgs::WaitForInputSkillResult result;
		result.percentage = 0;
		result.skillStatus = "Preempted";
		wait_for_input_skill_action_server_->setPreempted(result, "PREEMPTED");
		ROS_DEBUG_STREAM("Preempted goal with ID: " << goal_integer_);
	}
}


bool WaitForIntegerActionServer::checkIfPreempted() {
	if (wait_for_input_skill_action_server_->isPreemptRequested()) {
		preemptCallback();
		return true;
	}

	return false;
}


void WaitForIntegerActionServer::publishSuccess() {
	task_manager_msgs::WaitForInputSkillResult result;
	result.percentage = 100;
	result.skillStatus = "Success";
	pending_goal_ = false;
	wait_for_input_skill_action_server_->setSucceeded(result, "SUCCEEDED");

	ROS_DEBUG_STREAM("Goal with ID: " << goal_integer_ << " succeeded");
}

} /* namespace wait_for_input_actions */

