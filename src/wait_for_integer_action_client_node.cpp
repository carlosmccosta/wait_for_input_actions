/**\file wait_for_integer_action_client_node.cpp
 * \brief Action client node for waiting for integers
 *
 * @version 1.0
 * @author carloscosta
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <string>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <task_manager_msgs/WaitForInputSkillAction.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


// ###################################################################################   <main>   ##############################################################################
int main(int argc, char** argv) {
	ros::init(argc, argv, "wait_for_integer_action_client_node");

	ros::NodeHandlePtr node_handle(new ros::NodeHandle());
	ros::NodeHandlePtr private_node_handle(new ros::NodeHandle("~"));

	int goalId;
	private_node_handle->param("goalId", goalId, 1);

	std::string action_server_name;
	private_node_handle->param("action_server_name", action_server_name, std::string("WaitForInputSkillActionServer"));

	double time_out;
	private_node_handle->param("time_out", time_out, 10.0);

	actionlib::SimpleActionClient<task_manager_msgs::WaitForInputSkillAction> actionClient(*node_handle, action_server_name, true);

	ROS_INFO_STREAM("Waiting for action server " << action_server_name << " to start.");
	actionClient.waitForServer();

	ROS_INFO("Action server started, sending goal...");
	task_manager_msgs::WaitForInputSkillGoal goal;
	goal.inputId = goalId;
	actionClient.sendGoal(goal);

	bool finished_before_timeout = actionClient.waitForResult(ros::Duration(time_out));
	if (finished_before_timeout) {
		actionlib::SimpleClientGoalState state = actionClient.getState();
		ROS_INFO_STREAM("Action finished: " << state.toString());
	} else {
		ROS_INFO("Action did not finish before the time out. Preempting...");
		actionClient.cancelGoal();
		actionClient.waitForResult(ros::Duration(time_out));

		actionlib::SimpleClientGoalState state = actionClient.getState();
		ROS_INFO_STREAM("Action state: " << state.toString());
	}

	return 0;
}
// ###################################################################################   </main>   #############################################################################
