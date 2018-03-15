/**\file wait_for_integer_action_client_node.cpp
 * \brief Action client node for waiting for integers
 *
 * @version 1.0
 * @author carloscosta
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
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

	actionlib::SimpleActionClient<task_manager_msgs::WaitForInputSkillAction> actionClient(*node_handle, "WaitForInputSkillActionServer", true);

	ROS_INFO("Waiting for action server to start.");
	actionClient.waitForServer();

	ROS_INFO("Action server started, sending goal...");
	int goalId;
	private_node_handle->param("goalId", goalId, 1);
	task_manager_msgs::WaitForInputSkillGoal goal;
	goal.inputId = goalId;
	actionClient.sendGoal(goal);

	bool finished_before_timeout = actionClient.waitForResult(ros::Duration(10.0));
	if (finished_before_timeout) {
		actionlib::SimpleClientGoalState state = actionClient.getState();
		ROS_INFO_STREAM("Action finished: " << state.toString());
	} else {
		ROS_INFO("Action did not finish before the time out. Preempting...");
		actionClient.cancelGoal();
		actionClient.waitForResult(ros::Duration(10.0));

		actionlib::SimpleClientGoalState state = actionClient.getState();
		ROS_INFO_STREAM("Action state: " << state.toString());
	}

	return 0;
}
// ###################################################################################   </main>   #############################################################################
