#pragma once

/**\file wait_for_integer_action.h
 * \brief File with WaitForIntegerActionServer class definition
 *
 * @version 1.0
 * @author carloscosta
 */

#include <memory>
#include <string>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <std_msgs/Int32.h>
#include <task_manager_msgs/WaitForInputSkillAction.h>


namespace wait_for_input_actions {
/**
 * \brief Action server class for waiting for integers
 */
class WaitForIntegerActionServer {
	public:
		WaitForIntegerActionServer();
		virtual ~WaitForIntegerActionServer();

		virtual bool loadConfigurationFromParameterServer(ros::NodeHandlePtr& _node_handle, ros::NodeHandlePtr& _private_node_handle);
		virtual void start();

		void integerCallback(const std_msgs::Int32::ConstPtr& msg);
		void goalCallback();
		void preemptCallback();
		bool checkIfPreempted();
		void publishSuccess();


		typedef actionlib::SimpleActionServer<task_manager_msgs::WaitForInputSkillAction> WaitForInputSkillActionServer;
	protected:
		ros::NodeHandlePtr node_handle_;
		ros::NodeHandlePtr private_node_handle_;
		ros::Subscriber subscriber_integer_;
		std::shared_ptr<WaitForInputSkillActionServer> wait_for_input_skill_action_server_;
		std::string subscriber_integer_topic_;
		int last_received_integer_;
		int goal_integer_;
		bool pending_goal_;
};

} /* namespace wait_for_input_actions */
