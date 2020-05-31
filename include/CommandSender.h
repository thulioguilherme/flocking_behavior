#pragma once
#ifndef COMMAND_SENDER_H
#define COMMAND_SENDER_H

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

#include <mrs_lib/param_loader.h>

#include <mrs_msgs/SetInt.h>

#include <std_srvs/Trigger.h>

namespace command_sender{

class CommandSender : public nodelet::Nodelet {
public:
  virtual void onInit();

private:
	/* flags */
	bool is_initialized_;

	/* parameters */
	std::string _leader_name_;
	std::vector<std::string> _uav_names_;
	std::vector<std::string> _flocking_command_list_;
	std::vector<std::string> _leader_command_list_;

	// | ------------------------ service server callbacks ----------------------- |

	bool callbackSendCommand(mrs_msgs::SetInt::Request& req, mrs_msgs::SetInt::Response& res);
	ros::ServiceServer srv_server_send_command_;
	std::vector<ros::ServiceClient> srv_client_uav_commands_;
  
};

}  // namespace command_sender

#endif