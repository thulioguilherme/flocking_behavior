#include <CommandSender.h>
#include <pluginlib/class_list_macros.h>

namespace command_sender {

  /* onInit() //{ */
  void CommandSender::onInit() {
    /* set flags to false */
    is_initialized_ = false;

    ros::NodeHandle nh("~");

    ros::Time::waitForValid();

    mrs_lib::ParamLoader param_loader(nh, "CommandSender");

    /* load parameters */
    param_loader.loadParam("uav_names", _uav_names_);
    param_loader.loadParam("leader_name", _leader_name_);
    param_loader.loadParam("leader_commands", _leader_command_list_);
    param_loader.loadParam("flocking_commands", _flocking_command_list_);

    if (!param_loader.loadedSuccessfully()) {
      ROS_ERROR("[CommandSender]: failed to load non-optional parameters!");
      ros::shutdown();
    }

    /* check if the name of the leader is on the list of UAVs */
    if(_leader_name_ != "") {
      std::vector<std::string>::iterator it = std::find(_uav_names_.begin(), _uav_names_.end(), _leader_name_);
      if(it == _uav_names_.end()) {
        ROS_ERROR("[CommandSender]: leader name is not on the list of UAVs. Shutting down.");
        ros::shutdown();
      }
    }

    /* service clients */
    for(int i = 0; i < _flocking_command_list_.size(); i++) {
      for(int j = 0; j < _uav_names_.size(); j++) {
        if(_uav_names_[j] == _leader_name_) {
          srv_client_uav_commands_.push_back(nh.serviceClient<std_srvs::Trigger>("/" + _uav_names_[j] + "/waypoint_flier/" + _leader_command_list_[i]));
        } else {
          srv_client_uav_commands_.push_back(nh.serviceClient<std_srvs::Trigger>("/" + _uav_names_[j] + "/formation/" + _flocking_command_list_[i]));
        }
      }
    }

    /* service servers */
    srv_server_send_command_ = nh.advertiseService("send_command", &CommandSender::callbackSendCommand, this);

    is_initialized_ = true;
    ROS_INFO_ONCE("[CommandSender]: initialized");

    ros::spin();
  }

  //}

  // | ------------------------ service server callbacks ----------------------- |

  /* callbackSendCommand() //{ */

  bool CommandSender::callbackSendCommand(mrs_msgs::SetInt::Request& req, mrs_msgs::SetInt::Response& res) {
    if (!is_initialized_) {
      ROS_WARN("[CommandSender]: Cannot change to hover mode, nodelet is not initialized");
      res.message = "Cannot change to hover mode, nodelet is not initialized";
      res.success = false;

      return true;
    }

    int index_command = req.value;

    /* check if the index of the command is valid */
    if(index_command < 0 || index_command >= _flocking_command_list_.size()) {
      ROS_WARN("[CommandSender]: The index of the requested command is out of bound");
      res.message = "The index of the requested command is out of bound";
      res.success = false;

      return true;
    }

    /* create new Trigger service message */
    std_srvs::Trigger srv_command_call;

    /* send commmand to every UAV on the list */
    int offset = index_command * _uav_names_.size();
    for(int i = 0; i < _uav_names_.size(); i++) {
      /* skip if doesn't have a service to a certain command */
      if((_uav_names_[i] == _leader_name_) && (_leader_command_list_[index_command] == "None")) {
        continue;
      }

      /* get name of the command */
      std::string command = (_uav_names_[i] == _leader_name_) ? _leader_command_list_[index_command] : _flocking_command_list_[index_command];

      /* call service */
      if(srv_client_uav_commands_[offset + i].call(srv_command_call)) {
        ROS_INFO("[CommandSender]: Called service from %s using the command %s", _uav_names_[i].c_str(), command.c_str());
      }
    }

    res.message = "Send the command to all UAVs";
    res.success = true;

    return true;
  }

  //}

} // namespace command_sender

PLUGINLIB_EXPORT_CLASS(command_sender::CommandSender, nodelet::Nodelet);