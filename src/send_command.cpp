#include <ros/ros.h>
#include <ros/package.h>

#include <std_srvs/Trigger.h>

// Usage: rosrun flocking send_command ($command) ($has_leader) ($UAV1_NAME) ($UAV2_NAME) ($UAV3_NAME)
// Possible commands: start_hover_mode, start_swarming_mode, close_node
// Possible values for has_leader: 0 (false) or 1 (true)
// The leader will be the ($UAV1_NAME)

int main(int argc, char **argv){
  ros::init(argc, argv, "send_command");

  ros::NodeHandle nh("~");

  ros::Time::waitForValid();

  std::string command, uav1_name, uav2_name, uav3_name;
  bool has_leader;

  if(argc != 6){
    ROS_ERROR("[SendCommand]: Wrong number of arguments. It should be 5 arguments. Shutting down");
    return 0;
  }else{
    command = argv[1];
    has_leader = ("1" == argv[2]);
    uav1_name = argv[3];
    uav2_name = argv[4];
    uav3_name = argv[5];
  }

  /* create new Trigger service message */
  std_srvs::Trigger srv_command_call;

  /* if there is a leader, request the start of the waypoint flier to the ($UAV1_NAME) instead the flocking behavior */
  if(has_leader){
    if(command == "start_swarming_mode"){
      ros::ServiceClient srv_client_uav1 = nh.serviceClient<std_srvs::Trigger>("/"+ uav1_name + "/waypoint_flier/start_waypoints_following");
      if(srv_client_uav1.call(srv_command_call)){
        ROS_INFO("[SendCommand]: Called service from %s using the command start_waypoints_following", uav1_name.c_str());
      }
    }
  }else{
    /* if there isn't a leader, request to turn on the flocking behavior */
    ros::ServiceClient srv_client_uav1 = nh.serviceClient<std_srvs::Trigger>("/"+ uav1_name +"/formation/"+ command);
    if(srv_client_uav1.call(srv_command_call)){
      ROS_INFO("[SendCommand]: Called service from %s using the command %s", uav1_name.c_str(), command.c_str());
    }
  }

  /* request the command to the ($UAV2_NAME) */
  ros::ServiceClient srv_client_uav2 = nh.serviceClient<std_srvs::Trigger>("/"+ uav2_name +"/formation/"+ command);
  if(srv_client_uav2.call(srv_command_call)){
    ROS_INFO("[SendCommand]: Called service from %s using the command %s", uav2_name.c_str(), command.c_str());
  }

  /* request the command to the ($UAV3_NAME) */
  ros::ServiceClient srv_client_uav3 = nh.serviceClient<std_srvs::Trigger>("/"+ uav3_name +"/formation/"+ command);
  if(srv_client_uav3.call(srv_command_call)){
    ROS_INFO("[SendCommand]: Called service from %s using the command %s", uav3_name.c_str(), command.c_str());
  }

  return 1;
}