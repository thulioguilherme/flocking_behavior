#include <ros/ros.h>
#include <ros/package.h>

#include <std_srvs/Trigger.h>

int main(int argc, char **argv){
	ros::init(argc, argv, "send_command");

	ros::NodeHandle nh("~");

	ros::Time::waitForValid();

	std::string command;

	std::string uav1_name, uav2_name, uav3_name, uav4_name;

	if(argc < 6){
		ROS_ERROR("[SendCommand]: Wrong number of arguments. It should be 5 arguments. Shuttind down");
		return 0;
	}else{
		command = argv[1];
		uav1_name = argv[2];
		uav2_name = argv[3];
		uav3_name = argv[4];
		uav4_name = argv[5];
	}

	ros::ServiceClient srv_client_uav1 = nh.serviceClient<std_srvs::Trigger>("/"+ uav1_name +"/formation/"+ command);
	ros::ServiceClient srv_client_uav2 = nh.serviceClient<std_srvs::Trigger>("/"+ uav2_name +"/formation/"+ command);
	ros::ServiceClient srv_client_uav3 = nh.serviceClient<std_srvs::Trigger>("/"+ uav3_name +"/formation/"+ command);
	ros::ServiceClient srv_client_uav4 = nh.serviceClient<std_srvs::Trigger>("/"+ uav4_name +"/formation/"+ command);
	ros::ServiceClient srv_client_sensor_neighbor;

	std_srvs::Trigger srv_command_call;

	if(command == "start_swarming_mode"){
		srv_client_sensor_neighbor = nh.serviceClient<std_srvs::Trigger>("/sensor_neighbor/start_experiment");
		if(srv_client_sensor_neighbor.call(srv_command_call)){
			ROS_INFO("[SendCommand]: Called service from SensorNeighbor using the command: start_experiment");
		}
	}else if(command == "close_node"){
		srv_client_sensor_neighbor = nh.serviceClient<std_srvs::Trigger>("/sensor_neighbor/close_node");
		if(srv_client_sensor_neighbor.call(srv_command_call)){
			ROS_INFO("[SendCommand]: Called service from SensorNeighbor using the command: close_node");
		}
	}

	if(srv_client_uav1.call(srv_command_call)){
		ROS_INFO("[SendCommand]: Called service from %s using the command: %s", uav1_name.c_str(), command.c_str());
	}

	if(srv_client_uav2.call(srv_command_call)){
		ROS_INFO("[SendCommand]: Called service from %s using the command: %s", uav2_name.c_str(), command.c_str());
	}

	if(srv_client_uav3.call(srv_command_call)){
		ROS_INFO("[SendCommand]: Called service from %s using the command: %s", uav3_name.c_str(), command.c_str());
	}

	if(srv_client_uav4.call(srv_command_call)){
		ROS_INFO("[SendCommand]: Called service from %s using the command: %s", uav4_name.c_str(), command.c_str());
	}

	return 1;
}