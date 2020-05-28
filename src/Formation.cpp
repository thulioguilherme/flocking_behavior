#include <Formation.h>
#include <pluginlib/class_list_macros.h>

namespace formation
{

/* onInit() //{ */
void Formation::onInit(){
	/* set flags to false */
	is_initialized_ = false;
	hover_mode_ = false;
	swarming_mode_ = false;

	ros::NodeHandle nh("~");

	ros::Time::waitForValid();

	mrs_lib::ParamLoader param_loader(nh, "Formation");

	/* Load experiment parameters */
	param_loader.loadParam("experiment/duration", _duration_timer_experiment_);
	param_loader.loadParam("uav_name", _uav_name_);
	param_loader.loadParam("frame", _frame_);
	param_loader.loadParam("rate/publish_speed", _rate_timer_publisher_speed_);
	param_loader.loadParam("desired_height", _desired_height_);
	param_loader.loadParam("land_at_the_end", _land_end_);

	/* Load proximal control parameters */
	param_loader.loadParam("flocking/proximal/noise", _noise_);
	param_loader.loadParam("flocking/proximal/desired_distance", _desired_distance_);
	param_loader.loadParam("flocking/proximal/strength_potential", _strength_potential_);
	
	steepness_potential_ = log(2) / log(_desired_distance_ / _noise_);
  	max_range_ = 1.8 * _desired_distance_;

  	/* Load motion control parameters */
	param_loader.loadParam("flocking/MDMC/K1", _K1_);
	param_loader.loadParam("flocking/MDMC/K2", _K2_);
	param_loader.loadParam("flocking/MDMC/move_forward", _move_forward_);
	
	if(!param_loader.loadedSuccessfully()){
    	ROS_ERROR("[Formation]: failed to load non-optional parameters!");
    	ros::shutdown();
  	}

  	/* Subscriber */
	sub_neighbors_info_ = nh.subscribe("/sensor_neighbor/"+ _uav_name_ +"/neighbors", 1, &Formation::callbackUAVNeighbors, this);

  	/* Service client */
  	srv_client_switcher_ = nh.serviceClient<mrs_msgs::String>("/"+ _uav_name_ + "/control_manager/switch_tracker");
  	srv_client_land_ = nh.serviceClient<std_srvs::Trigger>("/"+ _uav_name_ + "/uav_manager/land");

  	/* Service servers */
  	srv_server_hover_mode_ = nh.advertiseService("start_hover_mode", &Formation::callbackStartHoverMode, this);
  	srv_server_swarming_mode_ = nh.advertiseService("start_swarming_mode", &Formation::callbackStartSwarmingMode, this);
  	srv_server_close_node_ = nh.advertiseService("close_node", &Formation::callbackCloseNode, this);
  	
  	/* Publishers */
  	pub_speed_ = nh.advertise<mrs_msgs::SpeedTrackerCommand>("/"+ _uav_name_ +"/control_manager/speed_tracker/command", 1);

	/* Timers */
	timer_publisher_speed_ = nh.createTimer(ros::Rate(_rate_timer_publisher_speed_), &Formation::callbackTimerPublishSpeed, this, false, false);
	timer_experiment_end_ = nh.createTimer(ros::Duration(_duration_timer_experiment_ + 1), &Formation::callbackTimerExperiment, this, false, false);

	ROS_INFO_ONCE("[Formation]: initialized");

	is_initialized_ = true;
}

//}

// | ------------------------- subscriber callbacks -------------------------- |

/* callbackUAVNeighbors() //{ */

void Formation::callbackUAVNeighbors(const flocking::Neighbors::ConstPtr& neighbors){
	/* return if is not initialized or the code is not on swarming mode */
	if(!is_initialized_ || !swarming_mode_) return;

	if(neighbors->num_neighbors > 0){
		double prox_vector_x = 0.0, prox_vector_y = 0.0, prox_magnitude;
		for(int i = 0; i < neighbors->num_neighbors; i++){
			if(neighbors->range[i] <= max_range_){
				/* compute proximal control vector */
				prox_magnitude = Formation::getProximalMagnitude(neighbors->range[i]);
				prox_vector_x += prox_magnitude * cos(neighbors->bearing[i]);
				prox_vector_y += prox_magnitude * sin(neighbors->bearing[i]);
			}
		}

  		/* convert flocking control (f = p) vector to angular and linear speed */
  		{
  			std::scoped_lock lock(mutex_speed_);
  			w_ = prox_vector_y * _K2_;
			u_ = prox_vector_x * _K1_ + _move_forward_;
			
			has_speed_command_ = true;
  		}
	}
}

//}

// | --------------------------- timer callbacks ----------------------------- |

/* callbackTimerExperiment() //{ */

void Formation::callbackTimerExperiment(const ros::TimerEvent& event){
	/* turn off swarming mode (switch back to hover mode) */
	swarming_mode_ = false;
	
	/* wait to the hover mode send one last speed command to the UAV stand still */
	ros::Duration(0.5).sleep();

	/* stop timer publisher speed (turn off hover mode) */
	timer_publisher_speed_.stop();
	
	/* request land service */
	if(_land_end_){
		ROS_INFO("[Formation]: Calling land service");
		std_srvs::Trigger srv_land_call;
    	srv_client_land_.call(srv_land_call);	
	}
	
	ROS_INFO_ONCE("[Formation]: The experiment time is over. Shutting down");

	/* shutdown node */
	ros::shutdown();

	return;
}

//}

/* callbackTimerPublishSpeed() //{ */

void Formation::callbackTimerPublishSpeed(const ros::TimerEvent& event){
	if(!is_initialized_) return;

	if(hover_mode_){
		/* Create new SpeedTrackerCommand msg */
		std_msgs::Header h;
		h.stamp = ros::Time::now();
		h.frame_id = _uav_name_ + "/" + _frame_;

		mrs_msgs::SpeedTrackerCommand speed_command;
		speed_command.header = h;

		speed_command.use_acceleration = false;
		speed_command.use_force = false;
		speed_command.use_heading_rate = false;

		/* use only heading, height and velocity */
		speed_command.use_heading = true;
		speed_command.use_height = true;
		speed_command.use_velocity = true;

		/* send command to stand still if the code is on hover mode */
		speed_command.velocity.x = 0.0;
		speed_command.velocity.y = 0.0;
		speed_command.velocity.z = 0.0;
		speed_command.heading_rate = 0.0;
		speed_command.height = _desired_height_;

		if(swarming_mode_){
			/* check if the swarming mode has a command */
			std::scoped_lock lock(mutex_speed_);
			if(has_speed_command_){
				speed_command.velocity.x = u_;
				speed_command.heading = w_;
				has_speed_command_ = false;
			}
		}

		try{
			pub_speed_.publish(speed_command);
		}catch(...){
			ROS_ERROR("Exception caught during publishing topic %s", pub_speed_.getTopic().c_str());
		}
	}
}

//}

// | ----------------------- service server callbacks ----------------------- |

/* callbackStartSwarmingMode() //{ */

bool Formation::callbackStartHoverMode(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res){
	if(!is_initialized_){
		ROS_WARN("[Formation]: Cannot change to hover mode, nodelet is not initialized");
		res.message = "Cannot change to hover mode, nodelet is not initialized";
		res.success = false;

		return true;
	} 

	/* change code to hover mode */
	hover_mode_ = true;

	/* start publishing speed commands */
	timer_publisher_speed_.start();

	ROS_INFO("[Formation]: Changed to hover mode. It is safe now to start the swarming mode");
	res.message = "Changed to hover mode. It is safe now to start the swarming mode";
	res.success = true;
	
	return true;
}

//}

/* callbackStartSwarmingMode() //{ */

bool Formation::callbackStartSwarmingMode(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res){
	if(!is_initialized_){
		res.message = "Cannot change to swarming mode, nodelet is not initialized";
		res.success = false;
		
		ROS_WARN("[Formation]: cannot change to swarming mode, nodelet is not initialized");

		return true;
	}

	/* can only change to swarming mode if the code is on hover mode */
	if(!hover_mode_){
		ROS_WARN("[Formation]: Cannot change to swarming mode, hover mode has not been started");
		res.message = "Cannot change to swarming mode, hover mode has not been started";
		res.success = false;

		return true;
	}

	/* create new String message */
	mrs_msgs::String srv_switch_msg;
	srv_switch_msg.request.value = "SpeedTracker";

	/* request tracker change to SpeedTracker */
	if(srv_client_switcher_.call(srv_switch_msg)){
		ROS_INFO("[Formation]: Changed to SpeedTracker. Starting the swarming mode");
		res.message = "Changed to SpeedTracker. Starting the swarming mode";
		res.success = true;

		/* change code to swarming mode */
		swarming_mode_ = true;

		/* start experiment countdown */
		timer_experiment_end_.start();

		ROS_INFO("[Formation]: The experiment has started and will last %s seconds", std::to_string(_duration_timer_experiment_).c_str());

		return true;
	}else{
		ROS_WARN("[Formation]: Could not call switch tracker service to change to swarming mode");
		res.message = "Could not call switch tracker service to change to swarming mode";
		res.success = false;

		return true;
	}
}

//}

/* callbackCloseNode() //{ */

bool Formation::callbackCloseNode(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res){
	if(!is_initialized_){
		res.message = "Cannot stop formation, nodelet is not initialized";
		res.success = false;
		
		ROS_WARN("[Formation]: Cannot stop formation, nodelet is not initialized");

		return true;
	}

	/* turn off swarming mode (switch back to hover mode) */
	swarming_mode_ = false;
	
	/* wait to the hover mode send one last speed command to the UAV stand still */
	ros::Duration(0.5).sleep();

	/* stop timer publisher speed (turn off hover mode) */
	timer_publisher_speed_.stop();
	
	/* request land service */
	if(_land_end_){
		ROS_INFO("[Formation]: Calling land service");
		std_srvs::Trigger srv_land_call;
    	srv_client_land_.call(srv_land_call);	
	}

	ROS_INFO("[Formation]: Closing node before the time. Shutting down");
	res.message = "Closing node before the time. Shutting down";
	res.success = true;

	/* shutdown node */
	ros::shutdown();

	return true;
}

//}

// | -------------------------- support functions -------------------------- |

/* getProximalMagnitude() //{ */

double Formation::getProximalMagnitude(double range){
	return -4 * steepness_potential_ * _strength_potential_ / range * (2 * pow(_noise_/range, 2*steepness_potential_) - pow(_noise_/range, steepness_potential_));
}

//}

} // namespace formation

PLUGINLIB_EXPORT_CLASS(formation::Formation, nodelet::Nodelet);