#include <SensorNeighbor.h>
#include <pluginlib/class_list_macros.h>
#include <mrs_lib/attitude_converter.h>

namespace sensor_neighbor{

void SensorNeighbor::onInit(){
	/* set flags to false */
	is_initialized_ = false;
	is_bag_open_ = false;

	ros::NodeHandle nh("~");

	ros::Time::waitForValid();

	mrs_lib::ParamLoader param_loader(nh, "SensorNeighbor");

	/* Load UAVs names */
	param_loader.loadParam("uav_names", _uav_names_);

	/* Load experiment parameters */
	param_loader.loadParam("experiment/duration", _duration_timer_experiment_);
	param_loader.loadParam("experiment/bag/time_step", _timer_bag_step_);

	if(!param_loader.loadedSuccessfully()){
    	ROS_ERROR("[SensorNeighbor]: failed to load non-optional parameters!");
    	ros::shutdown();
  	}

  	num_uavs_ = _uav_names_.size();

	/* Message filters */
	sub_odom_uav1_.subscribe(nh, "/"+ _uav_names_[0] +"/odometry/odom_slow", 1);
	sub_odom_uav2_.subscribe(nh, "/"+ _uav_names_[1] +"/odometry/odom_slow", 1);
	sub_odom_uav3_.subscribe(nh, "/"+ _uav_names_[2] +"/odometry/odom_slow", 1);
	sub_odom_uav4_.subscribe(nh, "/"+ _uav_names_[3] +"/odometry/odom_slow", 1);
	
	sync_.reset(new Sync(OdomPolicy(10), sub_odom_uav1_, sub_odom_uav2_, sub_odom_uav3_, sub_odom_uav4_));
	sync_->registerCallback(boost::bind(&SensorNeighbor::callbackUAVsPose, this, _1, _2, _3, _4));

	/* Publishers */
	for(int i = 0; i < int(num_uavs_); i++){
		neigbor_pub_.push_back(nh.advertise<flocking::Neighbors>("/sensor_neighbor/"+ _uav_names_[i] +"/neighbors", 1));
	}

	/* Service servers */
	srv_server_start_experiment_ = nh.advertiseService("start_experiment", &SensorNeighbor::callbackStartExperiment, this);
	srv_server_close_node_ = nh.advertiseService("close_node", &SensorNeighbor::callbackCloseNode, this);

	/* Timers */
	timer_bag_writer_ = nh.createTimer(ros::Duration(_timer_bag_step_), &SensorNeighbor::callbackTimerBagWriter, this, false, false);
	timer_experiment_ = nh.createTimer(ros::Duration(_duration_timer_experiment_ + 1), &SensorNeighbor::callbackTimerExperiment, this, false, false);

	ROS_INFO_ONCE("[SensorNeighbor]: initialized");

	is_initialized_ = true;
}

// | ---------------------- message filter callbacks ------------------------- |

/* callbackUAVsPose() //{ */

void SensorNeighbor::callbackUAVsPose(const nav_msgs::Odometry::ConstPtr& odom_uav1,
		const nav_msgs::Odometry::ConstPtr& odom_uav2,
		const nav_msgs::Odometry::ConstPtr& odom_uav3,
		const nav_msgs::Odometry::ConstPtr& odom_uav4){

	if(!is_initialized_) return;

	/* used to convert quartenion to euler */
	tf::Quaternion q;
	std::vector<double> x(num_uavs_);
	std::vector<double> y(num_uavs_);
	std::vector<double> orientation(num_uavs_);

	/* collect UAV1 heading */
  // If you search for the "bearing", the "yaw" is not what you "want".
  // ROS uses the extrinsic RPY convention, so the yawing is the last rotation and it happens around the world z-axis.
  // That means that yaw=0 does not mean anything special, both the body-X and body-Y axis can have arbitrary heading depending on pitch and roll.
  // Ofc, this is "negligable" for small tilts, but its good to know.
  // I recommend to use the "heading", in our case defined as the azimuth of the body-x axis, i.e., atan2(by, bx)
	orientation[0] = mrs_lib::AttitudeConverter(odom_uav1->pose.pose.orientation).getHeading();
	orientation[1] = mrs_lib::AttitudeConverter(odom_uav2->pose.pose.orientation).getHeading();
	orientation[3] = mrs_lib::AttitudeConverter(odom_uav3->pose.pose.orientation).getHeading();
	orientation[4] = mrs_lib::AttitudeConverter(odom_uav4->pose.pose.orientation).getHeading();

	/* create new neigbors message */
	std::vector<flocking::Neighbors> neighbor_info(num_uavs_);
	
	/* compute range and bearing */
	double range, bearing_1, bearing_2;
	for(int i = 0; i < int(num_uavs_- 1); i++){
		for(int j = i+1; j < int(num_uavs_); j++){
			range = sqrt(pow(x[i] - x[j], 2) + pow(y[i] - y[j], 2));
			bearing_1 = math_operations::relativeBearing(x[i], y[i], orientation[i], x[j], y[j]);
			bearing_2 = math_operations::relativeBearing(x[j], y[j], orientation[j], x[i], y[i]);

			/* storage values on message */
			neighbor_info[i].range.push_back(range);
			neighbor_info[j].range.push_back(range);
			neighbor_info[i].bearing.push_back(bearing_1);
			neighbor_info[j].bearing.push_back(bearing_2);
		}
	}

	/* publish info */
	for(int i = 0; i < int(num_uavs_); i++){
		neighbor_info[i].header.frame_id = _uav_names_[i] + "/local_origin";
		neighbor_info[i].header.stamp = ros::Time::now();
		neighbor_info[i].num_neighbors = num_uavs_ - 1;

		neigbor_pub_[i].publish(neighbor_info[i]);
	}

	{
		/* save UAV pose to write on the rosbag */
		std::scoped_lock lock(mutex_pose_);
		uav1_x_.data = x[0];
		uav1_y_.data = y[0];
		uav1_yaw_.data = orientation[0];

		uav2_x_.data = x[1];
		uav2_y_.data = y[1];
		uav2_yaw_.data = orientation[1];

		uav3_x_.data = x[2];
		uav3_y_.data = y[2];
		uav3_yaw_.data = orientation[2];

		uav4_x_.data = x[3];
		uav4_y_.data = y[3];
		uav4_yaw_.data = orientation[3];
	}
}

//}

// | --------------------------- timer callbacks ----------------------------- |

/* callbackTimerExperiment() \\{ */

void SensorNeighbor::callbackTimerExperiment([[maybe_unused]] const ros::TimerEvent& event){
	if(!is_initialized_) return;

	if(is_bag_open_){
		/* stop rosbag writer */
		is_bag_open_ = false;
		timer_bag_writer_.stop();

		/* close rosbag */
		try{
			bag_.close();
		}catch(...){
			ROS_ERROR("[SensorNeighbor]: Could not close the bag");
		}

		ROS_INFO_ONCE("[SensorNeighbor]: The experiment time is over. Shutting down");
	}

	/* shutdown node */
	ros::shutdown();

	return;
}

//}

/* callbackTimerBagWriter() //{ */
void SensorNeighbor::callbackTimerBagWriter([[maybe_unused]] const ros::TimerEvent& event){
	if(!is_initialized_ || !is_bag_open_) return;

	/* write data on rosbag */
	ros::Time t = ros::Time::now();
	{
		std::scoped_lock lock(mutex_pose_);
		try{
			/* UAV1 data */
			bag_.write("uav1_x", t, uav1_x_);
			bag_.write("uav1_y", t, uav1_y_);
			bag_.write("uav1_yaw", t, uav1_yaw_);
			
			/* UAV2 data */
			bag_.write("uav2_x", t, uav2_x_);
			bag_.write("uav2_y", t, uav2_y_);
			bag_.write("uav2_yaw", t, uav2_yaw_);
			
			/* UAV3 data */
			bag_.write("uav3_x", t, uav3_x_);
			bag_.write("uav3_y", t, uav3_y_);
			bag_.write("uav3_yaw", t, uav3_yaw_);

			/* UAV4 data */
			bag_.write("uav4_x", t, uav4_x_);
			bag_.write("uav4_y", t, uav4_y_);
			bag_.write("uav4_yaw", t, uav4_yaw_);
		}catch(...){
			ROS_ERROR("[SensorNeighbor]: Could not write on rosbag file");
		}
	}
}

//}

// | ----------------------- service server callbacks ------------------------ |

/* callbackStartExperiment() //{ */

bool SensorNeighbor::callbackStartExperiment([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res){
	if(is_bag_open_){
		ROS_WARN("[SensorNeighbor]: Request to start the experiment but it is already started");
		res.success = true;
		res.message = "[SensorNeighbor]: Request to start the experiment but it is already started";
		return true;
	}

	/* generate rosbag name */
	time(&rawtime_);
	timeinfo_ = localtime (&rawtime_);
	strftime(buffer_,80,"%m-%d-%H:%M",timeinfo_);
	bag_name_ = "flocking-"+ std::string(buffer_) +".bag";
	
	ROS_INFO_ONCE("[SensorNeighbor]: Saving data of the experiment on the file %s", bag_name_.c_str());

	/* open rosbag */
	try{
		bag_.open(bag_name_, rosbag::bagmode::Write);
	}catch(...){
		ROS_ERROR("[SensorNeighbor]: Could not open the bag %s. Shutting down", bag_name_.c_str());
		res.success = false;
		res.message = "[SensorNeighbor]: Could not open the bag %s. Shutting down", bag_name_.c_str();
		
		/* shutdown node if could not open the rosbag */
		ros::shutdown();
		return true;
	}
	
	/* start rosbag writer */
	is_bag_open_ = true;
	timer_bag_writer_.start();
	
	/* start experiment countdown */
	timer_experiment_.start();

	ROS_INFO("[SensorNeighbor]: The experiment has started and will last %s seconds", std::to_string(_duration_timer_experiment_).c_str());
	res.success = true;
	res.message = "[SensorNeighbor]: The experiment has started and will last "+ std::to_string(_duration_timer_experiment_)  +" seconds";
	return true;
}

//}

/* callbackCloseNode() //{ */

bool SensorNeighbor::callbackCloseNode([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res){
	if(is_bag_open_){
		/* stop rosbag writer */
		timer_bag_writer_.stop();

		/* close rosbag */
		bag_.close();

		/* stop experiment countdown */
		timer_experiment_.stop();
	}

	ROS_INFO_ONCE("[SensorNeighbor]: Close node called. Shutting down");
	res.success = true;
	res.message = "[SensorNeighbor]: Close node called. Shutting down";

	/* shutdown node */
	ros::shutdown();

	return true;
}

//}

} // namespace sensor_neighbor

PLUGINLIB_EXPORT_CLASS(sensor_neighbor::SensorNeighbor, nodelet::Nodelet);
