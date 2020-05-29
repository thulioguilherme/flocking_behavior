#include <SensorNeighbor.h>
#include <pluginlib/class_list_macros.h>
#include <mrs_lib/attitude_converter.h>

namespace sensor_neighbor
{

void SensorNeighbor::onInit() {
  /* set flags to false */
  is_initialized_ = false;
  is_bag_open_    = false;

  ros::NodeHandle nh("~");

  ros::Time::waitForValid();

  mrs_lib::ParamLoader param_loader(nh, "SensorNeighbor");

  /* Load UAVs names */
  param_loader.loadParam("uav_name", this_uav_name_);
  param_loader.loadParam("uav_names", _uav_names_);

  /* Load experiment parameters */
  param_loader.loadParam("experiment/duration", _duration_timer_experiment_);
  /* param_loader.loadParam("experiment/bag/time_step", _timer_bag_step_); */

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[SensorNeighbor]: failed to load non-optional parameters!");
    ros::shutdown();
  }

  auto itr = std::find(_uav_names_.begin(), _uav_names_.end(), this_uav_name_);
  if (itr != _uav_names_.cend()) {
    this_uav_name_idx_ = std::distance(_uav_names_.begin(), itr);
  } else {
    ROS_ERROR("UAV name %s is not in the list of all UAVs.", this_uav_name_.c_str());
    ros::shutdown();
  }

  num_uavs_ = _uav_names_.size();

  /* Message filters */
  const std::string topic_name_1 =
      _uav_names_[0] != this_uav_name_ ? "/" + _uav_names_[0] + "/odometry/slow_odom" : "/" + _uav_names_[0] + "/odometry/odom_main";
  const std::string topic_name_2 =
      _uav_names_[1] != this_uav_name_ ? "/" + _uav_names_[1] + "/odometry/slow_odom" : "/" + _uav_names_[1] + "/odometry/odom_main";
  const std::string topic_name_3 =
      _uav_names_[2] != this_uav_name_ ? "/" + _uav_names_[2] + "/odometry/slow_odom" : "/" + _uav_names_[2] + "/odometry/odom_main";
  const std::string topic_name_4 =
      _uav_names_[3] != this_uav_name_ ? "/" + _uav_names_[3] + "/odometry/slow_odom" : "/" + _uav_names_[3] + "/odometry/odom_main";
  sub_odom_uav1_.subscribe(nh, topic_name_1, 1);
  sub_odom_uav2_.subscribe(nh, topic_name_2, 1);
  sub_odom_uav3_.subscribe(nh, topic_name_3, 1);
  sub_odom_uav4_.subscribe(nh, topic_name_4, 1);

  sync_.reset(new Sync(OdomPolicy(20), sub_odom_uav1_, sub_odom_uav2_, sub_odom_uav3_, sub_odom_uav4_));
  sync_->registerCallback(boost::bind(&SensorNeighbor::callbackUAVsPose, this, _1, _2, _3, _4));

  /* Decentralization of this node (will be run on every unit independently) */
  neigbor_pub_ = nh.advertise<flocking::Neighbors>("/" + this_uav_name_  + "/sensor_neighbor/neighbors", 1);

  /* Service servers */
  /* srv_server_start_experiment_ = nh.advertiseService("start_experiment", &SensorNeighbor::callbackStartExperiment, this); */
  /* srv_server_close_node_       = nh.advertiseService("close_node", &SensorNeighbor::callbackCloseNode, this); */

  /* Timers */
  /* timer_bag_writer_ = nh.createTimer(ros::Duration(_timer_bag_step_), &SensorNeighbor::callbackTimerBagWriter, this, false, false); */
  /* timer_experiment_ = nh.createTimer(ros::Duration(_duration_timer_experiment_ + 1), &SensorNeighbor::callbackTimerExperiment, this, false, false); */

  ROS_INFO_ONCE("[SensorNeighbor]: initialized");
  is_initialized_ = true;

  ros::spin();
}

// | ---------------------- message filter callbacks ------------------------- |

/* callbackUAVsPose() //{ */

void SensorNeighbor::callbackUAVsPose(const nav_msgs::Odometry::ConstPtr& odom_uav1, const nav_msgs::Odometry::ConstPtr& odom_uav2,
                                      const nav_msgs::Odometry::ConstPtr& odom_uav3, const nav_msgs::Odometry::ConstPtr& odom_uav4) {

  if (!is_initialized_)
    return;

  /* used to convert quartenion to euler */
  tf::Quaternion      q;
  std::vector<double> x(num_uavs_);
  std::vector<double> y(num_uavs_);
  std::vector<double> orientation(num_uavs_);

  std::vector<nav_msgs::Odometry> odoms = {*odom_uav1, *odom_uav2, *odom_uav3, *odom_uav4};
  for (unsigned int i = 0; i < odoms.size(); i++) {
    x[i] = odoms[i].pose.pose.position.x;
    y[i] = odoms[i].pose.pose.position.y;

    // If you search for the "bearing", the "yaw" is not what you "want".
    // ROS uses the extrinsic RPY convention, so the yawing is the last rotation and it happens around the world z-axis.
    // That means that yaw=0 does not mean anything special, both the body-X and body-Y axis can have arbitrary heading depending on pitch and roll.
    // Ofc, this is "negligable" for small tilts, but its good to know.
    // I recommend to use the "heading", in our case defined as the azimuth of the body-x axis, i.e., atan2(by, bx)
    orientation[i] = mrs_lib::AttitudeConverter(odoms[i].pose.pose.orientation).getHeading();
  }

  /* create new neigbors message */
  flocking::Neighbors neighbor_info;

  /* compute range and bearing */
  double range, bearing_1;
  for (unsigned int j = 0; j < (unsigned int)(num_uavs_); j++) {
    if (this_uav_name_idx_ != j) {
      range     = sqrt(pow(x[this_uav_name_idx_] - x[j], 2) + pow(y[this_uav_name_idx_] - y[j], 2));
      bearing_1 = math_operations::relativeBearing(x[this_uav_name_idx_], y[this_uav_name_idx_], orientation[this_uav_name_idx_], x[j], y[j]);

      /* storage values on message */
      neighbor_info.range.push_back(range);
      neighbor_info.bearing.push_back(bearing_1);
    }
  }

  neighbor_info.header.frame_id = this_uav_name_ + "/local_origin";
  neighbor_info.header.stamp    = ros::Time::now();
  neighbor_info.num_neighbors   = num_uavs_ - 1;

  neigbor_pub_.publish(neighbor_info);

}  // namespace sensor_neighbor

//}

// | --------------------------- timer callbacks ----------------------------- |

/* callbackTimerExperiment() //{*/
void SensorNeighbor::callbackTimerExperiment([[maybe_unused]] const ros::TimerEvent& event) {
  if (!is_initialized_)
    return;

  if (is_bag_open_) {
    /* stop rosbag writer */
    is_bag_open_ = false;
    timer_bag_writer_.stop();

    /* close rosbag */
    try {
      bag_.close();
    }
    catch (...) {
      ROS_ERROR("[SensorNeighbor]: Could not close the bag");
    }

    ROS_INFO_ONCE("[SensorNeighbor]: The experiment time is over. Shutting down");
  }

  /* shutdown node */
  ros::shutdown();

  return;
}
/*//}*/

/* callbackTimerBagWriter() //{ */
void SensorNeighbor::callbackTimerBagWriter([[maybe_unused]] const ros::TimerEvent& event) {
  if (!is_initialized_ || !is_bag_open_)
    return;

  /* write data on rosbag */
  ros::Time t = ros::Time::now();
  {
    std::scoped_lock lock(mutex_pose_);
    try {
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
    }
    catch (...) {
      ROS_ERROR("[SensorNeighbor]: Could not write on rosbag file");
    }
  }
}

//}

// | ----------------------- service server callbacks ------------------------ |

/* callbackStartExperiment() //{ */

bool SensorNeighbor::callbackStartExperiment([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  if (is_bag_open_) {
    ROS_WARN("[SensorNeighbor]: Request to start the experiment but it is already started");
    res.success = true;
    res.message = "[SensorNeighbor]: Request to start the experiment but it is already started";
    return true;
  }

  /* generate rosbag name */
  time(&rawtime_);
  timeinfo_ = localtime(&rawtime_);
  strftime(buffer_, 80, "%m-%d-%H:%M", timeinfo_);
  bag_name_ = "flocking-" + std::string(buffer_) + ".bag";

  ROS_INFO_ONCE("[SensorNeighbor]: Saving data of the experiment on the file %s", bag_name_.c_str());

  /* open rosbag */
  try {
    bag_.open(bag_name_, rosbag::bagmode::Write);
  }
  catch (...) {
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
  res.message = "[SensorNeighbor]: The experiment has started and will last " + std::to_string(_duration_timer_experiment_) + " seconds";
  return true;
}

//}

/* callbackCloseNode() //{ */

bool SensorNeighbor::callbackCloseNode([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  if (is_bag_open_) {
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

}  // namespace sensor_neighbor

PLUGINLIB_EXPORT_CLASS(sensor_neighbor::SensorNeighbor, nodelet::Nodelet);
