#include <SensorNeighbor.h>
#include <pluginlib/class_list_macros.h>

namespace sensor_neighbor
{

void SensorNeighbor::onInit() {
  /* set flags to false */
  is_initialized_ = false;

  ros::NodeHandle nh("~");

  ros::Time::waitForValid();

  mrs_lib::ParamLoader param_loader(nh, "SensorNeighbor");

  /* Load UAVs names */
  param_loader.loadParam("uav_name", this_uav_name_);
  param_loader.loadParam("uav_names", _uav_names_);

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

  num_other_uavs_ = _uav_names_.size() - 1;

  /* Topics */
  const std::string topic_name_1 =
      _uav_names_[0] != this_uav_name_ ? "/" + _uav_names_[0] + "/odometry/slow_odom" : "/" + _uav_names_[0] + "/odometry/odom_main";
  const std::string topic_name_2 =
      _uav_names_[1] != this_uav_name_ ? "/" + _uav_names_[1] + "/odometry/slow_odom" : "/" + _uav_names_[1] + "/odometry/odom_main";
  const std::string topic_name_3 =
      _uav_names_[2] != this_uav_name_ ? "/" + _uav_names_[2] + "/odometry/slow_odom" : "/" + _uav_names_[2] + "/odometry/odom_main";
  
  /* Subscribers */
  sub_odom_uav1_ = nh.subscribe<nav_msgs::Odometry>(topic_name_1, 1, boost::bind(&SensorNeighbor::callbackUAVOdom, this, _1, _uav_names_[0]));
  sub_odom_uav2_ = nh.subscribe<nav_msgs::Odometry>(topic_name_2, 1, boost::bind(&SensorNeighbor::callbackUAVOdom, this, _1, _uav_names_[1]));
  sub_odom_uav3_ = nh.subscribe<nav_msgs::Odometry>(topic_name_3, 1, boost::bind(&SensorNeighbor::callbackUAVOdom, this, _1, _uav_names_[2]));
  
  /* Decentralization of this node (will be run on every unit independently) */
  neigbor_pub_ = nh.advertise<flocking::Neighbors>("/" + this_uav_name_ + "/sensor_neighbor/neighbors", 1);

  /* Timers */
  timer_pub_neighbors_ = nh.createTimer(ros::Rate(5.0), &SensorNeighbor::callbackTimerPubNeighbors, this);
  
  ROS_INFO_ONCE("[SensorNeighbor]: initialized");
  is_initialized_ = true;

  ros::spin();
}

// | ---------------------- subscriber callbacks ------------------------- |

/* callbackUAVOdom() //{ */

void SensorNeighbor::callbackUAVOdom(const nav_msgs::Odometry::ConstPtr& odom, const std::string uav_name) {

  if (!is_initialized_) {
    return;
  }
  {
    std::scoped_lock lock(mutex_odoms_);
    if (odoms_.find(uav_name) == odoms_.end()) {
      odoms_.insert(std::pair<std::string, nav_msgs::Odometry>(uav_name, *odom));
    } else {
      odoms_[uav_name] = *odom;
    }
  }
  if (uav_name == this_uav_name_) {
    has_odom_this_ = true;
  }
}

//}


// | --------------------------- timer callbacks ----------------------------- |

/* callbackTimerPubNeighbors() //{ */

void SensorNeighbor::callbackTimerPubNeighbors([[maybe_unused]] const ros::TimerEvent& event) {

  if (!is_initialized_ || !has_odom_this_) {
    return;
  }

  /* create new neigbors message */
  flocking::Neighbors neighbor_info;

  // get odometry of this UAV
  ros::Time          now = ros::Time::now();
  nav_msgs::Odometry odom_this;
  {
    std::scoped_lock lock(mutex_odoms_);
    odom_this           = odoms_[this_uav_name_];
    double heading_this = mrs_lib::AttitudeConverter(odom_this.pose.pose.orientation).getHeading();

    // compute relative info to all neighbors
    for (auto itr = odoms_.begin(); itr != odoms_.end(); ++itr) {
      if (itr->first == this_uav_name_) {
        continue;
      }

      // Check if we have new messages (slow_odom runs at 2Hz, so we give it small reserve)
      if ((now - itr->second.header.stamp).toSec() < 1.0) {

        // If you search for the "bearing", the "yaw" is not what you "want".
        // ROS uses the extrinsic RPY convention, so the yawing is the last rotation and it happens around the world z-axis.
        // That means that yaw=0 does not mean anything special, both the body-X and body-Y axis can have arbitrary heading depending on pitch and roll.
        // Ofc, this is "negligable" for small tilts, but its good to know.
        // I recommend to use the "heading", in our case defined as the azimuth of the body-x axis, i.e., atan2(by, bx)
        double x = itr->second.pose.pose.position.x;
        double y = itr->second.pose.pose.position.y;

        double range   = sqrt(pow(odom_this.pose.pose.position.x - x, 2) + pow(odom_this.pose.pose.position.y - y, 2));
        double bearing = math_operations::relativeBearing(odom_this.pose.pose.position.x, odom_this.pose.pose.position.y, heading_this, x, y);
        neighbor_info.range.push_back(range);
        neighbor_info.bearing.push_back(bearing);
      }
    }
  }

  /* only publish if have the information of all the others uavs */
  if(neighbor_info.range.size() == num_other_uavs_){
    neighbor_info.header.frame_id = this_uav_name_ + "/local_origin";
    neighbor_info.header.stamp    = now;
    neighbor_info.num_neighbors   = neighbor_info.range.size();
    neigbor_pub_.publish(neighbor_info);
  }
}

//}

}  // namespace sensor_neighbor

PLUGINLIB_EXPORT_CLASS(sensor_neighbor::SensorNeighbor, nodelet::Nodelet);
