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

  /* load UAVs names */
  param_loader.loadParam("uav_name", _this_uav_name_);
  param_loader.loadParam("uav_names", _uav_names_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[SensorNeighbor]: failed to load non-optional parameters!");
    ros::shutdown();
  }

  auto itr = std::find(_uav_names_.begin(), _uav_names_.end(), _this_uav_name_);
  if (itr != _uav_names_.cend()) {
    this_uav_name_idx_ = std::distance(_uav_names_.begin(), itr);
  } else {
    ROS_ERROR("UAV name %s is not in the list of all UAVs.", _this_uav_name_.c_str());
    ros::shutdown();
  }

  num_other_uavs_ = _uav_names_.size() - 1;

  /* subscribers */
  std::string topic_name;
  for (int i = 0; i < _uav_names_.size(); i++) {
    topic_name = _uav_names_[i] != _this_uav_name_ ? "/" + _uav_names_[i] + "/odometry/slow_odom" : "/" + _uav_names_[i] + "/odometry/odom_main";
    sub_odom_uavs_.push_back(nh.subscribe<nav_msgs::Odometry>(topic_name, 1, boost::bind(&SensorNeighbor::callbackUAVOdom, this, _1, _uav_names_[i])));
  }

  /* decentralization of this node (will be run on every unit independently) */
  neigbor_pub_ = nh.advertise<flocking::Neighbors>("/" + _this_uav_name_ + "/sensor_neighbor/neighbors", 1);

  /* timers */
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

  if (uav_name == _this_uav_name_) {
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

  /* get odometry of this UAV */
  ros::Time          now = ros::Time::now();
  nav_msgs::Odometry odom_this;
  {
    std::scoped_lock lock(mutex_odoms_);
    odom_this           = odoms_[_this_uav_name_];
    double heading_this = mrs_lib::AttitudeConverter(odom_this.pose.pose.orientation).getHeading();

    /* compute relative info to all neighbors */
    for (auto itr = odoms_.begin(); itr != odoms_.end(); ++itr) {
      if (itr->first == _this_uav_name_) {
        continue;
      }

      if ((now - itr->second.header.stamp).toSec() < 1.0) {
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
  if (neighbor_info.range.size() == num_other_uavs_) {
    neighbor_info.header.frame_id = _this_uav_name_ + "/local_origin";
    neighbor_info.header.stamp    = now;
    neighbor_info.num_neighbors   = neighbor_info.range.size();

    neigbor_pub_.publish(neighbor_info);
  }
}

//}

}  // namespace sensor_neighbor

PLUGINLIB_EXPORT_CLASS(sensor_neighbor::SensorNeighbor, nodelet::Nodelet);
