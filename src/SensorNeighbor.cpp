#include <SensorNeighbor.h>
#include <pluginlib/class_list_macros.h>

namespace sensor_neighbor
{

/*//{ onInit() */

void SensorNeighbor::onInit() {
  /* set flags to false */
  is_initialized_ = false;
  has_this_pose_  = false;

  ros::NodeHandle nh("~");

  ros::Time::waitForValid();

  mrs_lib::ParamLoader param_loader(nh, "SensorNeighbor");

  /* load parameters */
  param_loader.loadParam("sensor_type", _sensor_type_);
  param_loader.loadParam("uav_name", _this_uav_name_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[SensorNeighbor]: failed to load non-optional parameters!");
    ros::shutdown();
  }

  /* instantiate subscribers based on used sensor type */
  if (_sensor_type_ == "gps") {
    /* load others uav name */
    param_loader.loadParam("uav_names", _uav_names_);

    if (!param_loader.loadedSuccessfully()) {
      ROS_ERROR("[SensorNeighbor]: failed to load non-optional parameters!");
      ros::shutdown();
    }

    /* check if this UAV name is in the list of all UAVs */
    auto itr = std::find(_uav_names_.begin(), _uav_names_.end(), _this_uav_name_);
    if (itr == _uav_names_.cend()) {
      ROS_ERROR("UAV name %s is not in the list of all UAVs.", _this_uav_name_.c_str());
      ros::shutdown();
    }

    /* subscribe to others UAV Odometry */
    for (unsigned int i = 0; i < _uav_names_.size(); i++) {
      if (_uav_names_[i] == _this_uav_name_) {
        continue;
      }

      /* generate UAV index using UAV name */
      unsigned int uav_id = std::stoi(_uav_names_[i].substr(3));
      sub_odom_uavs_.push_back(nh.subscribe<nav_msgs::Odometry>("/" + _uav_names_[i] + "/odometry/slow_odom", 1,
                                                                boost::bind(&SensorNeighbor::callbackNeighborsUsingGPS, this, _1, uav_id)));
    }

    /* subscribe to this UAV Odometry */
    sub_this_uav_odom_ = nh.subscribe<nav_msgs::Odometry>("/" + _this_uav_name_ + "/odometry/odom_main", 1, &SensorNeighbor::callbackThisUAVOdom, this);

  } else if (_sensor_type_ == "uvdar") {
    /* subscribe to UVDAR filtered poses */
    sub_uvdar_filtered_poses_ = nh.subscribe<mrs_msgs::PoseWithCovarianceArrayStamped>("/" + _this_uav_name_ + "/uvdar/filteredPoses", 1,
                                                                                       &SensorNeighbor::callbackNeighborsUsingUVDAR, this);

    /* subscribe to this UAV Odometry */
    sub_this_uav_odom_ = nh.subscribe<nav_msgs::Odometry>("/" + _this_uav_name_ + "/odometry/odom_main", 1, &SensorNeighbor::callbackThisUAVOdom, this);
  } else {
    ROS_ERROR("[SensorNeighbor]: The sensor %s is not supported. Shutting down.", _sensor_type_.c_str());
    ros::shutdown();
  }

  /* publisher */
  neigbor_pub_ = nh.advertise<flocking::Neighbors>("/" + _this_uav_name_ + "/sensor_neighbor/neighbors", 1);

  /* timers */
  timer_pub_neighbors_ = nh.createTimer(ros::Rate(5.0), &SensorNeighbor::callbackTimerPubNeighbors, this);

  tfr_ = mrs_lib::Transformer("SensorNeighbor", _this_uav_name_);

  ROS_INFO_ONCE("[SensorNeighbor]: initialized");
  is_initialized_ = true;

  ros::spin();
}

/*//}*/

// | ---------------------- subscriber callbacks ------------------------- |

/* callbackThisUAVOdom() //{ */

void SensorNeighbor::callbackThisUAVOdom(const nav_msgs::Odometry::ConstPtr& odom) {
  if (!is_initialized_) {
    return;
  }

  {
    std::scoped_lock lock(mutex_this_uav_pose_);

    /* update position */
    this_uav_pose_.pose   = odom->pose.pose;
    this_uav_pose_.header = odom->header;

    /* turn on flag */
    has_this_pose_ = true;
  }
}

//}

/* callbackNeighborsUsingGPS() //{ */

void SensorNeighbor::callbackNeighborsUsingGPS(const nav_msgs::Odometry::ConstPtr& odom, const unsigned int uav_id) {
  if (!is_initialized_) {
    return;
  }

  {
    std::scoped_lock lock(mutex_neighbors_position_);

    /* create new msg */
    geometry_msgs::PointStamped uav_point;

    /* fill in msg */
    uav_point.point.x = odom->pose.pose.position.x;
    uav_point.point.y = odom->pose.pose.position.y;
    uav_point.point.z = odom->pose.pose.position.z;
    uav_point.header  = odom->header;

    /* save last estimated position */
    if (neighbors_position_.find(uav_id) == neighbors_position_.end()) {
      neighbors_position_.insert(std::pair<unsigned int, geometry_msgs::PointStamped>(uav_id, uav_point));
    } else {
      neighbors_position_[uav_id] = uav_point;
    }
  }
}

//}

/* callbackNeighborsUsingUVDAR //{ */

void SensorNeighbor::callbackNeighborsUsingUVDAR(const mrs_msgs::PoseWithCovarianceArrayStamped::ConstPtr& array_poses) {
  if (!is_initialized_ || !has_this_pose_) {
    return;
  }

  std::string odom_frame_id;
  {
    std::scoped_lock lock(mutex_this_uav_pose_);
    odom_frame_id = this_uav_pose_.header.frame_id;
  }

  auto tf = tfr_.getTransform(array_poses->header.frame_id, odom_frame_id);
  if (!tf.has_value()) {
    ROS_WARN("Could not transform pose from %s to %s", array_poses->header.frame_id.c_str(), odom_frame_id.c_str());
    return;
  }

  {
    std::scoped_lock lock(mutex_neighbors_position_);

    for (unsigned int i = 0; i < array_poses->poses.size(); i++) {
      /* create new msg */
      geometry_msgs::PointStamped uav_point;

      uav_point.point.x = array_poses->poses[i].pose.position.x;
      uav_point.point.y = array_poses->poses[i].pose.position.y;
      uav_point.point.z = array_poses->poses[i].pose.position.z;
      uav_point.header  = array_poses->header;


      auto uav_point_transformed = tfr_.transform(tf.value(), uav_point);

      if (uav_point_transformed.has_value()) {

        /* std::cout << "x: " << uav_point_transformed.value().point.x << " / y: " << uav_point_transformed.value().point.y << "\n"; */

        /* save estimated position */
        const unsigned int uav_id = array_poses->poses[i].id;

        if (neighbors_position_.find(uav_id) == neighbors_position_.end()) {
          neighbors_position_.insert(std::pair<unsigned int, geometry_msgs::PointStamped>(uav_id, uav_point_transformed.value()));
        } else {
          neighbors_position_[uav_id] = uav_point_transformed.value();
        }
      }
    }
  }
}

//}

// | --------------------------- timer callbacks ----------------------------- |

/* callbackTimerPubNeighbors() //{ */

void SensorNeighbor::callbackTimerPubNeighbors([[maybe_unused]] const ros::TimerEvent& event) {
  if (!is_initialized_ || !has_this_pose_) {
    return;
  }

  /* create new neigbors message */
  flocking::Neighbors neighbor_info;
  const ros::Time     now = ros::Time::now();

  /* get pose of this UAV */
  double focal_x, focal_y, focal_heading;
  {
    std::scoped_lock lock(mutex_this_uav_pose_);
    focal_x       = this_uav_pose_.pose.position.x;
    focal_y       = this_uav_pose_.pose.position.y;
    focal_heading = mrs_lib::AttitudeConverter(this_uav_pose_.pose.orientation).getHeading();
  }

  {
    std::scoped_lock lock(mutex_neighbors_position_);

    for (auto itr = neighbors_position_.begin(); itr != neighbors_position_.end(); ++itr) {
      if ((now - itr->second.header.stamp).toSec() < 2.0) {
        const double range   = sqrt(pow(focal_x - itr->second.point.x, 2) + pow(focal_y - itr->second.point.y, 2));
        const double bearing = math_utils::relativeBearing(focal_x, focal_y, focal_heading, itr->second.point.x, itr->second.point.y);

        neighbor_info.range.push_back(range);
        neighbor_info.bearing.push_back(bearing);
      }
    }
  }

  neighbor_info.header.frame_id = _this_uav_name_ + "/fcu";
  neighbor_info.header.stamp    = now;
  neighbor_info.num_neighbors   = neighbor_info.range.size();

  neigbor_pub_.publish(neighbor_info);
}

//}

}  // namespace sensor_neighbor

PLUGINLIB_EXPORT_CLASS(sensor_neighbor::SensorNeighbor, nodelet::Nodelet);
