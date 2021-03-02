#include <SensorNeighbor.h>
#include <pluginlib/class_list_macros.h>

namespace sensor_neighbor
{

/* onInit() //{ */

void SensorNeighbor::onInit() {
  /* set flags to false */
  is_initialized_ = false;
  has_this_pose_  = false;

  ros::NodeHandle nh("~");

  ros::Time::waitForValid();

  mrs_lib::ParamLoader param_loader(nh, "SensorNeighbor");

  /* load parameters */
  param_loader.loadParam("sensor_type", _sensor_type_);
  param_loader.loadParam("use_3D", _use_3D_);
  
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
      
      /* subscribe to slow_odom */
      sub_odom_uavs_.push_back(nh.subscribe<nav_msgs::Odometry>("/" + _uav_names_[i] + "/odometry/slow_odom", 1,
                               boost::bind(&SensorNeighbor::callbackNeighborsUsingGPSOdom, this, _1, uav_id)));
      
      /* subscribe to odom_local */
      sub_odom_local_uavs_.push_back(nh.subscribe<nav_msgs::Odometry>("/" + _uav_names_[i] + "/odometry/odom_local", 1,
                                     boost::bind(&SensorNeighbor::callbackNeighborsUsingGPSOdomLocal, this, _1, uav_id)));
    }
    
    sub_this_uav_local_odom_ = nh.subscribe<nav_msgs::Odometry>("/" + _this_uav_name_ + "/odometry/odom_local", 1,
                               &SensorNeighbor::callbackThisUAVLocalOdom, this);

  } else if (_sensor_type_ == "uvdar") {
    /* subscribe to UVDAR filtered poses */
    sub_uvdar_filtered_poses_ = nh.subscribe<mrs_msgs::PoseWithCovarianceArrayStamped>("/" + _this_uav_name_ + "/uvdar/filteredPoses", 1,
                                &SensorNeighbor::callbackNeighborsUsingUVDAR, this);
  } else {
    ROS_ERROR("[SensorNeighbor]: The sensor %s is not supported. Shutting down.", _sensor_type_.c_str());
    ros::shutdown();
  }

  /* subscribe to this UAV Odometry */
  sub_this_uav_odom_ = nh.subscribe<nav_msgs::Odometry>("/" + _this_uav_name_ + "/odometry/odom_main", 1, &SensorNeighbor::callbackThisUAVOdom, this);

  /* publisher */
  pub_neighbors_ = nh.advertise<flocking::Neighbors>("/" + _this_uav_name_ + "/sensor_neighbor/neighbors", 1);

  /* timers */
  timer_pub_neighbors_ = nh.createTimer(ros::Rate(5.0), &SensorNeighbor::callbackTimerPubNeighbors, this);

  /* transformer */
  tfr_ = mrs_lib::Transformer("SensorNeighbor", _this_uav_name_);

  ROS_INFO_ONCE("[SensorNeighbor]: initialized");
  is_initialized_ = true;

  ros::spin();
}

//}

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

/* callbackThisUAVLocalOdom() //{ */

void SensorNeighbor::callbackThisUAVLocalOdom(const nav_msgs::Odometry::ConstPtr& odom) {
  if (!is_initialized_) {
    return;
  }

  {
    std::scoped_lock lock(mutex_neighbors_height_);

    /* update local height */
    this_uav_local_height_.value  = odom->pose.pose.position.z;
    this_uav_local_height_.header = odom->header;

    /* turn on flag */
    has_this_uav_local_height_ = true;
  }
}

//}

/* callbackNeighborsUsingGPSOdom() //{ */

void SensorNeighbor::callbackNeighborsUsingGPSOdom(const nav_msgs::Odometry::ConstPtr& odom, const unsigned int uav_id) {
  if (!is_initialized_) {
    return;
  }

  {
    std::scoped_lock lock(mutex_neighbors_position_2d_);

    /* create new msg */
    flocking::Point2DStamped uav_point;

    /* fill in msg */
    uav_point.x      = odom->pose.pose.position.x;
    uav_point.y      = odom->pose.pose.position.y;
    uav_point.header = odom->header;

    /* store position */
    if (neighbors_position_2d_.find(uav_id) == neighbors_position_2d_.end()) {
      neighbors_position_2d_.insert(std::pair<unsigned int, flocking::Point2DStamped>(uav_id, uav_point));
    } else {
      neighbors_position_2d_[uav_id] = uav_point;
    }
  }
}

//}


/* callbackNeighborsUsingGPSOdomLocal //{ */

void SensorNeighbor::callbackNeighborsUsingGPSOdomLocal(const nav_msgs::Odometry::ConstPtr& odom, const unsigned int uav_id) {
  if (!is_initialized_) {
    return;
  }

  {
    std::scoped_lock lock(mutex_neighbors_height_);
    
    /* create new msg */
    mrs_msgs::Float64Stamped uav_height;

    /* fill in msg */
    uav_height.value  = odom->pose.pose.position.z;
    uav_height.header = odom->header;

    /* store height */
    if (neighbors_height_.find(uav_id) == neighbors_height_.end()) {
      neighbors_height_.insert(std::pair<unsigned int, mrs_msgs::Float64Stamped>(uav_id, uav_height));
    } else {
      neighbors_height_[uav_id] = uav_height;
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

  double max_height_difference = 0.0;

  if (_sensor_type_ == "gps") {
    {
      std::scoped_lock lock(mutex_neighbors_position_2d_);

      for (auto itr_point = neighbors_position_2d_.begin(); itr_point != neighbors_position_2d_.end(); ++itr_point) {      
        /* check 2D position stamp */
        if ((now - itr_point->second.header.stamp).toSec() > 2.0) continue;

        /* estimate bearing */
        const double bearing = math_utils::relativeBearing(focal_x, focal_y, focal_heading, itr_point->second.x, itr_point->second.y);
        
        double range, inclination;
        
        {
          std::scoped_lock lock(mutex_neighbors_height_);
          
          /* check if the height of the uav in the local_origin frame is available */
          if (neighbors_height_.find(itr_point->first) == neighbors_height_.end()) continue;

          /* check height stamp */
          unsigned int uav_id = itr_point->first;
          if ((now - neighbors_height_[uav_id].header.stamp).toSec() > 2.0) continue;

          /* check if this uav local height is available and local height stamp */
          if (!has_this_uav_local_height_ || (now - this_uav_local_height_.header.stamp).toSec() > 2.0) continue;

          if (_use_3D_) { 
            /* estimate range and inclination */
            range       = sqrt(pow(focal_x - itr_point->second.x, 2) + pow(focal_y - itr_point->second.y,
                               2) + pow(this_uav_local_height_.value - neighbors_height_[uav_id].value, 2));

            inclination = math_utils::inclination(focal_x, focal_y, this_uav_local_height_.value, itr_point->second.x, itr_point->second.y,
                                                  neighbors_height_[uav_id].value);
          } else { /* using 2D */
            unsigned int uav_id   = itr_point->first;
            max_height_difference = math_utils::getMaxValue(max_height_difference, neighbors_height_[uav_id].value - this_uav_local_height_.value);

            /* estimate range and inclination */
            range       = sqrt(pow(focal_x - itr_point->second.x, 2) + pow(focal_y - itr_point->second.y, 2));
            inclination = M_PI / 2;
          }
        }
        
        neighbor_info.range.push_back(range);
        neighbor_info.bearing.push_back(bearing);
        neighbor_info.inclination.push_back(inclination);
      }
    }
  } else { /* using uvdar - always 2D */
    std::scoped_lock lock(mutex_neighbors_position_);

    for (auto itr = neighbors_position_.begin(); itr != neighbors_position_.end(); ++itr) {
      if ((now - itr->second.header.stamp).toSec() < 2.0) {
        /* estimate bearing, range and inclination */
        const double bearing     = math_utils::relativeBearing(focal_x, focal_y, focal_heading, itr->second.point.x, itr->second.point.y);
        const double range       = sqrt(pow(focal_x - itr->second.point.x, 2) + pow(focal_y - itr->second.point.y, 2));
        const double inclination = M_PI / 2;

        /* estimate max height difference */
        max_height_difference = math_utils::getMaxValue(max_height_difference, itr->second.point.z);

        neighbor_info.range.push_back(range);
        neighbor_info.bearing.push_back(bearing);
        neighbor_info.inclination.push_back(inclination);
      }
    }
  }

  neighbor_info.header.frame_id = _this_uav_name_ + "/fcu";
  neighbor_info.header.stamp    = now;
  neighbor_info.num_neighbors   = neighbor_info.range.size();
  neighbor_info.max_height_diff = max_height_difference;

  pub_neighbors_.publish(neighbor_info);
}

//}

}  // namespace sensor_neighbor

PLUGINLIB_EXPORT_CLASS(sensor_neighbor::SensorNeighbor, nodelet::Nodelet);
