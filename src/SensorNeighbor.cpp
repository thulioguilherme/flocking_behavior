#include <SensorNeighbor.h>
#include <pluginlib/class_list_macros.h>

namespace sensor_neighbor {

  void SensorNeighbor::onInit() {
    /* set flags to false */
    is_initialized_ = false;
    has_this_pose_ = false;

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
    
    } else if (_sensor_type_ == "uvdar") {
      /* subscribe to UVDAR filtered poses */
      sub_uvdar_filtered_poses_ = nh.subscribe<mrs_msgs::PoseWithCovarianceArrayStamped>("/" + _this_uav_name_ + "/uvdar/filteredPoses", 1, &SensorNeighbor::callbackNeighborsUsingUVDAR, this);

    } else {
      ROS_ERROR("[SensorNeighbor]: The sensor %s is not supported. Shutting down.", _sensor_type_.c_str());
      ros::shutdown();
    }

    /* subscribe to this UAV Odometry */
    sub_this_uav_odom_ = nh.subscribe<nav_msgs::Odometry>("/" + _this_uav_name_ + "/odometry/odom_main", 1, &SensorNeighbor::callbackThisUAVOdom, this);

    /* publisher */
    neigbor_pub_ = nh.advertise<flocking::Neighbors>("/" + _this_uav_name_ + "/sensor_neighbor/neighbors", 1);

    /* timers */
    timer_pub_neighbors_ = nh.createTimer(ros::Rate(5.0), &SensorNeighbor::callbackTimerPubNeighbors, this);

    ROS_INFO_ONCE("[SensorNeighbor]: initialized");
    is_initialized_ = true;

    ros::spin();
  }

  // | ---------------------- subscriber callbacks ------------------------- |

  /* callbackThisUAVOdom() //{ */

  void SensorNeighbor::callbackThisUAVOdom(const nav_msgs::Odometry::ConstPtr& odom) {
    if (!is_initialized_) {
      return;
    }

    {
      std::scoped_lock lock(mutex_this_uav_pose_);
      
      /* update position */
      this_uav_pose_.position.x = odom->pose.pose.position.x;
      this_uav_pose_.position.y = odom->pose.pose.position.y;
      this_uav_pose_.heading    = mrs_lib::AttitudeConverter(odom->pose.pose.orientation).getHeading();
      this_uav_pose_.header     = odom->header;
      
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
      flocking::Position2DStamped uav_pos2d;

      /* fill in msg */
      uav_pos2d.position.x = odom->pose.pose.position.x;
      uav_pos2d.position.y = odom->pose.pose.position.y;
      uav_pos2d.header     = odom->header;

      /* save last estimated position */
      if (neighbors_position_.find(uav_id) == neighbors_position_.end()) {
        neighbors_position_.insert(std::pair<unsigned, flocking::Position2DStamped>(uav_id, uav_pos2d));
      }else {
        neighbors_position_[uav_id] = uav_pos2d;
      }
    }
  }
  
  //}

  /* callbackNeighborsUsingUVDAR //{ */

  void SensorNeighbor::callbackNeighborsUsingUVDAR(const mrs_msgs::PoseWithCovarianceArrayStamped::ConstPtr& array_poses){
    if (!is_initialized_) {
      return;
    }

    {
      std::scoped_lock lock(mutex_neighbors_position_);

      for (unsigned int i = 0; i < array_poses->poses.size(); i++) {
        /* create new msg */
        flocking::Position2DStamped uav_pos2d;

        /* fill in msg */
        uav_pos2d.header = array_poses->header;
        uav_pos2d.position.x = array_poses->poses[i].pose.position.x;
        uav_pos2d.position.y = array_poses->poses[i].pose.position.y;

        /* save last estimated position */
        unsigned int uav_id = array_poses->poses[i].id;
        if (neighbors_position_.find(uav_id) == neighbors_position_.end()) {
          neighbors_position_.insert(std::pair<unsigned int, flocking::Position2DStamped>(uav_id, uav_pos2d));
        }else {
          neighbors_position_[uav_id] = uav_pos2d;
        }
      }
    }
  }

  // | --------------------------- timer callbacks ----------------------------- |

  /* callbackTimerPubNeighbors() //{ */

  void SensorNeighbor::callbackTimerPubNeighbors([[maybe_unused]] const ros::TimerEvent& event) {
    if (!is_initialized_ || !has_this_pose_) {
      return;
    }

    /* create new neigbors message */
    flocking::Neighbors neighbor_info;
    ros::Time now = ros::Time::now();

    if (_sensor_type_ == "gps") {
      /* get pose of this UAV */
      double focal_x, focal_y, focal_heading;
      
      {
        std::scoped_lock lock(mutex_this_uav_pose_);
        focal_x       = this_uav_pose_.position.x;
        focal_y       = this_uav_pose_.position.y;
        focal_heading = this_uav_pose_.heading;
      }

      {
        std::scoped_lock lock(mutex_neighbors_position_);

        for (auto itr = neighbors_position_.begin(); itr != neighbors_position_.end(); ++itr) {
          if ((now - itr->second.header.stamp).toSec() < 1.0) {
            double range   = sqrt(pow(focal_x - itr->second.position.x, 2) + pow(focal_y - itr->second.position.y, 2));
            double bearing = math_utils::relativeBearing(focal_x, focal_y, focal_heading, itr->second.position.x, itr->second.position.y);
          
            neighbor_info.range.push_back(range);
            neighbor_info.bearing.push_back(bearing);
          }
        }
      }
    } else if (_sensor_type_ == "uvdar") {
      double focal_heading;

      {
        std::scoped_lock lock(mutex_this_uav_pose_);
        focal_heading = this_uav_pose_.heading;
      }

      {
        std::scoped_lock lock(mutex_neighbors_position_);
        for (auto itr = neighbors_position_.begin(); itr != neighbors_position_.end(); ++itr) {
          if ((now - itr->second.header.stamp).toSec() < 1.0) {
            double range = sqrt(pow(itr->second.position.x, 2) + pow(itr->second.position.y, 2));
            double bearing = math_utils::relativeBearing(0.0, 0.0, focal_heading, itr->second.position.x, itr->second.position.y);

            neighbor_info.range.push_back(range);
            neighbor_info.bearing.push_back(bearing);
          }
        }
      }
    }

    neighbor_info.header.frame_id = _this_uav_name_ + "/local_origin";
    neighbor_info.header.stamp    = now;
    neighbor_info.num_neighbors   = neighbor_info.range.size();
    
    neigbor_pub_.publish(neighbor_info);
  }

  //}

}  // namespace sensor_neighbor

PLUGINLIB_EXPORT_CLASS(sensor_neighbor::SensorNeighbor, nodelet::Nodelet);