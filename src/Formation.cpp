#include <Formation.h>
#include <pluginlib/class_list_macros.h>

namespace formation
{

/* onInit() //{ */
void Formation::onInit() {
  /* set flags to false */
  is_initialized_ = false;
  hover_mode_     = false;
  swarming_mode_  = false;
  state_machine_running_ = false;

  ros::NodeHandle nh("~");

  ros::Time::waitForValid();

  mrs_lib::ParamLoader param_loader(nh, "Formation");

  /* load parameters */
  param_loader.loadParam("uav_name", _uav_name_);
  param_loader.loadParam("frame", _frame_);
  param_loader.loadParam("desired_height", _desired_height_);
  param_loader.loadParam("land_at_the_end", _land_end_);

  param_loader.loadParam("flocking/auto_start", _auto_start_);
  param_loader.loadParam("flocking/swarming_after_hover", _timeout_state_change_);
  param_loader.loadParam("flocking/duration", _timeout_flocking_);

  /* load proximal control parameters */
  param_loader.loadParam("flocking/proximal/noise", _noise_);
  param_loader.loadParam("flocking/proximal/desired_distance", _desired_distance_);
  param_loader.loadParam("flocking/proximal/strength_potential", _strength_potential_);

  steepness_potential_ = log(2) / log(_desired_distance_ / _noise_);
  max_range_           = 1.8 * _desired_distance_;

  /* load motion control parameters */
  param_loader.loadParam("flocking/motion/K1", _K1_);
  param_loader.loadParam("flocking/motion/K2", _K2_);
  param_loader.loadParam("flocking/motion/move_forward", _move_forward_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[Formation]: failed to load non-optional parameters!");
    ros::shutdown();
  }

  /* message filters */
  sub_neighbors_info_.subscribe(nh, "/" + _uav_name_ + "/sensor_neighbor/neighbors", 1);
  sub_odom_.subscribe(nh, "/" + _uav_name_ + "/odometry/odom_main", 1);
  sync_.reset(new Sync(FormationPolicy(10), sub_neighbors_info_, sub_odom_));
  sync_->registerCallback(boost::bind(&Formation::callbackUAVNeighbors, this, _1, _2));

  /* service client */
  srv_client_goto_ = nh.serviceClient<mrs_msgs::ReferenceStampedSrv>("/" + _uav_name_ + "/control_manager/reference");
  srv_client_land_ = nh.serviceClient<std_srvs::Trigger>("/" + _uav_name_ + "/uav_manager/land");

  /* service servers */
  srv_server_state_machine_ = nh.advertiseService("start_state_machine_in", &Formation::callbackStartStateMachine, this);
  srv_server_hover_mode_    = nh.advertiseService("start_hover_mode", &Formation::callbackStartHoverMode, this);
  srv_server_swarming_mode_ = nh.advertiseService("start_swarming_mode", &Formation::callbackStartSwarmingMode, this);
  srv_server_close_node_    = nh.advertiseService("close_node", &Formation::callbackCloseNode, this);

  /* timers */
  timer_state_machine_ = nh.createTimer(ros::Duration(_timeout_state_change_), &Formation::callbackTimerStateMachine, this, true, false);
  timer_flocking_end_  = nh.createTimer(ros::Duration(_timeout_flocking_), &Formation::callbackTimerAbortFlocking, this, false, false);

  ROS_INFO_ONCE("[Formation]: initialized");
  is_initialized_ = true;

  ros::spin();
}

//}

// | ----------------------- message filters callbacks ----------------------- |

/* callbackUAVNeighbors() //{ */

void Formation::callbackUAVNeighbors(const flocking::Neighbors::ConstPtr& neighbors, const nav_msgs::Odometry::ConstPtr& odom) {
  /* return if is not initialized or the code is not on swarming mode */
  if (!is_initialized_ || !swarming_mode_)
    return;

  if (neighbors->num_neighbors > 0) {
    double prox_vector_x = 0.0, prox_vector_y = 0.0, prox_magnitude;
    for (unsigned int i = 0; i < neighbors->num_neighbors; i++) {
      if (neighbors->range[i] <= max_range_) {
        /* compute proximal control vector */
        prox_magnitude = Formation::getProximalMagnitude(neighbors->range[i]);
        prox_vector_x += prox_magnitude * cos(neighbors->bearing[i]);
        prox_vector_y += prox_magnitude * sin(neighbors->bearing[i]);
      }
    }

    /* convert flocking control (f = p) vector to angular and linear movement */
    double w = prox_vector_y * _K2_;
    double u = prox_vector_x * _K1_ + _move_forward_;

    /* create reference stamped msg */
    mrs_msgs::ReferenceStampedSrv srv_reference_stamped_msg;

    /* fill in header */
    srv_reference_stamped_msg.request.header.stamp = ros::Time::now();
    srv_reference_stamped_msg.request.header.frame_id = _uav_name_ + "/" + _frame_;  


    /* fill in reference */
    double heading = mrs_lib::AttitudeConverter(odom->pose.pose.orientation).getHeading();
    srv_reference_stamped_msg.request.reference.position.x = odom->pose.pose.position.x + cos(heading) * u;
    srv_reference_stamped_msg.request.reference.position.y = odom->pose.pose.position.y + sin(heading) * u;
    srv_reference_stamped_msg.request.reference.position.z = odom->pose.pose.position.z;
    srv_reference_stamped_msg.request.reference.heading    = heading + w;

    /* request service */ 
    if (srv_client_goto_.call(srv_reference_stamped_msg)){ 
      
    }else{
      ROS_ERROR("Failed to call service.\n");
    }

  }
}

//}

// | --------------------------- timer callbacks ----------------------------- |

/* callbackTimerStateMachine() //{ */

void Formation::callbackTimerStateMachine([[maybe_unused]] const ros::TimerEvent& event) {
  if (!is_initialized_ || !state_machine_running_) {
    return;
  }

  ros::Time now = ros::Time::now();
  if (hover_mode_) {
    /* turn on swarming mode */
    state_change_time_ = now;
    swarming_mode_     = true;

    /* start flocking countdown */
    timer_flocking_end_.start();
  }

  return;
}

//}

/* callbackTimerAbortFlocking() //{ */

void Formation::callbackTimerAbortFlocking([[maybe_unused]] const ros::TimerEvent& event) {
  /* turn off swarming mode (switch back to hover mode) */
  swarming_mode_ = false;

  /* request land service */
  if (_land_end_) {
    ROS_INFO("[Formation]: Calling land service");
    std_srvs::Trigger srv_land_call;
    srv_client_land_.call(srv_land_call);
  }

  ROS_INFO_ONCE("[Formation]: The time is over. Shutting down");

  /* reset message filter */
  sync_.reset();

  /* shutdown node */
  ros::shutdown();

  return;
}

//}

// | ----------------------- service server callbacks ----------------------- |

/* callbackStartStateMachine() //{ */

bool Formation::callbackStartStateMachine([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  if(!_auto_start_){
    ROS_WARN("[Formation]: The automatic start is not on. The hover and swarming mode should start manually.");
    res.message = "[Formation]: The automatic start is not on. The hover and swarming mode should start manually.";
    res.success = false;
    return false;
  } else if (!is_initialized_) {
    ROS_WARN("[Formation]: Cannot start state machine, nodelet is not initialized.");
    res.message = "Cannot change to hover mode, nodelet is not initialized.";
    res.success = false;
    return false;
  } else if (hover_mode_) {
    ROS_WARN("[Formation]: Cannot start state machine, already in hover mode.");
    res.message = "Cannot start state machine, already in hover mode.";
    res.success = false;
    return false;
  } else if (swarming_mode_) {
    ROS_WARN("[Formation]: Cannot start state machine, already in swarming mode.");
    res.message = "Cannot start state machine, already in swarming mode.";
    res.success = false;
    return false;
  }

  /* change code to hover mode */
  hover_mode_            = true;
  state_change_time_     = ros::Time::now();
  state_machine_running_ = true;

  timer_state_machine_.start();

  ROS_INFO("[Formation]: Changed to hover mode. Swarming mode will be activated in %0.2f seconds.", _timeout_state_change_);
  res.message = "Changed to hover mode. Swarming mode will be activated in specified timeout.";
  res.success = true;

  return true;
}

//}

/* callbackStartHoverMode() //{ */

bool Formation::callbackStartHoverMode([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  if (!is_initialized_) {
    ROS_WARN("[Formation]: Cannot change to hover mode, nodelet is not initialized");
    res.message = "Cannot change to hover mode, nodelet is not initialized";
    res.success = false;
    return false;
  } else if (state_machine_running_) {
    ROS_WARN("[Formation]: Cannot change to hover mode. The states are handled by a state machine.");
    res.message = "Cannot change to hover mode. The states are handled by a state machine.";
    res.success = false;
    return false;
  }

  /* change code to hover mode */
  hover_mode_ = true;

  ROS_INFO("[Formation]: Changed to hover mode. It is safe now to start the swarming mode");
  res.message = "Changed to hover mode. It is safe now to start the swarming mode";
  res.success = true;

  return true;
}

//}

/* callbackStartSwarmingMode() //{ */

bool Formation::callbackStartSwarmingMode([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  if (!is_initialized_) {
    ROS_WARN("[Formation]: cannot change to swarming mode, nodelet is not initialized");
    res.message = "Cannot change to swarming mode, nodelet is not initialized";
    res.success = false;
    return false;
  } else if (state_machine_running_) {
    ROS_WARN("[Formation]: Cannot change to swarming mode. The states are handled by a state machine.");
    res.message = "Cannot change to swarming mode. The states are handled by a state machine.";
    res.success = false;
    return false;
  } else if (!hover_mode_) {
    ROS_WARN("[Formation]: Cannot change to swarming mode, hover mode has not been started");
    res.message = "Cannot change to swarming mode, hover mode has not been started";
    res.success = false;
    return false;
  }

 
  ROS_INFO("[Formation]: Starting the swarming mode");
  res.message = "Starting the swarming mode";
  res.success = true;
  
  /* change code to swarming mode */
  swarming_mode_ = true;

  /* start flocking countdown */
  timer_flocking_end_.start();

  ROS_INFO("[Formation]: The flocking behavior has started and will last %0.2f seconds", _timeout_flocking_);

  return true;
}

//}

/* callbackCloseNode() //{ */

bool Formation::callbackCloseNode([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  if (!is_initialized_) {
    ROS_WARN("[Formation]: Cannot stop formation, nodelet is not initialized");
    res.message = "Cannot stop formation, nodelet is not initialized";
    res.success = false;
    return false;
  } else if (state_machine_running_) {
    ROS_WARN("[Formation]: Cannot close node. The states are handled by a state machine.");
    res.message = "Cannot close node. The states are handled by a state machine.";
    res.success = false;
    return false;
  }

  /* turn off swarming mode (switch back to hover mode) */
  swarming_mode_ = false;

  /* request land service */
  if (_land_end_) {
    ROS_INFO("[Formation]: Calling land service");
    std_srvs::Trigger srv_land_call;
    srv_client_land_.call(srv_land_call);
  }

  ROS_INFO("[Formation]: Closing node before the time. Shutting down");
  res.message = "Closing node before the time. Shutting down";
  res.success = true;

  /* reset message filter */
  sync_.reset();
  
  /* shutdown node */
  ros::shutdown();

  return true;
}

//}

// | -------------------------- support functions -------------------------- |

/* getProximalMagnitude() //{ */

double Formation::getProximalMagnitude(double range) {
  return -4 * steepness_potential_ * _strength_potential_ / range *
         (2 * pow(_noise_ / range, 2 * steepness_potential_) - pow(_noise_ / range, steepness_potential_));
}

//}

}  // namespace formation

PLUGINLIB_EXPORT_CLASS(formation::Formation, nodelet::Nodelet);
