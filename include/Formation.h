#pragma once
#ifndef FORMATION_H
#define FORMATION_H

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/geometry_utils.h>

#include <mrs_msgs/ReferenceStampedSrv.h>
#include <mrs_msgs/Float64Stamped.h>

#include <nav_msgs/Odometry.h>

#include <std_srvs/Trigger.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

/* custom msgs */
#include <flocking/Neighbors.h>
#include <flocking/ModeStamped.h>

/* custom library */
#include <MathUtils.h>

using namespace message_filters;

namespace formation
{

class Formation : public nodelet::Nodelet {
public:
  virtual void onInit();

private:
  /* flags */
  bool is_initialized_;
  bool hover_mode_;
  bool swarming_mode_;
  bool _use_3D_;

  /* ros parameters */
  std::string _uav_name_;

  /* publishers */
  ros::Publisher pub_mode_changed_;
  ros::Publisher pub_virtual_heading_;

  // | ---------------------- proximal control parameters ---------------------- |

  double _desired_distance_;
  double _range_multipler_;
  double _steepness_potential_;
  double _strength_potential_;
  
  double max_range_;
  double noise_;
  
  // | ----------------------- motion control parameters ----------------------- |

  double _K1_;  // Linear gain
  double _K2_;  // Angular gain
  double _K3_;
  double _move_forward_;
  double _interpolate_coeff_;
  bool  _fixed_heading_;

  double virtual_heading_;
  double smooth_heading_;
  double initial_heading_;

  // | ----------------------- message filters callbacks ----------------------- |
  
  typedef sync_policies::ApproximateTime<flocking::Neighbors, nav_msgs::Odometry> FormationPolicy;
  typedef Synchronizer<FormationPolicy> Sync;
  boost::shared_ptr<Sync> sync_;
  
  message_filters::Subscriber<flocking::Neighbors> sub_neighbors_info_;
  message_filters::Subscriber<nav_msgs::Odometry>  sub_odom_;

  void        callbackUAVNeighbors(const flocking::Neighbors::ConstPtr& neighbors, const nav_msgs::Odometry::ConstPtr& odom);
  std::string _frame_;
  double      _desired_height_;
  double      _minimum_height_;
  
  // | --------------------------- timer callbacks ----------------------------- |

  /* after start the swarming mode, the node will run for ($_timeout_flocking_) seconds */
  void       callbackTimerAbortFlocking(const ros::TimerEvent& event);
  ros::Timer timer_flocking_end_;
  double     _timeout_flocking_;

  void       callbackTimerStateMachine(const ros::TimerEvent& event);
  ros::Timer timer_state_machine_;
  ros::Time  state_change_time_;
  bool       state_machine_running_;
  double     _timeout_state_change_;

  // | ------------------------ service clients callbacks ---------------------- |

  ros::ServiceClient srv_client_goto_;
  ros::ServiceClient srv_client_land_;
  bool               _land_end_;

  // | ------------------------ service server callbacks ----------------------- |

  /* start state machine (trigger hover now and wait #s timeout for swarm mode) */
  bool               callbackStartStateMachine(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  ros::ServiceServer srv_server_state_machine_;
  bool               _auto_start_;

  /* switch code from "no" mode to hover mode */
  bool               callbackStartHoverMode(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  ros::ServiceServer srv_server_hover_mode_;

  /* switch code from hover mode to swarming mode */
  bool               callbackStartSwarmingMode(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  ros::ServiceServer srv_server_swarming_mode_;

  /* shutdown node - use in case of emergency */
  bool               callbackCloseNode(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  ros::ServiceServer srv_server_close_node_;

  // | -------------------------- support functions ---------------------------- |

  double getProximalMagnitude(double range);
};

}  // namespace formation
#endif

