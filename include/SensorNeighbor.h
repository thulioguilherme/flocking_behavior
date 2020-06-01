#pragma once
#ifndef SENSOR_NEIGHBOR_H
#define SENSOR_NEIGHBOR_H

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

#include <mutex>

#include <mrs_lib/param_loader.h>

#include <MathOperations.h>

#include <nav_msgs/Odometry.h>

#include <map>
#include <std_srvs/Trigger.h>

#include <mrs_lib/attitude_converter.h>

/* custom msg */
#include <flocking/Neighbors.h>

namespace sensor_neighbor
{

class SensorNeighbor : public nodelet::Nodelet {
public:
  virtual void onInit();

private:
  /* flags */
  bool is_initialized_;
  bool has_odom_this_ = false;

  unsigned int             num_other_uavs_;
  unsigned int             this_uav_name_idx_;
  std::string              _this_uav_name_;
  std::vector<std::string> _uav_names_;
  std::mutex  mutex_odoms_;
  std::map<std::string, nav_msgs::Odometry> odoms_;

  // | ------------------------ subscriber callbacks --------------------------- |

  void callbackUAVOdom(const nav_msgs::Odometry::ConstPtr& odom, const std::string uav_name);
  ros::Subscriber sub_odom_uav1_;
  ros::Subscriber sub_odom_uav2_;
  ros::Subscriber sub_odom_uav3_;

  std::vector<ros::Subscriber> sub_odom_uavs_;

  // | --------------------------- timer callbacks ----------------------------- |
  
  void       callbackTimerPubNeighbors(const ros::TimerEvent& event);
  ros::Timer timer_pub_neighbors_;
  ros::Publisher  neigbor_pub_;

};

}  // namespace sensor_neighbor

#endif
