#pragma once
#ifndef SENSOR_NEIGHBOR_H
#define SENSOR_NEIGHBOR_H

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/attitude_converter.h>

#include <mrs_msgs/PoseWithCovarianceArrayStamped.h>

#include <nav_msgs/Odometry.h>

#include <std_srvs/Trigger.h>

#include <map>
#include <mutex>

/* custom msgs */
#include <flocking/Neighbors.h>
#include <flocking/Position2DStamped.h>
#include <flocking/PoseStamped.h>

/* custom library */
#include <MathUtils.h>

namespace sensor_neighbor
{

class SensorNeighbor : public nodelet::Nodelet {
public:
  virtual void onInit();

private:
  /* flags */
  bool is_initialized_;

  std::string              _sensor_type_;
  std::string              _this_uav_name_;
  std::vector<std::string> _uav_names_;

  // | ------------------------ subscriber callbacks --------------------------- |

  void                  callbackThisUAVOdom(const nav_msgs::Odometry::ConstPtr& odom);
  ros::Subscriber       sub_this_uav_odom_;
  flocking::PoseStamped this_uav_pose_;
  std::mutex            mutex_this_uav_pose_;
  bool                  has_this_pose_;

  std::map<unsigned int, flocking::Position2DStamped> neighbors_position_;
  std::mutex mutex_neighbors_position_;

  /* GPS */
  void callbackNeighborsUsingGPS(const nav_msgs::Odometry::ConstPtr& odom, const unsigned int id);
  std::vector<ros::Subscriber> sub_odom_uavs_;

  /* UVDAR */
  void callbackNeighborsUsingUVDAR(const mrs_msgs::PoseWithCovarianceArrayStamped::ConstPtr& array_pose);
  ros::Subscriber sub_uvdar_filtered_poses_;

  // | --------------------------- timer callbacks ----------------------------- |
  
  void           callbackTimerPubNeighbors(const ros::TimerEvent& event);
  ros::Timer     timer_pub_neighbors_;
  ros::Publisher neigbor_pub_;

};

}  // namespace sensor_neighbor

#endif
