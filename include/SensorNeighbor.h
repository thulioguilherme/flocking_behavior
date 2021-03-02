#pragma once
#ifndef SENSOR_NEIGHBOR_H
#define SENSOR_NEIGHBOR_H

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/transformer.h>

#include <mrs_msgs/PoseWithCovarianceArrayStamped.h>
#include <mrs_msgs/Float64Stamped.h>

#include <nav_msgs/Odometry.h>

#include <geometry_msgs/PointStamped.h>

#include <math.h>

#include <map>
#include <mutex>

/* custom msgs */
#include <flocking/Neighbors.h>
#include <flocking/Point2DStamped.h>

/* custom library */
#include <MathUtils.h>

using test_t = geometry_msgs::PointStamped;
template std::optional<test_t> mrs_lib::Transformer::transform<test_t>(const mrs_lib::TransformStamped& to_frame, const test_t& what);

namespace sensor_neighbor
{

class SensorNeighbor : public nodelet::Nodelet {
public:
  virtual void onInit();

private:
  /* flags */
  bool is_initialized_;
  bool _use_3D_;

  std::string              _sensor_type_;
  std::string              _this_uav_name_;
  std::vector<std::string> _uav_names_;

  mrs_lib::Transformer                     tfr_;
  std::optional<mrs_lib::TransformStamped> tf_output_;

  // | ------------------------ subscriber callbacks --------------------------- |

  void                       callbackThisUAVOdom(const nav_msgs::Odometry::ConstPtr& odom);
  ros::Subscriber            sub_this_uav_odom_;
  geometry_msgs::PoseStamped this_uav_pose_;
  std::mutex                 mutex_this_uav_pose_;
  bool                       has_this_pose_;

  void                     callbackThisUAVLocalOdom(const nav_msgs::Odometry::ConstPtr& odom);
  ros::Subscriber          sub_this_uav_local_odom_;
  mrs_msgs::Float64Stamped this_uav_local_height_;
  bool                     has_this_uav_local_height_;

  /* GPS */
  void                                             callbackNeighborsUsingGPSOdom(const nav_msgs::Odometry::ConstPtr& odom, const unsigned int uav_id);
  std::vector<ros::Subscriber>                     sub_odom_uavs_;
  std::map<unsigned int, flocking::Point2DStamped> neighbors_position_2d_;
  std::mutex                                       mutex_neighbors_position_2d_;

  void                                             callbackNeighborsUsingGPSOdomLocal(const nav_msgs::Odometry::ConstPtr& odom, const unsigned int uav_id);
  std::vector<ros::Subscriber>                     sub_odom_local_uavs_;
  std::map<unsigned int, mrs_msgs::Float64Stamped> neighbors_height_;
  std::mutex                                       mutex_neighbors_height_;

  /* UVDAR */
  void                                                callbackNeighborsUsingUVDAR(const mrs_msgs::PoseWithCovarianceArrayStamped::ConstPtr& array_pose);
  ros::Subscriber                                     sub_uvdar_filtered_poses_;
  std::map<unsigned int, geometry_msgs::PointStamped> neighbors_position_;
  std::mutex                                          mutex_neighbors_position_;

  // | --------------------------- timer callbacks ----------------------------- |

  void           callbackTimerPubNeighbors(const ros::TimerEvent& event);
  ros::Timer     timer_pub_neighbors_;
  ros::Publisher pub_neighbors_;
};

}  // namespace sensor_neighbor

#endif
