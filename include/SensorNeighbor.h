#pragma once
#ifndef SENSOR_NEIGHBOR_H
#define SENSOR_NEIGHBOR_H

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>
#include <rosbag/bag.h>
#include <mutex>

#include <time.h>

#include <mrs_lib/param_loader.h>

#include <MathOperations.h>

#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>


#include <std_srvs/Trigger.h>

#include <tf/transform_datatypes.h>

/* custom msg */
#include <flocking/Neighbors.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace message_filters;

namespace sensor_neighbor
{
	
class SensorNeighbor : public nodelet::Nodelet{
public:
	virtual void onInit();

private:
	/* flags */
	bool is_initialized_;
	bool is_bag_open_;

	// | ---------------------- message filter callbacks ------------------------- |

  	typedef sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry, nav_msgs::Odometry, nav_msgs::Odometry> OdomPolicy;
	typedef Synchronizer<OdomPolicy> Sync;
	boost::shared_ptr<Sync> sync_;
	message_filters::Subscriber<nav_msgs::Odometry> sub_odom_uav1_;
	message_filters::Subscriber<nav_msgs::Odometry> sub_odom_uav2_;
	message_filters::Subscriber<nav_msgs::Odometry> sub_odom_uav3_;
	message_filters::Subscriber<nav_msgs::Odometry> sub_odom_uav4_;

	void callbackUAVsPose(const nav_msgs::Odometry::ConstPtr& odom_uav1,
		const nav_msgs::Odometry::ConstPtr& odom_uav2,
		const nav_msgs::Odometry::ConstPtr& odom_uav3,
		const nav_msgs::Odometry::ConstPtr& odom_uav4);

	std::vector<ros::Publisher> neigbor_pub_;
	std::vector<std::string> _uav_names_;
	uint num_uavs_;

	// | --------------------------- timer callbacks ----------------------------- |

	/* after start the swarming mode, the node will run for ($_duration_timer_experiment_) seconds */
	void callbackTimerExperiment(const ros::TimerEvent& event);
	ros::Timer timer_experiment_;
	int _duration_timer_experiment_;

	/* after start the swarming mode, the node will start to save the data on a rosbag */
	void callbackTimerBagWriter(const ros::TimerEvent& event);
	rosbag::Bag bag_;
	ros::Timer timer_bag_writer_;
	std::mutex mutex_pose_;
	time_t rawtime_;
  	struct tm * timeinfo_;
  	char buffer_ [80];
  	std::string bag_name_;
  	int _timer_bag_step_;

  	/* variables to write the data on a rosbag file */
	std_msgs::Float32 uav1_x_, uav1_y_, uav1_yaw_;
	std_msgs::Float32 uav2_x_, uav2_y_, uav2_yaw_;
	std_msgs::Float32 uav3_x_, uav3_y_, uav3_yaw_;
	std_msgs::Float32 uav4_x_, uav4_y_, uav4_yaw_;

	// | ----------------------- service server callbacks ------------------------ |

	/* start to write on the rosbag file */
	bool callbackStartExperiment(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
	ros::ServiceServer srv_server_start_experiment_;

	/* shutdown node - use in case of emergency */
	bool callbackCloseNode(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
	ros::ServiceServer srv_server_close_node_;

};

}

#endif