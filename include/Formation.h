#pragma once
#ifndef FORMATION_H
#define FORMATION_H

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>
#include <mutex>

#include <mrs_lib/param_loader.h>

#include <mrs_msgs/String.h>
#include <mrs_msgs/SpeedTrackerCommand.h>

#include <std_msgs/Header.h>

#include <std_srvs/Trigger.h>

/* custom msg */
#include <flocking/Neighbors.h>

namespace formation
{

class Formation : public nodelet::Nodelet{
public:
	virtual void onInit();

private:
  /* flags */
	bool is_initialized_;
	bool hover_mode_;
	bool swarming_mode_;
	
	std::string _uav_name_;

	// | ---------------------- proximal control parameters ---------------------- |

	double _desired_distance_;
	double _noise_;
	double _strength_potential_;
	double max_range_;
	double steepness_potential_;

	// | --------- magnitude-dependent motion control (MDMC) parameters ---------- |

	double _K1_; // linear speed gain
	double _K2_; // angular speed gain
	double _move_forward_;
	double u_, w_; // linear and angular speed

	// | ------------------------- subscriber callbacks -------------------------- |
	
	void callbackUAVNeighbors(const flocking::Neighbors::ConstPtr& neighbors);
	ros::Subscriber sub_neighbors_info_;

	// | --------------------------- timer callbacks ----------------------------- |
	
	/* after start the swarming mode, the node will run for ($_duration_timer_flocking_) seconds */
	void callbackTimerFlocking(const ros::TimerEvent& event);
	ros::Timer timer_flocking_end_;
	int _duration_timer_flocking_;

	/* publish speed tracker commands with a rate of ($_rate_timer_publisher_speed) Hz */
	void callbackTimerPublishSpeed(const ros::TimerEvent& event);
	bool has_speed_command_;
	ros::Timer timer_publisher_speed_;
	ros::Publisher pub_speed_;
	std::mutex mutex_speed_;
	int _rate_timer_publisher_speed_;
	std::string _frame_;
	double _desired_height_;
	
	// | ------------------------ service clients callbacks ---------------------- |
	
	ros::ServiceClient srv_client_switcher_;

	ros::ServiceClient srv_client_land_;
	bool _land_end_;

	// | ------------------------ service server callbacks ----------------------- |

	/* switch code from "no" mode to hover mode */
	bool callbackStartHoverMode(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
	ros::ServiceServer srv_server_hover_mode_;

	/* switch code from hover mode to swarming mode */
	bool callbackStartSwarmingMode(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
	ros::ServiceServer srv_server_swarming_mode_;
	
	/* shutdown node - use in case of emergency */
	bool callbackCloseNode(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
	ros::ServiceServer srv_server_close_node_;

	// | -------------------------- support functions ---------------------------- |
	
	double getProximalMagnitude(double range);
}; 

} // namespace formation
#endif