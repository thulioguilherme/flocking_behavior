# Flocking behavior package

The flocking behavior package is a ROS package that implements a proximal control-based method for UAV self-organized flocking based on the work of [Ferrante et al. (2012)](https://journals.sagepub.com/doi/10.1177/1059712312462248).

## Overview

The package contains two central nodes:

* **SensorNeighbor**: this node is responsible for receive and process the data used to obtain the range and bearing of the neighbors and then send it to the next node through publisher-subscriber communication. It can process the data generated by two methods:
	* GPS - all the UAVs trades position's information through publisher-subscriber communication
	* UVDAR - each UAV estimates the relative position of its neighbors using the [UVDAR sensor](https://github.com/ctu-mrs/uvdar_core)

* **Formation**: this node uses the relative range and bearing of the neighbors to transform this information into the position where the UAV should go to achieve or maintain the flocking behavior. It forces the UAV to follow three essential rules to accomplish the desired behavior:
	* Keep a certain distance from its neighbors
	* Keep moving forward
	* Have non-holonomic movement

[Nodes communication](https://pastebin.com/raw/YLJutKER)

## Getting started

### Prerequisites
* [ROS Melodic](http://wiki.ros.org/melodic)
* [Multi-robot Systems Group UAV system](https://github.com/ctu-mrs/mrs_uav_system)
* [UVDAR drivers and processing](https://github.com/ctu-mrs/uvdar_core)
* [UVDAR Gazebo plugin](https://github.com/ctu-mrs/uvdar_gazebo_plugin)

### Installing

Follow the instructions to install our package into your machine and use it for testing in simulations

* Install all dependencies
* Clone the repository into a ROS workspace - `git clone https://github.com/thulioguilherme/flocking_behavior.git`
* Build the package using catkin tools - `catkin build flocking_behavior`

## Testing

The package can be tested using the [Multi-robot Systems Group UAV system](https://github.com/ctu-mrs/mrs_uav_system). There are a few examples already settled in the simulation directory.

### Running a simulation

1. Start one of the simulations from the simulation directory. The flocking behavior will start automatically after a few seconds if the parameter `auto_start` is `true` (see `config/flocking/default.yaml`). You don't do the following steps if this is the case. Only the optional step, if necessary. 
```
cd simulation/($SELECTED_SIMULATION) && ./start.sh
```

2. The following steps are only needed if the flocking behavior is not supposed to start automatically. After the takeoff, send the command to the UAVs to change to hover mode. For this, you need to call the service `send_command` from the node `CommandSender` using the request value equals to `0`.
```
rosservice call /command_sender/send_command "value: 0"
``` 

3. Then send the command to the UAVs to start the flocking behavior. For this, you need to call the service `send_command` from the node `CommandSender` using the request value equals to `1`.
```
rosservice call /command_sender/send_command "value: 1"
``` 

4. The flocking behavior will stop manually after the duration is over.

5. (**Optional**) You can end the flocking behavior anytime calling the service `send_command` from the node `CommandSender` using the request value equals to `2`.
```
rosservice call /command_sender/send_command "value: 2"
```
