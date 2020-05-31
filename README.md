# Flocking behavior package

Flocking behavior package is a ROS package that implements a flocking control for drones based on the article "Ferrante, E., Turgut, A. E., Huepe, C., Stranieri, A., Pinciroli, C., & Dorigo, M. (2012). Self-organized flocking with a mobile robot swarm: a novel motion control method. Adaptive Behavior, 20(6), 460-477".

## Functionality
* Node `send_command` can be used to call the same service from different UAVs at the same time
* Service `/($UAV_NAME)/formation/start_hover_mode` makes the UAV to start send `SpeedTrackerCommand` messages
* Service `/($UAV_NAME)/formation/start_swarming_mode` changes the tracker to `SpeedTracker` and turns on the flocking behavior
* Service `/($UAV_NAME)/formation/close_node` closes the node `Formation` instantly

## Nodes

The package contains two nodes:

1. **Sensor Neighbor**: this node simulates the behavior of the sensor that measures the relative range and bearing of the neighbors of a UAV using the odometry information. All the parameters used in ths node can be found in `config/uav_names.yaml`. 

2. **Formation**: this node uses the relative range and bearing of the neighbors to transform this information into a linear and angular speed. All the parameters used in this node can be found in `config/flocking/default.yaml` and `config/uav_names.yaml`.

### Nodes communication

The table below describe the subscribers, publishers, service servers and clients used in both nodes.

|  	| **Sensor Neighbor** 	| **Formation** 	|
|:-:	|:-:	|:-:	|
| **Subscribers** 	| /($UAV1)/odometry/slow_odom 				 /($UAV2)/odometry/slow_odom  /($UAV3)/odometry/slow_odom 	| /($UAV_NAME)/sensor_neighbors/neighbors 	|
| **Publishers** 	| /($UAV_NAME)/sensor_neighbors/neighbors 	| /($UAV_NAME)/control_manager/speed_tracker/command 	|
| **Service servers** 	| *None* 	| /($UAV_NAME)/formation/start_hover_mode 		 /($UAV_NAME)/formation/start_swarming_mode 				 /($UAV_NAME)/formation/close_node 	|
| **Service clients** 	| *None* 	| /($UAV_NAME)/control_manager/switch_tracker 				 /($UAV_NAME)/uav_manager/land 	|

## Simulation

### [Flocking behavior with 3 drones](https://youtu.be/9aexTp-uvD8) 

1. Start the simulation.
```
cd flocking/src/simulation/three_drones_flocking && ./start.sh
```

2. After the takeoff, send the command to the UAVs to start send `SpeedTrackerCommand` messages before changing the tracker. You can do it running the node `send_command` using the argument `start_hover_mode`. The UAV names should be the same as in the uav_names.yaml (see `config/uav_names.yaml`) and make sure every UAV is sending messages to the SpeedTracker before proceed to the next step.
```
rosrun flocking send_command start_hover_mode 0 ($UAV_1) ($UAV_2) ($UAV_3)
``` 

3. Send the command to the UAVs to change the tracker to SpeedTracker and start moving. You can do it running the node `send_command` using the argument `start_swarming_mode`. 
```
rosrun flocking send_command start_swarming_mode 0 ($UAV_1) ($UAV_2) ($UAV_3)
``` 

5. The flocking behavior will stop manually after the duration time of the simulation been reached (see `config/flocking/default.yaml`). After this, all UAVs will be request to land.

4. (*Optional*) You can stop the flocking behavior anytime using the argument `close_node`. After this, the node will close and all UAVs will be request to land. 
```
rosrun flocking send_command close_node 0 ($UAV_1) ($UAV_2) ($UAV_3)
```