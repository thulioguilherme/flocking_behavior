# Flocking behavior package

Flocking behavior package is a ROS package that implements a flocking control for drones based on the article "Ferrante, E., Turgut, A. E., Huepe, C., Stranieri, A., Pinciroli, C., & Dorigo, M. (2012). Self-organized flocking with a mobile robot swarm: a novel motion control method. Adaptive Behavior, 20(6), 460-477".

## Functionality
* Node `send_command` can be used to call the same service from different UAVs at the same time
* Service `/($UAV_NAME)/formation/start_hover_mode` makes the UAV to start send `SpeedTrackerCommand` messages
* Service `/($UAV_NAME)/formation/start_swarming_mode` changes the tracker to `SpeedTracker` and turns on the flocking behavior
* Service `/($UAV_NAME)/formation/close_node` closes the node `Formation` instantly
* Service `/sensor_neighbor/start_experiment` makes the node `Sensor Neighbor` starts to record the important data on a rosbag file
* Service `/sensor_neighbor/close_node` closes the node `Sensor Neighbor` instantly

## Nodes

The package contains two nodes:

1. **Sensor Neighbor**: this node simulates the behavior of the sensor that measures the relative range and bearing of the neighbors of a UAV using the odometry information. All the parameters used in ths node can be found in `config/experiments.yaml` and `config/uav_names.yaml`. This node is also responsible to create a rosbag file to save the important data.

2. **Formation**: this node uses the relative range and bearing of the neighbors to transform this information into a linear and angular speed. All the parameters used in this node can be found in `config/flocking/default.yaml`.

### Nodes communication

The table below describe the subscribers, publishers, service servers and clients used in both nodes.

|                 |                                                                   **Sensor Neighbor**                                                                 |                                                        **Formation**                                                       |
|:---------------:|:--------------------------------------------------------------------------------------------------------------------------------------------------:|:----------------------------------------------------------------------------------------------------------------------:|
|  **Subscribers**   |                 /($UAV1)/odometry/odom_main 				 /($UAV2)/odometry/odom_main 				 /($UAV3)/odometry/odom_main 				 /($UAV4)/odometry/odom_main                 |                                         /sensor_neighbors/($UAV_NAME)/neighbors                                        |
|    **Publishers**   | /sensor_neighbors/($UAV1)/neighbors 				 /sensor_neighbors/($UAV2)/neighbors 				 /sensor_neighbors/($UAV3)/neighbors 				 /sensor_neighbors/($UAV4)/neighbors |                                   /($UAV_NAME)/control_manager/speed_tracker/command                                   |
| **Service servers** |                                           /sensor_neighbor/start_experiment 				 /sensor_neighbor/close_node                                           | /($UAV_NAME)/formation/start_hover_mode 				 /($UAV_NAME)/formation/start_swarming_mode 				 /($UAV_NAME)/formation/close_node |
| **Service clients** |                       *None*                                                                                                                         |                       /($UAV_NAME)/control_manager/switch_tracker 				 /($UAV_NAME)/uav_manager/land                       |

## Usage

### Flocking behavior with 4 drones ([Youtube](https://www.youtube.com/watch?v=GyrmELmuqa8))

1. Start the simulation.
```
cd flocking/src/simulation/four_drones_flocking && ./start.sh
```

2. After the takeoff, send the command to the UAVs to start send `SpeedTrackerCommand` messages before changing the tracker. You can do it running the node `send_command` using the argument `start_hover_mode`. The UAV names should be the same as in the uav_names.yaml (see config/uav_names.yaml) and make sure every UAV is sending messages to the SpeedTracker before proceed to the next step.
```
rosrun flocking send_command start_hover_mode ($UAV_1) ($UAV_2) ($UAV_3) ($UAV_4)
``` 

3. Send the command to the UAVs to change the tracker to SpeedTracker and start moving. You can do it running the node `send_command` using the argument `start_swarming_mode`. This command will also call the service `/sensor_neighbor/start_experiment`.
```
rosrun flocking send_command start_swarming_mode ($UAV_1) ($UAV_2) ($UAV_3) ($UAV_4)
``` 

5. The experiment will stop both nodes manually after the duration time of the simulation been reached (see `config/experiment.yaml`). After the end of the experiment, all UAVs will be request to land.

4. (*Optional*) You can stop both nodes anytime using the argument `close_node`. After this, both nodes will close and all UAVs will be request to land. 
```
rosrun flocking send_command close_node ($UAV_1) ($UAV_2) ($UAV_3) ($UAV_4)
```