# Flocking behavior package

Flocking behavior package is a ROS package that implements a flocking control for drones based on the article "Ferrante, E., Turgut, A. E., Huepe, C., Stranieri, A., Pinciroli, C., & Dorigo, M. (2012). Self-organized flocking with a mobile robot swarm: a novel motion control method. Adaptive Behavior, 20(6), 460-477".

## Functionality
* Node `CommandSender` can be used to call the same service from different UAVs at the same time
* Service `/($UAV_NAME)/formation/start_hover_mode` makes the UAV to start send `SpeedTrackerCommand` messages
* Service `/($UAV_NAME)/formation/start_swarming_mode` changes the tracker to `SpeedTracker` and turns on the flocking behavior. If there is a leader, this UAV will moving to the specified points using the node `WaypointFlier` (see [`config/WaypointFlier/default.yaml`](https://pastebin.com/raw/Cv9XSyhr)).
* Service `/($UAV_NAME)/formation/close_node` closes the node `Formation` instantly

## Nodes

The package contains three nodes:

1. **Sensor Neighbor**: This node simulates the behavior of the sensor that measures the relative range and bearing of the neighbors of a UAV using the odometry information. All the parameters used in ths node can be found in `config/uav_names.yaml`. 

2. **Formation**: this node uses the relative range and bearing of the neighbors to transform this information into a linear and angular speed. All the parameters used in this node can be found in `config/flocking/default.yaml` and `config/uav_names.yaml`.

3. **CommandSender**: this node can be used to send the commands to all the UAVs at the same time. All the parameters used in this node can be found in `config/uav_names.yaml`.

[Nodes communication](https://drive.google.com/file/d/1Rje89SbCBMtI4yKTzetVEjhZZE36yv41/view?usp=sharing)

## Usage

1. Start one of the simulations.
```
cd simulation/($SIMULATION_FOLDER) && ./start.sh
```

2. After the takeoff, send the command to the UAVs to start send `SpeedTrackerCommand` messages before changing the tracker. For this, you need to call the service from the node `CommandSender` using the request value equals to `0`. The UAV names should be the same as in the uav_names.yaml (see `config/uav_names.yaml`) and make sure every UAV is sending messages to the SpeedTracker before proceed to the next step.
```
rosservice call /command_sender/send_command "value: 0"
``` 

3. Send the command to the UAVs to change the tracker to SpeedTracker and start moving. For this, you need to call the service from the node `CommandSender` using the request value equals to `1`.
```
rosservice call /command_sender/send_command "value: 1"
``` 

5. The flocking behavior will stop manually after the duration time of the simulation been reached (see `config/flocking/default.yaml`). After this, all UAVs will be request to land.

4. (*Optional*) You can stop the flocking behavior anytime calling the service from the node `CommandSender` using the request value equals to `2`. After this, the node `Formation` will close and all UAVs will be request to land. 
```
rosservice call /command_sender/send_command "value: 2"
```

## Videos

* Flocking behavior with 3 drones - [Youtube](https://youtu.be/9aexTp-uvD8)
* Flocking behavior with 3 drones (one leader) - [YouTube](https://youtu.be/selyilRWQWc)