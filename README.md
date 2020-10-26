# Flocking behavior package

Flocking behavior package is a ROS package that implements a flocking control for drones based on the article "Ferrante, E., Turgut, A. E., Huepe, C., Stranieri, A., Pinciroli, C., & Dorigo, M. (2012). Self-organized flocking with a mobile robot swarm: a novel motion control method. Adaptive Behavior, 20(6), 460-477".

## Functionality

The package contains three nodes:

1. **Sensor Neighbor**: this node simulates the functionality of a sensor that measures the relative range and bearing of the neighbors of a UAV using the odometry information. All the parameters used in this node is in the `config/uav_names.yaml` file. 

2. **Formation**: this node uses the relative range and bearing of the neighbors to transform this information into the position where the UAV should go to reach the flocking behavior. All the parameters used in this node is in the `config/flocking/default.yaml` and `config/uav_names.yaml` files.

3. **CommandSender**: this node requests related services from different UAVs at the same time. All the parameters used in this node is in the `config/uav_names.yaml` and `config/commands.yaml` files.

[Nodes communication](https://pastebin.com/raw/YLJutKER)

## Usage

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
