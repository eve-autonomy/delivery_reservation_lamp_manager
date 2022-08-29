# Lamp manager for delivery reservation

## Overview
By changing lighting pattern of the lamp, this node notifies the following status through `/dio_ros_driver`. 

1. Notify surrounding workers of on-demand delivery reservation status.
1. Notify operator of shutdown status.

Lighting patterns are listed below.

|Reservation status|lighting pattern|
|:-----------------|:---------------|
|Not reserved      |Off             |
|In progress       |Blinks(1Hz)     |
|Reserved          |Light up        |

|Shutdown status|lighting pattern|
|:-----------------|:---------------|
|Standby for shutdown |Blinks(2Hz)  |
|Start of shutdown |Repeat the following;<br>Blinks twice at 0.2 second intervals, and the second light off for 1.5 seconds.|

## Input and Output
- input
  - from [autoware_state_machine](https://github.com/eve-autonomy/autoware_state_machine)
    - `/autoware_state_machine/lock_state` \[[autoware_state_machine_msgs/msg/StateLock](https://github.com/eve-autonomy/autoware_state_machine_msgs/blob/main/msg/StateLock.msg)\]:<br>Reservation status for on-demand delivery.
  - from [shutdown_manager](https://github.com/eve-autonomy/shutdown_manager)
    - `/shutdown_manager/state` \[[shutdown_manager_msgs/msg/StateShutdown](https://github.com/eve-autonomy/shutdown_manager_msgs/blob/main/msg/StateShutdown.msg)\]:<br>Shutdown status.
- output
  - to [dio_ros_driver](https://github.com/tier4/dio_ros_driver)
    - `/dio/dout3` \[[dio_ros_driver/msg/DIOPort](https://github.com/tier4/dio_ros_driver/blob/develop/ros2/msg/DIOPort.msg)\]:<br>Digital-out assignment to a 3rd pin in [0-7] general-purpose outputs. This topic is remapped from `/delivery_reservation_lamp_out`.

## Node Graph
![node graph](http://www.plantuml.com/plantuml/proxy?cache=no&src=https://raw.githubusercontent.com/eve-autonomy/delivery_reservation_lamp_manager/main/docs/node_graph.pu)

## Parameter description
This node has no parameters.
