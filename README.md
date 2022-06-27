# Lamp manager for delivery reservation

## Overview
By changing lighting pattern of the lamp, this node notifies surrounding workers of on-demand delivery reservation status through `/dio_ros_driver`. 

Lighting patterns are listed below.

|Reservation status|lighting pattern|
|:-----------------|:---------------|
|Not reserved      |Off             |
|In progress       |Blinks          |
|Reserved          |Light up        |

## Input and Output
- input
  - from [autoware_state_machine](https://github.com/eve-autonomy/autoware_state_machine)
    - `/autoware_state_machine/lock_state` \[[autoware_state_machine_msgs/msg/StateLock](https://github.com/eve-autonomy/autoware_state_machine_msgs/blob/main/msg/StateLock.msg)\]:<br>Reservation status for on-demand delivery.
- output
  - to [dio_ros_driver](https://github.com/tier4/dio_ros_driver)
    - `/dio/dout3` \[[dio_ros_driver/msg/DIOPort](https://github.com/tier4/dio_ros_driver/blob/develop/ros2/msg/DIOPort.msg)\]:<br>Digital-out assignment to a 3rd pin in [0-7] general-purpose outputs. This topic is remapped from `/delivery_reservation_lamp_out`.

## Node Graph
![node graph](http://www.plantuml.com/plantuml/proxy?cache=no&src=https://raw.githubusercontent.com/eve-autonomy/delivery_reservation_lamp_manager/main/docs/node_graph.pu)

## Parameter description
This node has no parameters.
