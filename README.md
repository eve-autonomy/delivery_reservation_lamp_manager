# Lamp manager for delivery reservation

## Overview
By changing lighting pattern of the lamp, this node notifies surrounding workers of on-demand delivery reservation status and shutdown acceptance status through `/dio_ros_driver`. 

Lighting patterns are listed below.

|Reservation status|lighting pattern|
|:-----------------|:---------------|
|Not reserved      |Off             |
|In progress       |Blink once every second |
|Reserved          |Light up        |
|Standby for shutdown |Blink once every 0.5 seconds |
|Start of shutdown |Light up        |

## Input and Output
- input
  - from [autoware_state_machine](https://github.com/eve-autonomy/autoware_state_machine) and [shutdown_manager](https://github.com/eve-autonomy/shutdown_manager)
    - `/delivery_reservation_lock_state` \[[autoware_state_machine_msgs/msg/StateLock](https://github.com/eve-autonomy/autoware_state_machine_msgs/blob/main/msg/StateLock.msg)\]:<br>Reservation status for on-demand delivery. Or shutdown acceptance status.
- output
  - to [dio_ros_driver](https://github.com/tier4/dio_ros_driver)
    - `/dio/dout3` \[[dio_ros_driver/msg/DIOPort](https://github.com/tier4/dio_ros_driver/blob/develop/ros2/msg/DIOPort.msg)\]:<br>Digital-out assignment to a 3rd pin in [0-7] general-purpose outputs. This topic is remapped from `/delivery_reservation_lamp_out`.

## Node Graph
![node graph](http://www.plantuml.com/plantuml/proxy?cache=no&src=https://raw.githubusercontent.com/eve-autonomy/delivery_reservation_lamp_manager/main/docs/node_graph.pu)

## Parameter description
This node has no parameters.
