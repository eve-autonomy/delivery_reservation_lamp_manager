# delivery_reservation_lamp_manager

## Overview
By changing the lighting pattern of the lamp, this node notifies senders of the on-demand delivery reservation status. 

The lighting pattern is defined as follows.
<table>
  <thead>
    <tr>
      <th scope="col">Reservation status</th>
	    <th scope="col">lighting pattern</th>
	  </tr>
  </thead>
  <tbody>
    <tr>
      <td>Not reserved</td>
      <td>Keeps going off</td>
    </tr>
    <tr>
      <td>In progress</td>
      <td>Blinks</td>
    </tr>
    <tr>
      <td>Reserved</td>
      <td>Keeps lighting</td>
    </tr>
  </tbody>
</table>

## Input and Output
- input
  - from [autoware_state_machine](https://github.com/eve-autonomy/autoware_state_machine)
    - `/autoware_state_machine/lock_state` : Reservation status for on-demand delivery.
- output
  - to [dio_ros_driver](https://github.com/tier4/dio_ros_driver)
    - `/dio/dout3` : GPIO output topic. (this topic remapping from /delivery_reservation_lamp_out)

## Node Graph
![node graph](http://www.plantuml.com/plantuml/proxy?src=https://raw.githubusercontent.com/eve-autonomy/delivery_reservation_lamp_manager/docs/node_graph.pu)

## Parameter description
This node has no parameters.
