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
  - from eve oss
    - `/autoware_state_machine/lock_state` : Reservation status for on-demand delivery.
- output
  - to tier iv oss
    - `/dio/dout3` : GPIO output topic. (this topic remapping from /delivery_reservation_lamp_out)

## Node Graph
![image](https://user-images.githubusercontent.com/33311630/172437128-7c8c673a-d65b-4415-b6fa-5a0274445294.png)

<details>

  <summary> plantuml </summary>

```

@startuml

rectangle "eve oss" {
  usecase "/autoware_state_machine"
  usecase "/delivery_reservation_lamp_manager"
}

rectangle "tier iv oss" {
  usecase "/dio_ros_driver"
}
(/autoware_state_machine) -> (/delivery_reservation_lamp_manager) : /autoware_state_machine/lock_state

(/delivery_reservation_lamp_manager) -> (/dio_ros_driver) : /dio/dout3

@enduml

```

</details>

## Parameter description
This node has no parameters.
