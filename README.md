# denso_cobotta_ros

In order to operate cobotta, execute following command.

```sh
roslaunch denso_cobotta_bringup denso_cobotta_bringup.launch
roslaunch denso_cobotta_driver denso_cobotta_driver.launch
```

After executing the above command, you can use the following services and topics.

- service
  - /cobotta/clear_error
  - /cobotta/clear_robot_error
  - /cobotta/clear_safe_state
  - /cobotta/get_brake_state
  - /cobotta/get_motor_state
  - /cobotta/set_LED_state
  - /cobotta/set_brake_state
  - /cobotta/set_motor_state
- topic
  - /cobotta/close_button
  - /cobotta/gripper_move
  - /cobotta/gripper_action
  - /cobotta/miniIO_input
  - /cobotta/miniIO_output
  - /cobotta/open_button
  - /cobotta/position_button
  - /cobotta/robot_state
  - /cobotta/safe_state

## Installation and requirements

Please see the [Getting stated](https://densorobot.github.io/docs/denso_cobotta_ros/getting_started.html) page.
