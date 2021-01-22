# CHANGELOG

## [1.2.4] -- 2021-01-20
### Fixed
- Overspeed ([#14](https://github.com/DENSORobot/denso_cobotta_ros/issues/14))
  * Changed planning_adapters from `AddTimeParameterization` to `AddTimeOptimalParameterization`
    in denso_cobotta_moveit_config/launch/ompl_planning_pipeline.launch.xml.

### Changed
- Support email address
  * Changed to fa-support@denso-wave.com.


## [1.2.3] -- 2020-07-30
Melodic migration <http://wiki.ros.org/melodic/Migration>

### Changed
- ROS Melodic
  * Added denso_cobotta_moveit_config/config/sensors_3d.yaml.
  * Removed xacro option "--inorder".
  * Removed parameter "kinematics_solver_attempts".
  * Changed namespace of PID parameters.

### Fixed
- Removing Deprecated Exception Specifications from C++17

## [1.2.2] -- 2019-09-11
### Added
- Gazebo
  * Added tuned PID parameters.

### Changed
- joint_limits.yaml
  * Changed max_velocity

## [1.2.2-dev] -- 2019-07-09
### Added
- [Testing] Gazebo
  * Supported arm simulation with gazebo.
  * Gripper simulation is currently not supported.
  * PID gains are currently not tuned.

    Before starting gazebo simulation, **gazebo_ros_control** package is required.

  * Add `sim` argument to denso_cobotta_bringup.launch for gazebo simulation.
	```sh
	$ roslaunch denso_cobotta_bringup denso_cobotta_bringup.launch sim:=true gripper_type:=none
	```

  * [Gazebo system requirements: Intel i5 and Nvidia card.](http://gazebosim.org/tutorials?tut=guided_b1&cat=)

	COBOTTA has Atom CPU and no graphics card,
	so it doesn't meet the requirements of Gazebo.
	Do Gazebo simulation on your PC.

### Changed
- cobotta_description
  * Changed for Gazebo simulation.
- Parallel gripper
  * Fix gripper_move and gripper_action for RViz.

	Changed parameter 'target_position' to set half width '0 - 0.015'
	as we use mimic joint.

  * Fix cancel action.
  * Fix feedback action.
- Vacuum gripper
  * Fix cancel action.
  * Fix feedback action.
- scripts/packing_pose.py
  * Supported Gazebo simulation.
  * Changed the library moveit-python to moveit-commander.

## [1.2.1] -- 2019-07-03
### Added
- CALSET
  * Added CALSET to denso_cobotta_driver node for high precision arm control.
- Vacuum gripper
  * Added vacuum gripper action to denso_cobotta_gripper node.
- gripper_type argument
  * Added `gripper_type` argument to denso_cobotta_bringup.launch for supporting vacuum gripper.
- packing_pose.py
  * Python script for packing COBOTTA in a box.

### Changed
- Performance
  * Changed update rate of denso_cobotta_gripper.
- joint_limits.yaml
  * Changed max_velocity and max_acceleration to default factory setting values.

## [1.2.0] -- 2019-04-25
### Added
- [Testing] packing_pose.py
- Added to comfirm the linux-driver version

### Changed
- Changed some asynchronous services to synchronous ones.
  * /cobotta/set_motor_state
  * /cobotta/set_brake_state
  * /cobotta/clear_error
  * /cobotta/clear_robot_error
  * /cobotta/clear_safe_state

- Renamed 3 buttons on 3-axis
  * position button -> function button
  * open button -> plus button
  * close button -> minus button

- Renamed topic to under_scored (cf. wiki.ros.org/CppStyleGuide)
  * GripperMove -> gripper_move

- LED
  * Changed LED it can not change color on error or STO by default
  * On STO, LED changed to red

### Fixed
- Fixed the bug not to open the gripper
- Some fixes to STO

## [1.1.0] -- 2019-02-26

- Publish on the web

## [0.1.0] -- 2019-01-23

- Initial release
