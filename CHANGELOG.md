# CHANGELOG

## [Unreleased]
- CALSET

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
