/*
 * Copyright (C) 2018-2019  DENSO WAVE INCORPORATED
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 */
#ifndef GRIPPER_VACUUM_H
#define GRIPPER_VACUUM_H

#include "denso_cobotta_gripper/gripper_base.h"
#include "denso_cobotta_gripper/VacuumMoveAction.h"

namespace denso_cobotta_gripper
{
enum class VacuumMoveDirection
{
  Suction = -1,
  Stop = 0,
  Blow = 1,
};

using namespace denso_cobotta_gripper;

class GripperVacuum : public GripperBase
{
public:
  static constexpr const char* TAG = "GripperVacuum";

  GripperVacuum();
  virtual ~GripperVacuum() = default;
  bool initialize(ros::NodeHandle& nh) override;
  bool read(void) override;
  bool write(void) override;
  bool update(void) override;

private:
  bool publish(void);
  bool subscribe(void);
  bool initActionServers(ros::NodeHandle& nh);
  bool initGripperMove(const double& target_position, const double& speed_percentage);
  bool updateVacuumDetectParameter(double speed_percentage);

  // Action callback functions.
  bool vacuumMoveActionGoal(const VacuumMoveGoalConstPtr& goal);
  void actionCancel(void);
  void actionFeedback();

  // Action servers.
  std::shared_ptr<actionlib::SimpleActionServer<VacuumMoveAction>> as_vacuum_move_;
};

}  // namespace denso_cobotta_gripper

#endif  // GRIPPER_VACUUM_H
