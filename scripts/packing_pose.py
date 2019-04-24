# -*- coding: utf-8 -*-
#!/usr/bin/env python
import sys
import rospy
import actionlib
import math
from moveit_python import *
from geometry_msgs.msg import Pose, Point, Quaternion
from denso_cobotta_gripper.msg import GripperMoveAction, GripperMoveGoal
from denso_cobotta_driver.srv import GetMotorState

# NOTE: Before start this program, please launch denso_cobotta_bring.launch

joints_name = ["joint_1", "joint_2",
               "joint_3", "joint_4", "joint_5", "joint_6"]


def arm_move(move_group, degree_joints):
    radian_joints = [x / 180.0 * math.pi for x in degree_joints]
    return move_group.moveToJointPosition(joints_name, radian_joints, 0.02)


def gripper_move(gripper_client, width, speed, force, timeout=10):
    goal = GripperMoveGoal()
    goal.target_position = width
    goal.speed = speed
    goal.effort = force
    gripper_client.send_goal(goal)


def is_motor_on():
    rospy.wait_for_service(
        '/cobotta/get_motor_state', 3.0)
    try:
        get_motor_state = rospy.ServiceProxy(
            '/cobotta/get_motor_state', GetMotorState)
        res = get_motor_state()
        return res.state
    except rospy.ServiceException, e:
        print >> sys.stderr, "  Service call failed: %s" % e


joints_packing_old = [90, -60, 125, 90, -95, 0]
joints_packing_new = [90, -30, 120, -170, -94, 0]
joints_home = [0, 30, 100, 0, 50, 0]

rospy.init_node("moveit_py")
move_group = MoveGroupInterface("arm", "base_link")

gripper_client = actionlib.SimpleActionClient(
    '/cobotta/gripper_move', GripperMoveAction)

print("Move selected pose")
print("0: Old packing pose, 1: New packing pose, 2: Home pose, Others: Exit")
while True:
    input = raw_input("  Select the value: ")
    if input.isdigit():
        input = int(input)
    joints = []

    if input == 0:
        joints = joints_packing_old
    elif input == 1:
        joints = joints_packing_new
    elif input == 2:
        joints = joints_home
    else:
        break

    if is_motor_on() is not True:
        print >> sys.stderr, "  Please motor on."
        continue
    arm_move(move_group, joints)
    gripper_move(gripper_client, 0.03, 10, 10, 10)
