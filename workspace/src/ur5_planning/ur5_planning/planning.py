#!/usr/bin/env python3
"""
A script to outline the fundamentals of the moveit_py motion planning API.
From: https://raw.githubusercontent.com/ros-planning/moveit2_tutorials/main/doc/examples/motion_planning_python_api/scripts/motion_planning_python_api_tutorial.py
"""

import time

import rclpy
from rclpy.logging import get_logger

from geometry_msgs.msg import PoseStamped
from moveit.planning import (
    MoveItPy,
)

def plan_and_execute(
    robot,
    planning_component,
    logger,
    sleep_time=0.0,
):
    """Helper function to plan and execute a motion."""
    # plan to goal
    logger.info("Planning trajectory")

    plan_result = planning_component.plan()

    # execute the plan
    if plan_result:
        logger.info("Executing plan")
        robot_trajectory = plan_result.trajectory
        robot.execute(robot_trajectory, controllers=[])
    else:
        logger.error("Planning failed")

    time.sleep(sleep_time)


def main():

    ###################################################################
    # MoveItPy Setup
    ###################################################################
    rclpy.init()
    logger = get_logger("moveit_py.pose_goal")

    # instantiate MoveItPy instance and get planning component
    ur5 = MoveItPy(node_name="moveit_py")
    ur5_arm = ur5.get_planning_component("hand")
    logger.info("MoveItPy instance created")

    ###########################################################################
    # Set goal state with PoseStamped message
    ###########################################################################

    # set plan start state to current state
    ur5_arm.set_start_state_to_current_state()

    pose_goal = PoseStamped()
    pose_goal.header.frame_id = "base_link"
    pose_goal.pose.orientation.w = 1.0
    pose_goal.pose.position.x = 0.28
    pose_goal.pose.position.y = -0.2
    pose_goal.pose.position.z = 0.5
    ur5_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="wrist_3_link")

    # plan to goal
    plan_and_execute(ur5, ur5_arm, logger, sleep_time=3.0)

if __name__ == "__main__":
    main()
