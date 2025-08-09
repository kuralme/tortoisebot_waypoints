#! /usr/bin/env python

import rospy
import actionlib
import unittest
import rostest
import numpy as np
from geometry_msgs.msg import Point
from tortoisebot_waypoints.msg import WaypointActionAction, WaypointActionGoal, WaypointActionResult
from nav_msgs.msg import Odometry
from tf import transformations
import math

PKG = 'tortoisebot_waypoints'
NAME = 'waypoints_test_ros_as'


class TestWaypointActionServer(unittest.TestCase):

    def setUp(self):
        '''This runs before each test.'''

        rospy.init_node('waypoint_action_test_node', anonymous=True)
        self.client = actionlib.SimpleActionClient('tortoisebot_as', WaypointActionAction)

        rospy.loginfo("Waiting for action server...")
        self.assertTrue(self.client.wait_for_server(rospy.Duration(10)), "Action server not available within timeout")
        rospy.loginfo("Connected to action server.")

        self.robot_position = None
        self.robot_yaw = None

        self.goal1 = WaypointActionGoal()
        self.goal2 = WaypointActionGoal()
        self.goal1.position = Point(-0.3, -0.3, 0.0)
        self.goal2.position = Point(-0.2, 0.4, 0.0)

        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.sleep(1) # to ensure odom is received

    def odom_callback(self, msg):
        self.robot_position = msg.pose.pose.position
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        euler = transformations.euler_from_quaternion(quaternion)
        self.robot_yaw = euler[2]

    def test_goal_1(self):
        '''Test to check position and yaw after moving to goal 1'''
        
        rospy.loginfo("Sending goal 1...")
        self.client.send_goal(self.goal1)
        self.client.wait_for_result(timeout=rospy.Duration(25))

        result = self.client.get_result()
        rospy.loginfo(f"Result 1: {result}")
        self.assertIsNotNone(result, "No result returned for goal 1")
        self.assertTrue(result.success, "Goal 1 was not successful")

        distance_to_goal = np.linalg.norm([self.robot_position.x - self.goal1.position.x, 
                                           self.robot_position.y - self.goal1.position.y])
        desired_yaw = math.atan2(self.goal1.position.y - 0.0, self.goal1.position.x - 0.0) # Angle to goal1 from origin

        # Check that the distance to goal and final heading are within threshold
        self.assertLess(distance_to_goal, 0.1, "Robot is not within acceptable distance to goal 1")
        self.assertAlmostEqual(self.robot_yaw, desired_yaw, delta=0.25, msg="Yaw mismatch goal 1")


    def test_goal_2(self):
        '''Test to check position and yaw after moving to goal 2'''
        
        rospy.loginfo("Sending goal 2...")
        self.client.send_goal(self.goal2)
        self.client.wait_for_result(timeout=rospy.Duration(25))

        result = self.client.get_result()
        rospy.loginfo(f"Result 2: {result}")
        self.assertIsNotNone(result, "No result returned for goal 2")
        self.assertTrue(result.success, "Goal 2 was not successful")

        distance_to_goal = np.linalg.norm([self.robot_position.x - self.goal2.position.x, 
                                           self.robot_position.y - self.goal2.position.y])
        desired_yaw = math.atan2(self.goal2.position.y - self.goal1.position.y,
             self.goal2.position.x - self.goal1.position.x) # Angle to goal2 from goal1

        # Check that the distance to goal and final heading are within threshold
        self.assertLess(distance_to_goal, 0.1, "Robot is not within acceptable distance to goal 2")
        self.assertAlmostEqual(self.robot_yaw, desired_yaw, delta=0.25, msg="Yaw mismatch goal 2")

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestWaypointActionServer)
