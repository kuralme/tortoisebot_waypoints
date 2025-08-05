# tortoisebot_waypoints

This ROS package implements an **action server** for navigating the TortoiseBot to user-defined 2D waypoints using the `/cmd_vel` and `/odom` topics. The goal is to robot reach the target position and orientation based on action goals in simulation (ROS Noetic/Gazebo).

The main goal of the package is unit test with `rostest` and `unittest` to verify that robot reaches the goal position and orientation(within a small tolerance)

## Running unit testing

Make sure your ROS workspace built, sourced and Tortoisebot is ready within Gazebo environment.

- Start the waypoint server:

```bash
source devel/setup.bash
rosrun tortoisebot_waypoints tortoisebot_action_server.py
```

- In second terminal, navigate to your ROS workspace and run testing:

```bash
source devel/setup.bash
rostest tortoisebot_waypoints waypoints_test.test --reuse-master
```

**Expected result:**
```
...
[ROSTEST]-----------------------------------------------------------------------

[tortoisebot_waypoints.rosunit-waypoints_test/test_goal_1][passed]
[tortoisebot_waypoints.rosunit-waypoints_test/test_goal_2][passed]

SUMMARY
 * RESULT: SUCCESS
 * TESTS: 2
 * ERRORS: 0
 * FAILURES: 0

rostest log file is in /home/user/.ros/log/rostest-1_xterm-13821.log
```


### Testing for failure case

Modify **test/waypoints_test_ros_as.py** script to change goals to something unachievable. Example:
```
...
Line 35        self.goal1.position = Point(10., 10., 0.0)
Line 36        self.goal2.position = Point(-10., -10., 0.0)
...
```

Build the ROS workspace and run test node again. Since the goal won't be reached in time, test will result in time out and fail: "No result returned for goal X".
```

SUMMARY
 * RESULT: FAIL
 * TESTS: 2
 * ERRORS: 0
 * FAILURES: 2

rostest log file is in /home/user/.ros/log/rostest-1_xterm-13821.log
```
