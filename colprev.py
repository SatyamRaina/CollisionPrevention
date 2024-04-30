#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class RobotNavigator:
    def __init__(self):
        rospy.init_node('robot_navigator', anonymous=True)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.stop_distance = 0.5  # Stop if an obstacle is closer than 0.5 meters
        self.forward_speed = 0.2  # Linear speed for forward movement
        self.angular_speed = 0.5  # Angular speed for turning
        self.obstacle_detected = False

    def scan_callback(self, data):
        # Check if an obstacle is detected within the stop distance
        min_distance = min(data.ranges)
        if min_distance < self.stop_distance:
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False

    def stop_robot(self):
        # Stop the robot's movement
        cmd_msg = Twist()
        cmd_msg.linear.x = 0
        cmd_msg.angular.z = 0
        self.cmd_pub.publish(cmd_msg)

    def move_forward(self):
        # Move the robot forward while checking for obstacles
        cmd_msg = Twist()
        cmd_msg.linear.x = self.forward_speed
        self.cmd_pub.publish(cmd_msg)

    def turn(self):
        # Turn the robot to avoid obstacles
        cmd_msg = Twist()
        cmd_msg.angular.z = self.angular_speed
        self.cmd_pub.publish(cmd_msg)

    def navigate(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            if self.obstacle_detected:
                self.stop_robot()
                self.turn()  # Turn to avoid the obstacle
            else:
                self.move_forward()
            rate.sleep()

if __name__ == '__main__':
    try:
        navigator = RobotNavigator()
        navigator.navigate()
    except rospy.ROSInterruptException:
        pass
