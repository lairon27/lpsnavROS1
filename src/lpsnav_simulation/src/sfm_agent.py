#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Twist
import numpy as np

class SfmAgent:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('sfm_agent', anonymous=True)

        # Get start and goal positions from ROS parameters
        start_x = rospy.get_param('~start_x', -5.0)
        start_y = rospy.get_param('~start_y', 0.0)
        goal_x = rospy.get_param('~goal_x', -5.0)
        goal_y = rospy.get_param('~goal_y', 7.0)
        max_speed = rospy.get_param('~max_speed', 0.5)

        # Initialize position, goal, velocity, and max speed
        self.position = np.array([start_x, start_y])
        self.goal = np.array([goal_x, goal_y])
        self.max_speed = max_speed
        self.velocity = np.zeros(2)

        # ROS Publishers for pose and velocity commands
        self.pose_pub = rospy.Publisher('/agent_sfm/pose', PoseStamped, queue_size=10)
        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Timer to periodically update the agent's position
        rospy.Timer(rospy.Duration(0.1), self.update_position)

    def update_position(self, event):
        # Calculate direction toward the goal
        direction = self.goal - self.position
        distance = np.linalg.norm(direction)

        if distance > 0.05:  # Threshold to detect when goal is reached
            direction /= distance  # Normalize direction
            speed = min(self.max_speed, distance)
            self.velocity = direction * speed
            self.position += self.velocity * 0.1  # Update position based on velocity

            rospy.loginfo(f"SfmAgent moving: Position: {self.position}, Goal: {self.goal}")

            # Publish pose and twist for movement
            self.publish_pose()
            self.publish_twist(speed, np.arctan2(direction[1], direction[0]))
        else:
            rospy.loginfo("SfmAgent reached the goal")
            self.stop_movement()

    def publish_pose(self):
        # Publish the robot's current position
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'map'
        pose.pose.position.x = float(self.position[0])
        pose.pose.position.y = float(self.position[1])
        pose.pose.orientation.w = 1.0
        self.pose_pub.publish(pose)

    def publish_twist(self, speed, heading):
        # Publish twist commands (linear and angular velocities)
        twist = Twist()
        twist.linear.x = speed  # Forward movement
        twist.angular.z = heading  # Turn toward goal
        self.twist_pub.publish(twist)

    def stop_movement(self):
        # Publish zero velocities to stop the robot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.twist_pub.publish(twist)

# Main function to start the SfmAgent
if __name__ == '__main__':
    try:
        agent = SfmAgent()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

