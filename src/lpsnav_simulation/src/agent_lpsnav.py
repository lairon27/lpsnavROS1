#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Float32MultiArray
import numpy as np
from utils import helper  # Assuming you have a helper module for some geometric functions


class LpsnavAgent:
    def __init__(self, agent_id, start, goal, max_speed, config):
        # Initialize ROS node
        rospy.init_node(f'agent_lpsnav_{agent_id}_node', anonymous=True)

        self.agent_id = agent_id
        self.position = np.array(start)
        self.goal = np.array(goal)
        self.max_speed = max_speed
        self.velocity = np.zeros(2)  # Initial velocity
        self.heading = np.arctan2(goal[1] - start[1], goal[0] - start[0])  # Initial heading
        self.config = config

        # Initialize internal Lpsnav variables
        self.heading_samples = config.get("heading_samples", 31)
        self.leg_tol = config.get("legibility_tol", 0.05)
        self.pred_tol = config.get("predictability_tol", 0.05)
        self.max_cost = float(config.get("max_cost", 10.0))
        self.receding_horiz = config.get("receding_horiz", 5)
        self.sensing_horiz = config.get("sensing_horiz", 10)
        self.speed_samples = config.get("speed_samples", 5)
        self.long_space = config.get("longitudinal_space", [0.1, 0.5])
        self.lat_space = config.get("lateral_space", [0.1, 0.2])
        self.subgoal_priors = np.array(config.get("subgoal_priors", [0.48, 0.02, 0.5]))

        # Interaction variables and primitive scores
        self.pred_pos = {}
        self.int_lines = {}
        self.pred_int_lines = {}
        self.cost_rt = self.receding_horiz
        self.cost_tp = self.config.get('prim_horiz', 1)
        self.prim_leg_score = {}
        self.prim_pred_score = {}
        self.current_leg_score = {}
        self.passing_ratio = {}
        self.speed_idx = 0
        self.heading_idx = self.heading_samples // 2
        self.col_mask = np.full((self.speed_samples, self.heading_samples), False)
        self.tau = {}
        self.pass_inf_diff_hist = {}

        # ROS Publishers and Subscribers
        self.pose_pub = rospy.Publisher(f'/agent_lpsnav_{self.agent_id}/pose', PoseStamped, queue_size=10)
        self.twist_pub = rospy.Publisher(f'/agent_lpsnav_{self.agent_id}/cmd_vel', Twist, queue_size=10)
        self.env_sub = rospy.Subscriber('/environment/update', Float32MultiArray, self.env_callback)

        # Timer to periodically update the agent's position
        rospy.Timer(rospy.Duration(0.1), self.update_position)

    def env_callback(self, msg):
        """Handle environment updates, receive data about other agents' positions."""
        other_agent_positions = np.array(msg.data).reshape(-1, 2)  # Assume environment gives [x1, y1, x2, y2, ...]
        self.get_interacting_agents(other_agent_positions)

    def get_interacting_agents(self, other_agent_positions):
        """ Identify agents that are within the sensing horizon and possibly interacting. """
        self.interacting_agents = {}
        for idx, other_pos in enumerate(other_agent_positions):
            # Calculate time to interaction and check for collision
            time_to_interaction = helper.cost_to_line(self.position, self.velocity,
                                                      self.int_lines.get(idx, np.zeros(2)), np.zeros(2))
            in_radius = helper.dist(self.position, other_pos) < self.sensing_horiz
            in_horiz = time_to_interaction < self.sensing_horiz
            intersecting = helper.is_intersecting(self.position, self.goal,
                                                  *self.int_lines.get(idx, [self.position, other_pos]))
            if in_radius and in_horiz and intersecting:
                self.interacting_agents[idx] = other_pos

    def update_position(self, event):
    	""" Update the agent's position based on current goals and interactions. """
    	# If there are no interactions, move directly to the goal
    	if not self.interacting_agents:
        	self.get_goal_prims(0.1)  # Example time delta
    	else:
        	self.get_leg_pred_prims(0.1)  # Handle interactions

    	# Calculate direction and move towards the goal
    	direction = self.goal - self.position
    	distance = np.linalg.norm(direction)

    	# Adjust the goal tolerance to check proximity before stopping
    	goal_tolerance = 0.05  # Stop when closer than 5cm to the goal
    	if distance > goal_tolerance:
        	# Normalize direction and calculate the desired heading
        	direction /= distance
        	desired_heading = np.arctan2(direction[1], direction[0])

       	 # Calculate the angular error and normalize it to [-pi, pi]
        	angular_error = desired_heading - self.heading
        	angular_error = np.arctan2(np.sin(angular_error), np.cos(angular_error))

        	# Limit the angular velocity to avoid over-rotation
        	angular_speed = np.clip(angular_error, -1.0, 1.0)

        	# Gradually reduce speed as the robot approaches the goal
        	speed = min(self.max_speed, distance)
        	self.velocity = direction * speed
        	self.position += self.velocity * 0.1  # Update position based on velocity

        	# Log the agent's position and update the heading
        	rospy.loginfo(f"LpsnavAgent moving: Position: {self.position}, Goal: {self.goal}, Distance: {distance}, Angular error: {angular_error}, Velocity: {self.velocity}")
        	self.heading = desired_heading

        	# Publish pose and twist for movement
        	self.publish_pose()
        	self.publish_twist(speed, angular_speed)
    	else:
        	rospy.loginfo(f"LpsnavAgent reached the goal within tolerance {goal_tolerance}")
        	self.stop_movement()


    def get_leg_pred_prims(self, dt):
        """ Compute motion primitives considering legibility and predictability based on interacting agents. """
        score = np.full((self.speed_samples, self.heading_samples), np.inf)
        for k in self.interacting_agents:
            legibility_score = (1 - self.tau.get(k, 0)) * self.prim_leg_score.get(k, np.zeros(1))
            predictability_score = self.tau.get(k, 0) * self.prim_pred_score.get(k, np.zeros(1))
            combined_score = legibility_score + predictability_score
            score = np.minimum(score, combined_score)
        
        # Choose best direction based on score
        self.speed_idx, self.heading_idx = np.unravel_index(np.argmax(score), score.shape)

    def get_goal_prims(self, dt):
        """ Calculate motion primitives for moving directly toward the goal. """
        next_pos = self.position + dt * self.velocity
        goal_cost = helper.dist(next_pos, self.goal)
        self.speed_idx, self.heading_idx = np.unravel_index(np.argmin(goal_cost), goal_cost.shape)

    def publish_pose(self):
        """ Publish the agent's current position. """
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'map'
        pose.pose.position.x = float(self.position[0])
        pose.pose.position.y = float(self.position[1])
        pose.pose.orientation.w = 1.0  # Simplified orientation (no rotation)
        self.pose_pub.publish(pose)

    def publish_twist(self, speed, angular_speed):
        """ Publish velocity commands to control the robot's movement. """
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = angular_speed
        self.twist_pub.publish(twist)

    def stop_movement(self):
        """ Stop the agent's movement by publishing zero velocities. """
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.twist_pub.publish(twist)


if __name__ == '__main__':
    try:
        agent = LpsnavAgent(agent_id=0, start=[0.0, 5.0], goal=[5.0, 0.0], max_speed=1.0, config={
            'heading_samples': 31,
            'legibility_tol': 0.05,
            'predictability_tol': 0.05,
            'max_cost': 10.0,
            'receding_horiz': 5,
            'sensing_horiz': 10,
            'speed_samples': 5,
            'longitudinal_space': [0.1, 0.5],
            'lateral_space': [0.1, 0.2],
            'subgoal_priors': [0.48, 0.02, 0.5]
        })
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

