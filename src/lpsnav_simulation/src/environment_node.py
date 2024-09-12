#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray

class EnvironmentNode:
    def __init__(self):
        # Initialize the ROS1 node
        rospy.init_node('environment_node', anonymous=True)
        
        # Dictionary to store agent positions
        self.agent_positions = {}
        
        # Publisher to broadcast the environment update
        self.update_pub = rospy.Publisher('/environment/update', Float32MultiArray, queue_size=10)

        # Timer to periodically broadcast the environment state (every 0.1 seconds)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.broadcast_environment)

    def update_position(self, agent_id, position):
        # Update the position of the agent with the given ID
        self.agent_positions[agent_id] = position

    def broadcast_environment(self, event):
        # Create a Float32MultiArray message to broadcast agent positions
        msg = Float32MultiArray()
        flattened_positions = []

        # Flatten the agent positions into a single list [x1, y1, x2, y2, ...]
        for pos in self.agent_positions.values():
            flattened_positions.extend([pos[0], pos[1]])

        # Set the data of the message and publish it
        msg.data = flattened_positions
        self.update_pub.publish(msg)

if __name__ == '__main__':
    try:
        # Create an instance of EnvironmentNode and spin to keep the node alive
        node = EnvironmentNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

