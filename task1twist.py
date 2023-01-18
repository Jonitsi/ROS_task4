import rclpy
from rclpy.node import Node
import sys
import time
import math
import os
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import Twist


#Heavily relied on: https://www.youtube.com/watch?v=eJ4QPrYqMlw tutorial by "the construct"

class Publisher(Node):
	def __init__(self):
		super().__init__('LidarMapper')
		qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,history=rclpy.qos.HistoryPolicy.KEEP_LAST,depth=1)
		
		self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile=qos_policy)
		self.string_sub = self.create_subscription(String, '/String', self.string_callback, qos_profile=qos_policy)
		self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
		self.x_coordinate = 0
		self.y_coordinate = 0
		self.q = 0
		self.yaw = 0
		self.checkpoint_list = [[2, -6], [5, 5], [-1, 5], [-7, -1], [0,0]]
		self.checkpoint = self.checkpoint_list[0]
		self.twist = Twist()
		self.checkpoint_index = 0
		self.filepath = ""
		self.trajectory = []
		self.twist_points = []
	
	def odom_callback(self, odom):
		self.x_coordinate = odom.pose.pose.position.x
		self.y_coordinate = odom.pose.pose.position.y
		self.trajectory.append([150 - odom.pose.pose.position.x * 10, 150 - odom.pose.pose.position.y * 10])
		self.q = odom.pose.pose.orientation
		self.yaw = self.quarternion_to_euler(self.q.x, self.q.y, self.q.z, self.q.w)
		inc_x = self.checkpoint[0] - self.x_coordinate
		inc_y = self.checkpoint[1] - self.y_coordinate
		angle_to_cp = math.atan2(inc_y, inc_x)
		
		if abs(angle_to_cp - self.yaw) > 0.1:
			self.twist.linear.x = 0.0
			self.twist.angular.z = 0.3
			self.publisher.publish(self.twist)
			self.twist_points.append(-1)
			
		elif abs(inc_x) < 0.2 and abs (inc_y) < 0.2:
			if self.checkpoint_index < len(self.checkpoint_list):
				self.checkpoint_index = self.checkpoint_index + 1
				if self.checkpoint_index < len(self.checkpoint_list):
					self.checkpoint = self.checkpoint_list[self.checkpoint_index]
					print("Checkpoint ", self.checkpoint_list[self.checkpoint_index - 1], " reached, next checkpoint: ", self.checkpoint_list[self.checkpoint_index])
				else:
					print("Reached destination")
			if self.checkpoint_index > len(self.checkpoint_list) - 1:
				self.twist.linear.x = 0.0
				self.twist.angular.z = 0.0
				self.publisher.publish(self.twist)
				
				array = np.genfromtxt(self.filepath, delimiter = ',')
				plt.matshow(array)
				self.trajectory = np.array(self.trajectory)
				plt.plot(self.trajectory[:, 1], self.trajectory[:, 0])
				plt.show()
				plt.plot(self.twist_points)
				plt.show()
				
				self.checkpoint = self.checkpoint_list[self.checkpoint_index]
		else:
			self.twist.linear.x = 0.3
			self.twist.angular.z = 0.0
			self.publisher.publish(self.twist)
			self.twist_points.append(1)
			
	def quarternion_to_euler(self, x, y, z ,w):
		t0 = 2.0 * (w * z + x * y)
		t1 = 1.0 - 2.0 * (y * y + z * z)
		yaw = math.atan2(t0, t1)
		return yaw
	
	def string_callback(self, msg):
		if self.filepath == "":
			print('Filepath found')
			self.filepath = msg.data
			
def main(args=None):
	rclpy.init(args=args)
	feature_extracter = Publisher()
	rclpy.spin(feature_extracter) 
	feature_extracter.destroy_node()
	rclpy.shutdown()			

if __name__ == '__main__':
	main()			
			
			
