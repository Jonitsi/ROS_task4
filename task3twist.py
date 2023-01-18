import rclpy
import sys
import time
import math
import os
import numpy as np
import matplotlib.pyplot as plt
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetPlan
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class Publisher(Node):
	def __init__(self):
		super().__init__('LidarMapper')
		qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,history=rclpy.qos.HistoryPolicy.KEEP_LAST,depth=1)
		client_group = MutuallyExclusiveCallbackGroup()
		string_group = MutuallyExclusiveCallbackGroup()
		odometry_group = MutuallyExclusiveCallbackGroup()
		
		self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, callback_group=odometry_group, qos_profile=qos_policy)
		self.string_sub = self.create_subscription(String, '/String', self.string_callback, callback_group=string_group, qos_profile=qos_policy)
		self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
		self.x_coordinate = 0
		self.y_coordinate = 0
		self.q = 0
		self.theta = 0
		self.checkpoint_list = [[2, -6], [5, 5], [-1, 5], [-7, -1], [0,0]] #list of checkpoints
		self.checkpoint = 1.0
		self.path = []
		self.goal = []
		self.twist = Twist()
		self.checkpoint_index = 0
		self.filepath = ""
		self.trajectory = []
		self.twist_points = []
		self.visited = 0
		
		#New for task2
		self.client = self.create_client(GetPlan, '/get_dijkstra_path', callback_group=client_group)
		while not self.client.wait_for_service(timeout_sec=0.5):
			self.get_logger().info('service unavailable, waiting...')
		self.request = GetPlan.Request()
	
	def odom_callback(self, odom):
		self.x_coordinate = odom.pose.pose.position.x
		self.y_coordinate = odom.pose.pose.position.y
		#self.trajectory.append([150 - odom.pose.pose.position.x * 10, 150 - odom.pose.pose.position.y * 10])
		np.append(self.trajectory, [150 - odom.pose.pose.position.x * 10, 150 - odom.pose.pose.position.y * 10])	
		self.q = odom.pose.pose.orientation
		self.theta = self.quarternion_to_euler(self.q.x, self.q.y, self.q.z, self.q.w)
		
		if len(self.path) == 0:
			self.send_request()
			self.goal = self.checkpoint_list[self.path[0]]
		inc_x = self.goal[0] - self.x_coordinate
		inc_y = self.goal[1] - self.y_coordinate
		angle_to_cp = math.atan2(inc_y, inc_x)
		
		if abs(angle_to_cp - self.theta) > 0.1:
			self.twist.linear.x = 0.0
			self.twist.angular.z = 0.3
			self.publisher.publish(self.twist)
			self.twist_points.append(-1)
			
		elif abs(inc_x) < 0.1 and abs (inc_y) < 0.1:
			if self.checkpoint_index < len(self.path):
				self.checkpoint_index = self.checkpoint_index + 1
				if self.checkpoint_index < len(self.path):
					self.goal = self.checkpoint_list[self.path[self.checkpoint_index]]
					print('Checkpoint reached. Next checkpoint: ', self.path[self.checkpoint_index])
				else:
					print('Checking next checkpoint...')
					self.visited = self.visited +1
					if self.visited > len(self.checkpoint_list):
						self.twist.linear.x == 0.0
						self.twist.angular.z == 0.0
						self.publisher.publish(self.twist)
						
						array = np.genfromtxt(self.filepath, delimiter = ',')
						plt.matshow(array)
						self.trajectory = np.array(self.trajectory)
						#print(self.trajectory)
						plt.plot(int(self.trajectory[:,1]), int(self.trajectory[:,0]))
						self.checkpoint_list = np.array(self.checkpoint_list)
						self.checkpoint_list = 150 - self.checkpoint_list * 10
						plt.scatter(self.checkpoint_list[:,1], self.checkpoint_list[:,0])
						plt.show()
						plt.plot(self.twist_points)
						plt.show()
						
					self.checkpoint_index = 0
					self.path = [0]
					#time.sleep(1)
					self.send_request() 
					#self.checkpoint = self.checkpoint_list[self.path[self.checkpoint_index]]
					self.checkpoint = self.checkpoint_list[self.path[0]]
					
		else:
			self.twist.linear.x = 0.3
			self.twist.angular.z = 0.0
			self.publisher.publish(self.twist)
			self.twist_points.append(1)
			
	def quarternion_to_euler(self, x, y, z ,w):
		t0 = 2.0 * (w * z + x * y)
		t1 = 1.0 - 2.0 * (y * y + z * z)
		yaw_z = math.atan2(t0, t1)
		return yaw_z
	
	def string_callback(self, msg):
		if self.filepath == "":
			print('Filepath found')
			self.filepath = msg.data
	
	def send_request(self):
		self.request.start.pose.position.x = self.x_coordinate
		self.request.start.pose.position.y = self.y_coordinate
		#self.request.goal.pose.position.x = float(self.checkpoint)
		self.future = self.client.call_async(self.request)
		print('Waiting for checkpoint...')
		rclpy.spin_until_future_complete(self, self.future)
		print('Checkpoint received!')
		reply = self.future.result()
		for stpose in reply.plan.poses:
			self.path.append(int(stpose.pose.position.x))
		#print(self.path)
			
def main(args=None):
	rclpy.init(args=args)
	publisher = Publisher()
	while rclpy.ok():
		rclpy.spin_once(publisher)
	publisher.destroy_node()
	rclpy.shutdown()			

if __name__ == '__main__':
	main()			
			
			
