import rclpy
from rclpy.node import Node
import sys
import time
import math
import os
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import TwistWithCovariance

class Lidar2d(Node):
	def __init__(self):
		super().__init__('LidarMapper')
		qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,history=rclpy.qos.HistoryPolicy.KEEP_LAST,depth=1)
		self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile=qos_policy)
		self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile=qos_policy)
		self.string_pub = self.create_publisher(String, '/String', 10)
		self.map = np.zeros([300, 300])
		self.map.fill(-1)
		self.odometry = Odometry()
		self.pos = [0, 0]
		self.q = 0
		
	def scan_callback(self, msg):
		self.pos[0] = 150 - int(round(self.odometry.pose.pose.position.x * 10))
		self.pos[1] = 150 - int(round(self.odometry.pose.pose.position.y * 10))
		print(self.pos)
		
		#Find "infinite" ranges from scan and set them to range 3.6
		ranges = msg.ranges
		for r in range(len(ranges)):
			if ranges[r] > 3.5:
				ranges[r] = 3.6
		
		#Get points list from scanned poinst
		self.q = self.odometry.pose.pose.orientation
		yaw = self.quarternion_to_euler(self.q.x, self.q.y, self.q.z, self.q.w)
		points = self.polar_to_cartesian_coordinate(ranges, msg.angle_min, msg.angle_max, yaw)
		points = np.around(points, decimals = 1)
		points = points*10
		
		#mapping with bresenham algorithm
		for point in points:
			point_x = int(point[0])
			point_y = int(point[1])
			zeroes = self.bresenham_points(self.pos, [self.pos[0] - point_x, self.pos[1] - point_y])
			
			#empty positions
			for zero in zeroes:
				self.map[zero[0]][zero[1]] = 0
				
			#occupied positions
			if msg.ranges[r] < 3.59:
				self.map[self.pos[0] - point_x][self.pos[1] - point_y] = 1
		
		#current robot position is occupied
		self.map[self.pos[0]][self.pos[1]] = 1

		
	#doesnt really do anything
	def odom_callback(self, odometry):
		self.odometry = odometry
		
	#bresenham algorith to fill the empty/free spaces in map with zeroes
	def bresenham_points(self, p0, p1):
		point_list = []
		x0, y0 = p0[0], p0[1]
		x1, y1 = p1[0], p1[1]
		dx = abs(x1-x0)
		dy = abs(y1-y0)

		if x0 < x1:
			sx = 1
		else:
			sx = -1

		if y0 < y1:
        		sy = 1
		else:
			sy = -1

		err = dx-dy
    
		while True:
			point_list.append([x0, y0])
			if x0 == x1 and y0 == y1:
				break
            
			e2 = 2*err
			if e2 > -dy:
				err = err - dy
				x0 = x0 + sx
				
			if e2 < dx:
				err = err + dx
				y0 = y0 + sy
    
		point_list.pop(0)
		point_list.pop(len(point_list) -1)
		return point_list
	
	#get yaw based on orientation (called in scan_callback)
	def quarternion_to_euler(self, x, y, z ,w):
		t0 = 2.0 * (w * z + x * y)
		t1 = 1.0 - 2.0 * (y * y + z * z)
		yaw = math.atan2(t0, t1)
		return yaw
		
	#get cartesian coordinates
	def polar_to_cartesian_coordinate(self, ranges, angle_min, angle_max, yaw):
		angle_step = (angle_max - angle_min) / len(ranges)
		angle = angle_min + yaw
		points = []
		for range in ranges:
			x = range * np.cos(angle)
			y = range * np.sin(angle)
			angle += angle_step
			points.append([x,y])
		return points
	
	#update csv file, which includes map
	def file_updater(self):
		print('File saved')
		np.savetxt('gridmap.csv', self.map, delimiter=',')
		msg = String()
		msg.data = str(os.path.abspath('gridmap.csv'))
		self.string_pub.publish(msg)
		
def main(args=None):
	rclpy.init(args=args)
	lidar2d = Lidar2d()
	
	time.sleep(1)
	rclpy_check_rate = 10 
	rclpy_check_rate = lidar2d.create_rate(10, lidar2d.get_clock())

	lidar2d.get_logger().info("Starting mapper...")
	try:
		try:
			while rclpy.ok():
				filter_timer = lidar2d.create_timer(2.0, lidar2d.file_updater)
				rclpy.spin(lidar2d)             
				pass
			    

		except KeyboardInterrupt :
			lidar2d.get_logger().error('Keyboard Interrupt detected! Trying to stop mapping node!')
	except Exception as e:
		lidar2d.destroy_node()
		lidar2d.get_logger().info("Failed %r."%(e,))
	finally:
		rclpy.shutdown()
		lidar2d.destroy_node() 
		
if __name__ == '__main__':
	main()	
		
		
		
		
		
		
