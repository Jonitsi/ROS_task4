import sys
import rclpy
import time
import math
import numpy as np
import matplotlib.pyplot as plt

from rclpy.node import Node
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped
from queue import PriorityQueue

class Service(Node):
	def __init__(self):
		super().__init__('Path_service')
		self.service = self.create_service(GetPlan, '/get_dijkstra_path', self.plan_callback)
		
		self.checkpoint_list = [[2, -6], [5, 5], [-1, 5], [-7, -1], [0,0]]
		self.cp_links = [[0,1],[0,4],[1,2],[2,3],[4,3]] #connections between checkpoints
		self.adjacency_matrix = np.zeros([len(self.checkpoint_list), len(self.checkpoint_list)])
		self.weight_matrix = np.zeros([len(self.checkpoint_list), len(self.checkpoint_list)])
		
		for i in self.cp_links:
			self.adjacency_matrix[i[0], i[1]] = 1
			self.adjacency_matrix[i[1], i[0]] = 1
			weight = self.distance_between_checkpoints(self.checkpoint_list[i[0]], self.checkpoint_list[i[1]])
			self.weight_matrix[i[0], i[1]] = weight
			self.weight_matrix[i[1], i[0]] = weight
			
		print('started service')
		
	def plan_callback(self, request, response):
		start = request.start.pose.position
		goal_cp = int(request.goal.pose.position.x)
		start_cp = self.find_closest_node(start)
		
		visited = set()
		q = PriorityQueue()
		q.put((0, start_cp, [start_cp]))
		visited.add(start_cp)
		
		while not q.empty() :
			current_cost, current_cp, current_path = q.get()
		
			if current_cp == goal_cp:
				print('plan ready, path is: {}', format(current_path))
				final_path = current_path
				break
			
			for neighbor_id, is_a_neighbor in enumerate(self.adjacency_matrix[current_cp]):
				if is_a_neighbor == 1:
					if neighbor_id not in visited:
						visited.add(neighbor_id)
						q.put((current_cost + self.weight_matrix[current_cp][neighbor_id], neighbor_id, current_path + [neighbor_id]))
			
		for i in final_path:
			pose = PoseStamped()
			pose.pose.position.x = float(i)
			response.plan.poses.append(pose)
		print('checkpoint sent')
		return response
	
	def find_closest_node(self, start):
		x = start.x
		y = start.y
		nearest_index = 0
		min_distance = 100000
		for index, checkpoint in enumerate(self.checkpoint_list):
			distance = self.distance_between_checkpoints([x, y], checkpoint)
			if distance < min_distance:
				nearest_index = index
				min_distance = distance
		
		return nearest_index
		
	def distance_between_checkpoints(self, point1, point2):
		d_x = abs(point1[0] - point2[0])
		d_y = abs(point1[0] - point2[0])
		return math.sqrt(d_x**2 + d_y**2)
		
def main(args=None):
	rclpy.init(args=args)
	srv = Service()
	rclpy.spin(srv)
	srv.destroy_node()
	rclpy.shutdown()
if __name__ == '__main__':
	main()				
						
						
						
						
						
						
						
						
				
				
				
				
				
				
