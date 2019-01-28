#!/usr/bin/env python
import rospy
import math
from math import *
import matplotlib.pyplot as plt
import time
import utm
import sys
from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Float64
from std_msgs.msg import Float64
from actionlib_msgs.msg import GoalStatusArray
from actionlib_msgs.msg import GoalStatus


class PID:

	def __init__(self):
		

		self.path_points = []
		self._i = 0
		self.final_goal = 0

		self._sub = rospy.Subscriber("/ada/fix", NavSatFix,self.call_gps)

		self._path = rospy.wait_for_message("route_points", Path)
		
		self.pub =  rospy.Publisher('ackermann_cmd', AckermannDrive, queue_size=10, latch=True)
		self.ackermann_msg = AckermannDrive()

		
		self.state_pub = rospy.Publisher('state',Float64 ,queue_size=10, latch=True)
		self.control_effort = rospy.Subscriber('control_effort', Float64, self.control_callback)

		self.goal_sub = rospy.Subscriber("ada/goal_status",GoalStatusArray,self.call_goal) 

		self.lateral_offset = 0.0
		self.prev_lateral_offset = 0.0
		self.yaw_error = 0.0
		self.Ld = 0.0
		self.Rd = 0.0
		self.dt = 0.01
		self.error = 0.0
		self.yaw_angle_vehicle = 0.0
		self.distance = 0.0
		self.i = 0
		self.current_pose = []
		self.prev_pose = []
		self.road_slope = None
		self.direction = 0.0
		self.prev_error = 0.0
		self.path_points = []
		self.Velocity_x = 10	
		# self.max_steering_angle = 

	def control_callback(self,control):

		steering_angle = control.data
		self.ackermann_msg.speed = self.Velocity_x
		self.ackermann_msg.steering_angle = steering_angle
		
		# if steering_angle > max_steering_angle:
		# 	steering_angle = max_steering_angle
		# elif steering_angle < - max_steering_angle:
		# 	steering_angle = - max_steering_angle

		self.pub.publish(self.ackermann_msg)
		rospy.loginfo(steering_angle)

		if self._i == len(self.path_points):
			self.control_effort.unregister()


	def call_goal(self,goal):
		self.final_goal = goal.status_list[len(goal.status_list)-1].status
		
			

	def get_path(self):

		for i in range (0,len(self._path.poses)):
			my_tuple = (self._path.poses[i].pose.position.x, self._path.poses[i].pose.position.y)
			plt.plot(my_tuple[0],my_tuple[1],'bo')
			self.path_points.append(my_tuple)




	def call_gps(self,gps):
		
		UTMx, UTMy, _, _ = utm.from_latlon(gps.latitude, gps.longitude)
		plt.plot(UTMx,UTMy,'ro')
		self.current_pose = [UTMx,UTMy]
		self.vehicle_position()
		self.prev_pose = self.current_pose

		if self.final_goal == 3:
			self.control_effort.unregister()
			plt.show()
			rospy.loginfo("reached final goal")
			self._sub.unregister()


	def vehicle_position(self):
		
		if self._i <= len(self.path_points)-1:

			if len(self.prev_pose) > 0 :

				dist_vehicle_pose = sqrt((self.path_points[self._i][0] - self.current_pose[0])**2 + (self.path_points[self._i][1] - self.current_pose[1])**2)
				dist_prev_vehicle_pose = sqrt((self.path_points[self._i][1] - self.prev_pose[1])**2+(self.path_points[self._i][0] - self.prev_pose[0])**2)
				
				if dist_vehicle_pose > dist_prev_vehicle_pose:
					self._i = self._i + 1
					rospy.loginfo(self._i)
					plt.plot(self.current_pose[0],self.current_pose[1],'*')

				if self._i == len(self.path_points):
					sys.exit()
			
			self.calc_error()
			# self.steering_angle_rate()





	def distance_center_lane(self) :
		# this function calculate the distance of the vehicle from the center of the lane
	
		if self._i == 0:
			
			way_point_1 = (self.path_points[0][0],self.path_points[0][1])
			way_point_2 = (self.path_points[1][0],self.path_points[1][1])
			gps = (self.current_pose[0],self.current_pose[1])

			self.distance = abs((way_point_1[1] - way_point_2[1])*gps[0] + (way_point_2[0] - way_point_1[0])*gps[1] + (way_point_1[0]*way_point_2[1] - way_point_2[0]*way_point_1[1])) / sqrt((way_point_1[1] - way_point_2[1])**2 + (way_point_2[0] - way_point_1[0])**2)
	
		else:

			way_point_1 = (self.path_points[self._i-1][0],self.path_points[self._i-1][1])
			way_point_2 = (self.path_points[self._i][0],self.path_points[self._i][1])
			gps = (self.current_pose[0],self.current_pose[1])
			
			self.distance = abs((way_point_1[1] - way_point_2[1])*gps[0] + (way_point_2[0] - way_point_1[0])*gps[1] + (way_point_1[0]*way_point_2[1] - way_point_2[0]*way_point_1[1])) / sqrt((way_point_1[1] - way_point_2[1])**2 + (way_point_2[0] - way_point_1[0])**2)


		direction = self.vehicle_lane_position()

		self.distance *= direction

		return self.distance



	def vehicle_lane_position(self):
		#this function return the vehicle position from the lane left or right the lane
		# pos = (x-x1)(y2-y1) - (y-y1)(x2-x1)
		if self._i == 0:
			pos = ((self.current_pose[0] - self.path_points[self._i][0])*(self.path_points[self._i + 1][1] - self.path_points[self._i][1])) - ((self.current_pose[1] - self.path_points[self._i][1])*(self.path_points[self._i + 1][0] - self.path_points[self._i][0]))
		else:
			pos = ((self.current_pose[0] - self.path_points[self._i-1][0])*(self.path_points[self._i][1] - self.path_points[self._i-1][1])) - ((self.current_pose[1] - self.path_points[self._i-1][1])*(self.path_points[self._i][0] - self.path_points[self._i-1][0]))

		if pos > 0: #right
			rospy.loginfo("I am right")
			self.Rd = 1.5 - self.distance
			self.Ld = 1.5 + self.distance
			self.direction = -1
		elif pos < 0: #left
			rospy.loginfo("I am left")
			self.Rd = 1.5 + self.distance
			self.Ld = 1.5 - self.distance
			self.direction = 1

		return self.direction


	def vehicle_yaw_angle(self):

		if len(self.prev_pose) == 0 :  #angle of the vehicle in the starting position
			yaw_angle_vehicle = math.degrees(math.atan2(self.current_pose[1] , self.current_pose[0]))
		else:    #angle of the vehicle after it starts moving
			x_v = self.current_pose[0] - self.prev_pose[0]
			y_v = self.current_pose[1] - self.prev_pose[1]
			yaw_angle_vehicle = math.atan(y_v/x_v)

		return yaw_angle_vehicle

	

	def road_yaw_angle(self):

		current_path_point = self.path_points[self._i]
		prev_path_point = self.path_points[self._i - 1]
				
		if self._i == 0:  
			self.road_slope = (self.path_points[self._i+1][1] - self.path_points[self._i][1]) / (self.path_points[self._i+1][0] - self.path_points[self._i][0])

		else:
			self.road_slope = (current_path_point[1] - prev_path_point[1]) / (current_path_point[0] - prev_path_point[0])
		
		yaw_angle_road = math.atan(self.road_slope)
		
		return yaw_angle_road

	

	def vehicle_yaw_error(self):

		yaw_angle_road = self.road_yaw_angle()
		yaw_angle_vehicle = self.vehicle_yaw_angle()

		self.yaw_error = abs(yaw_angle_road) - abs(yaw_angle_vehicle)
		# self.yaw_error =  abs(yaw_angle_vehicle) - abs(math.atan(self.road_slope))
		
		return self.yaw_error


	def calc_error(self):

		self.lateral_offset = self.distance_center_lane()

		if self.i == 0:
			error = self.lateral_offset

		else:
			self.yaw_error = self.vehicle_yaw_error()
			error = self.lateral_offset + self.Velocity_x * self.yaw_error * self.dt

		error *= -1
		
		self.state_pub.publish(error)
		self.i += 1
		self.steering_angle_rate()



	def radius_curvature(self):

		try:
			a =  sqrt((self.path_points[self._i +1][0] - self.path_points[self._i][0])**2 + (self.path_points[self._i+1][1] - self.path_points[self._i][1])**2)

			b =  sqrt((self.path_points[self._i+2][0] - self.path_points[self._i+1][0])**2 + (self.path_points[self._i+2][1] - self.path_points[self._i+1][1])**2)

			c =  sqrt((self.path_points[self._i +2][0] - self.path_points[self._i][0])**2 + (self.path_points[self._i+2][1] - self.path_points[self._i][1])**2)
			
			radius = (a*b*c )/ sqrt((a + b +c)*(b + c - a)*(c + a - b)*(a + b - c))
			
			print("radius is" + str(radius))
			
			return radius
		except:
			return None

	def steering_angle_rate(self):
		
		Lf = 1.8
		Lr = 1.6
		r = self.radius_curvature()

		if r !=	 None:
			k = 1/r
			steering_angle = math.atan(((Lf + Lr) / Lr)*math.tan(math.asin(Lr * k)))
			steering_angle_rate = steering_angle / self.dt

			self.ackermann_msg. steering_angle_velocity = steering_angle_rate

			return steering_angle_rate



if __name__ == '__main__':
	try:
		rospy.init_node('PID', anonymous=True)
		steering_control = PID()
		steering_control.get_path()
		rospy.spin()

	except rospy.ROSInterruptException:
		print("exception occured")	
    
