#!/usr/bin/env python

import roslib, rospy
import copy
import numpy as np
import os, sys

from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import PointCloud

import DiscreteTimeKalmanFilter
import imp

import time

from optparse import OptionParser

class DataAssociator(object):
    def __init__(self, kalman_parameters_filename):
        ''' 
        First: Load the kalman parameters from the filename. Note: in this particular case, you could have used "import kalman_parameters.py", because that file is in the same directory as this python script. Using imp makes it easy to load a python module from anywhere on the computer. Best programming practice would probably be to load these parameters from a generic file type, such as a yaml file, however, it dramatically simplifies the code by using a python file, which helps keep everything more transparent and digestible. 
        '''
        self.kalman_parameters = imp.load_source('kalman_parameters', kalman_parameters_filename)
        
        # Initialize the kalman filter
        self.kalman_filter = DiscreteTimeKalmanFilter.DiscreteKalmanFilter(     x0      = None, 
                                                                                P0      = self.kalman_parameters.P0, 
                                                                                phi     = self.kalman_parameters.phi, 
                                                                                gamma   = self.kalman_parameters.gamma, 
                                                                                H       = self.kalman_parameters.H, 
                                                                                Q       = self.kalman_parameters.Q, 
                                                                                R       = self.kalman_parameters.R, 
                                                                                gammaW  = self.kalman_parameters.gammaW,
                                                                                )
        
        # Initialize the node
        rospy.init_node('data_associator')
        self.time_start = time.time()
        
        # Publishers.
        self.pubTrackedObjects_Float32MultiArray  = rospy.Publisher('/tracking_template/kalman_filtered_points', Float32MultiArray)
        self.pubTrackedObjects_Pose               = rospy.Publisher('/tracking_template/kalman_filtered_pose', PoseWithCovarianceStamped)
        self.pubTrackedObjects_Twist              = rospy.Publisher('/tracking_template/kalman_filtered_twist', TwistWithCovarianceStamped)
        
        # Subscriptions.
        self.subImage = rospy.Subscriber('/tracking_template/tracked_points', PointCloud, self.point_tracker)
        
    def point_tracker(self, pointcloud):
        # Assume there there is, at most, 1 object (Bonus: get code to work for multiple objects! Hint: use a new kalman filter for each object)
        if len(pointcloud.points) > 0:
            point = pointcloud.points[0]
            measurement = np.matrix([point.x, point.y]).T # note: the shape is critical; this is a matrix, not an array (so that matrix mult. works)
        else:
            measurement = None    
            nmeasurements = 2
            
        if measurement is None: # propagate a blank measurement
            m = np.matrix([np.nan for i in range(2) ]).T
            xhat, P, K = self.kalman_filter.update( None )
        else:                   # propagate the measurement
            xhat, P, K = self.kalman_filter.update( measurement ) # run kalman filter
        
        ### Publish the results as a simple array
        float_time = pointcloud.header.stamp.secs + pointcloud.header.stamp.nsecs*1e-9
        x = xhat.item(0) # xhat is a matrix, .item() gives you the actual value
        xdot = xhat.item(1)
        y = xhat.item(2)
        ydot = xhat.item(3)
        p_vector = P.reshape(16).tolist()[0]
        data = [float_time, x, xdot, y, ydot]
        data.extend(p_vector)
        
        float32msg = Float32MultiArray()
        float32msg.data = data
        self.pubTrackedObjects_Float32MultiArray.publish( float32msg )
        ###
        
        ### Publish the results as a ROS type pose (positional information)
        # see: http://docs.ros.org/jade/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html
        pose = PoseWithCovarianceStamped()
        pose.header = pointcloud.header
        pose.pose.pose.position.x = x
        pose.pose.pose.position.y = y
        pose.pose.pose.position.z = 0
        pose.pose.pose.orientation.x = 0
        pose.pose.pose.orientation.y = 0
        pose.pose.pose.orientation.z = 0
        pose.pose.pose.orientation.w = 0
        x_x = P[0,0]
        x_y = P[0,2]
        y_y = P[2,2]
        position_covariance = [x_x, x_y, 0, 0, 0, 0, x_y, y_y, 0, 0, 0, 0]
        position_covariance.extend([0]*24)
        # position_covariance is the row-major representation of a 6x6 covariance matrix
        pose.pose.covariance = position_covariance
        self.pubTrackedObjects_Pose.publish( pose )
        ###
        
        ### Publish the results as a ROS type twist (velocity information)
        # see: http://docs.ros.org/jade/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html
        twist = TwistWithCovarianceStamped()
        twist.header = pointcloud.header
        twist.twist.twist.linear.x = xdot
        twist.twist.twist.linear.y = ydot
        twist.twist.twist.linear.z = 0
        twist.twist.twist.angular.x = 0
        twist.twist.twist.angular.y = 0
        twist.twist.twist.angular.z = 0
        dx_dx = P[1,1]
        dx_dy = P[1,3]
        dy_dy = P[3,3]
        velocity_covariance = [x_x, x_y, 0, 0, 0, 0, x_y, y_y, 0, 0, 0, 0]
        velocity_covariance.extend([0]*24)
        # position_covariance is the row-major representation of a 6x6 covariance matrix
        twist.twist.covariance = velocity_covariance
        self.pubTrackedObjects_Twist.publish( twist )
        ###
        
        
    def main(self):
        while not rospy.is_shutdown():
            rospy.spin()            
            
                
if __name__ == '__main__':
    parser = OptionParser()
    parser.add_option("--kalman_parameters_filename", type="str", dest="kalman_parameters_filename", default='none',
                        help="name and path of kalman_parameters_filename")
    (options, args) = parser.parse_args()
    
    data_associator = DataAssociator(options.kalman_parameters_filename)
    data_associator.main()
