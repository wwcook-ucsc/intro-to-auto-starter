#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import String, Float32
from sensor_msgs.msg import LaserScan
from lab2.msg import ScanRange


class LaserScanNode():
    def __init__(self):
        
        # create a publisher handle to publish ScanRange messages to the 
        # laserscan_range topic. Make sure you use self.var_name = xyz so
	# that you are able to use this handle in your other functions
       	"""ADD PUBLISHER HANDLER HERE""""
        
        # subscriber handle for the scan message. This handle 
        # will subscribe to scan and recieve LaserScan messages. Each
        # time this happens the scan_callback function is called
        rospy.Subscriber('scan', LaserScan, self.scan_callback)
        
        # initialize and register this node with the ROS master node
        rospy.init_node('laserscan_listen', anonymous=False)
        
        # Create an instance of the scan range message to store your closest and farthest
        # points. Remember, these attributes can be accesed with:
        # self.scan_range.closest_point. Also note the attributes are set to default values
        # currently.
       	""" CREATE INSTANCE OF MESSAGE HERE"""
    
    # this is the callback for the scan message. 
    # here we will use the scan_data parameter to access the ranges 
    # from the LaserScan message and figure out the closest and farthest point
    def scan_callback(self, scan_data):
        """ADD CALLBACK CODE HERE"""
        pass

    # the publish method is  called on an interval in the main
    # loop of this file. This is where we publish our ranges to 
    # the topic. 
    def publish(self):
        """ADD PUBLISHER CODE HERE"""
        pass
        

if __name__ == '__main__':
    ls = LaserScanNode()

    rate = rospy.Rate(10) # 10hz
    try:
	# Add a while loop which calls the publish() function of the laser scan node
	# on the interval we have defined
	""" CREATE PUBLISHER LOOP HERE """

    except rospy.ROSInterruptException:
        pass
