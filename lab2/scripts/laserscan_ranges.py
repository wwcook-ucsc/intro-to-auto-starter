#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import String, Float32
from sensor_msgs.msg import LaserScan
from lab2.msg import ScanRange


class LaserScanNode():
    def __init__(self):
        self.publishing = False
        
        # initialize and register this node with the ROS master node
        rospy.init_node('laserscan_listen', anonymous=False)
        
        # Create an instance of the scan range message to store your closest and farthest
        # points. Remember, these attributes can be accesed with:
        # self.scan_range.closest_point. Also note the attributes are set to default values
        # currently.
       	self.scan_range = ScanRange()
        
        # create a publisher handle to publish ScanRange messages to the 
        # laserscan_range topic. Make sure you use self.var_name = xyz so
	    # that you are able to use this handle in your other functions
       	self.laserscan_range_pub = rospy.Publisher('/laserscan_ranges',
                                                   ScanRange,
                                                   queue_size=10)
        
        # subscriber handle for the scan message. This handle 
        # will subscribe to scan and recieve LaserScan messages. Each
        # time this happens the scan_callback function is called
        rospy.Subscriber('scan', LaserScan, self.scan_callback)
    
    # this is the callback for the scan message. 
    # here we will use the scan_data parameter to access the ranges 
    # from the LaserScan message and figure out the closest and farthest point
    def scan_callback(self, scan_data):
        # Write code to loop through the laser scan ranges and find the closest
        # and farthest values. Store those values in the ScanRange instance you created
        # in __init__()

        if not self.publishing:
            return

        nearest_distance = scan_data.range_max + 1.0
        nearest_i = -1
        farthest_distance = scan_data.range_min - 1.0
        farthest_i = -1
        for i, distance in enumerate(scan_data.ranges):
            if distance < scan_data.range_min:
                continue
            if distance > scan_data.range_max:
                continue
            if math.isnan(distance):
                continue
            if math.isinf(distance):
                continue
            if distance < nearest_distance:
                nearest_distance = distance
                nearest_i = i
            if distance > farthest_distance:
                farthest_distance = distance
                farthest_i = i

        if nearest_distance < scan_data.range_min:
            return
        if farthest_distance > scan_data.range_max:
            return

        nearest_angle = scan_data.angle_min + float(nearest_i) * scan_data.angle_increment
        farthest_angle = scan_data.angle_min + float(farthest_i) * scan_data.angle_increment

        self.scan_range.nearest_distance = nearest_distance
        self.scan_range.nearest_angle = nearest_angle
        self.scan_range.farthest_distance = farthest_distance
        self.scan_range.farthest_angle = farthest_angle

        self.laserscan_range_pub.publish(self.scan_range)
        

if __name__ == '__main__':
    try:
        ls = LaserScanNode()
        ls.publishing = True
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
