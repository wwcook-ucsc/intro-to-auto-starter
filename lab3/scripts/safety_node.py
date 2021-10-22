#!/usr/bin/env python
import rospy
import math
from math import sin, cos

# import ROS msg types and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool


class Safety(object):
    """
    The class that handles emergency braking.
    """
    def __init__(self, ttc_threshold):
        """
        One publisher should publish to the /brake topic with a AckermannDriveStamped brake message.
        One publisher should publish to the /brake_bool topic with a Bool message.
        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /odom topic to get the current speed of the vehicle.
        The subscribers should use the provided odom_callback and scan_callback as callback methods
        NOTE that the x component of the linear velocity in odom is the speed
        """
        rospy.init_node('safety_node')

        rospy.on_shutdown(self.shutdown)

        self.ttc_threshold = ttc_threshold

        # set up message data variables
        self.velocity = None
        self.ranges = None
        self.angle_min = None
        self.angle_increment = None
        self.range_min = None
        self.range_max = None

        # create ROS subscribers and publishers.
        self.brake_msg = AckermannDriveStamped()
        self.brake_pub = rospy.Publisher('/brake', AckermannDriveStamped, queue_size=10)
        self.brake_bool_msg = Bool()
        self.brake_bool_msg.data = True
        self.brake_bool_pub = rospy.Publisher('/brake_bool', Bool, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        rate = rospy.Rate(10)

        rospy.loginfo('emergency braking system is running')

        while not rospy.is_shutdown():
            # just make sure the /brake topic is warm
            # might not be necessary but better safe than crashed
            self.brake_pub.publish(self.brake_msg)

            if self.velocity is not None and self.ranges is not None:
                xvel, yvel = self.velocity['x'], self.velocity['y']
                ranges = list(self.ranges)
                angle_min = self.angle_min
                angle_increment = self.angle_increment
                self.velocity = None
                self.ranges = None
                for i, distance in enumerate(ranges):
                    # make sure distance is a valid range
                    if math.isinf(distance) or math.isnan(distance):
                        continue
                    if distance < self.range_min:
                        # choose to brake when the sensor says something is
                        # invalidly close
                        self.brake()
                    distance = min(distance, self.range_max)
                    # project velocity onto range angle
                    angle = angle_min + i * angle_increment
                    proj_vel = xvel * cos(angle) + yvel * sin(angle)
                    if proj_vel <= 0.0:
                        continue

                    ttc = distance / proj_vel
                    if ttc < self.ttc_threshold:
#                        print('TTC {} at angle {}'.format(ttc, angle * 180. / math.pi))
                        self.brake()
            rate.sleep()

    def odom_callback(self, odom_msg):
        # update current speed
        self.velocity = {
            'x': odom_msg.twist.twist.linear.x,
            'y': odom_msg.twist.twist.linear.y,
        }

    def scan_callback(self, scan_msg):
        # update current ranges
        self.ranges = list(scan_msg.ranges)
        self.angle_min = scan_msg.angle_min
        self.angle_increment = scan_msg.angle_increment
        self.range_min = scan_msg.range_min
        self.range_max = scan_msg.range_max
    
    def brake(self):
        # publish brake message
        rospy.loginfo('BRAKE!!')
        self.brake_bool_pub.publish(self.brake_bool_msg)
    
    def shutdown(self):
        rospy.loginfo('emergency braking system is shutting down')


if __name__ == '__main__':
    try:
        Safety(ttc_threshold=0.3)
    except Exception as e:
        rospy.logerr('emergency braking system is terminating')
        rospy.logerr(e)
    except:
        rospy.logerr('emergency braking system is terminating')
