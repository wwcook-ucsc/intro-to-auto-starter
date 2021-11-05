#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from lab4.msg import PIDInput
from ackermann_msgs.msg import AckermannDriveStamped


class CarControl(object):

    def __init__(self, Kp, Kd, speed):
        self._Kp = Kp
        self._Kd = Kd
        self._speed = speed

        self._last_error = None
        self._last_time = None

        rospy.init_node('car_control')

        self.drive_msg = AckermannDriveStamped()
        self.drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=10)

        self.pid_input_sub = rospy.Subscriber('pid_input', PIDInput, self.pid_input_callback)

        rospy.loginfo('car_control is running')

        rospy.spin()

    def pid_input_callback(self, data):
        time = rospy.get_time()
        if self._last_error is not None:
            d_error = (data.error - self._last_error) / (time - self._last_time)
            u = self._Kp * (data.error + self._Kd * d_error)
            self.drive_msg.drive.steering_angle = u
            self.drive_msg.drive.speed = self._speed
            self.drive_pub.publish(self.drive_msg)
#            rospy.loginfo('d_error: {:10.3f}'.format(d_error))
        self._last_error = data.error
        self._last_time = time


if __name__ == '__main__':
    CarControl(2.2, .001, 2)
