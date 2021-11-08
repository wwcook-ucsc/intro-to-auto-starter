#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from lab4.msg import PIDInput
from ackermann_msgs.msg import AckermannDriveStamped
from math import exp


def clipped_linear(x, x0, y0, x1, y1):
    """mx+b but clipped

    :param x: current input
    :param x0: minimum clipping input
    :param y0: minimum clipped output
    :param x1: maximum clipping input
    :param y1: maximum clipped output
    """
    if x <= x0:
        return y0
    elif x >= x1:
        return y1
    else:
        return (y1 - y0) / (x1 - x0) * (x - x0) + y0


class CarControl(object):

    def __init__(self, Kp, Kd, min_speed, max_speed, d_error_alpha):
        rospy.init_node('car_control')

        self.Kp = Kp
        self.Kd = Kd
        self.min_speed = min_speed
        self.max_speed = max_speed
        self._d_error_alpha = d_error_alpha

        self._last_error = None
        self._last_time = None
        self._smoothed_error = None
        self._smoothed_d_error = None
        self._smoothed_period = None
        self._front_distance = 0.0
        self._smoothed_front_distance = 0.0
        self._speed = 0.0

        self.drive_msg = AckermannDriveStamped()
        self.drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=10)

        self.pid_input_sub = rospy.Subscriber('pid_input', PIDInput, self.pid_input_callback)

        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        rospy.loginfo('car_control is running')

        rospy.spin()

    def pid_input_callback(self, data):
        time = rospy.get_time()
        if self._smoothed_error is not None and self._smoothed_period is not None:
            update_rate = 100.  # Hz
            alpha = (1. - exp(-self._smoothed_period * update_rate))
            self._smoothed_error += alpha * (data.error - self._smoothed_error)
        else:
            self._smoothed_error = data.error
        if self._last_error is not None:
            # estimate time between calls for time derivatives
            # otherwise derivative is very noisy.
            # use an exponential running average, where alpha is actually
            # computed dynamically
            curr_period = time - self._last_time
            if self._smoothed_period is not None:
                alpha = (1. - exp(-self._smoothed_period))
                self._smoothed_period += alpha * (curr_period - self._smoothed_period)
            else:
                self._smoothed_period = curr_period

            # estimate derivative of error
            d_error = (data.error - self._last_error) / self._smoothed_period
            if self._smoothed_d_error is not None:
                update_rate = 20.  # Hz
                alpha = (1. - exp(-self._smoothed_period * update_rate))
                self._smoothed_d_error += alpha * (d_error - self._smoothed_d_error)
            else:
                self._smoothed_d_error = d_error

            if self._last_time is not None and int(time*10.) > int(self._last_time*10.):
                rospy.loginfo('d_error: {:10.4f}'.format(d_error))
                rospy.loginfo('\tdelta_error: {:10.4f}'.format(d_error * self._smoothed_period))
                rospy.loginfo('smoothed_d_error: {:10.4f}'.format(self._smoothed_d_error))

            Kp = self.Kp
            # if self._speed > 3.:
            #     Kp /= self._speed * 3.0
            u = Kp * (self._smoothed_error + self.Kd * self._smoothed_d_error)
            self.drive_msg.drive.steering_angle = u
            # self.drive_msg.drive.speed = clipped_linear(
            #     min(self._front_distance, self._smoothed_front_distance),
            #     .5,  # distance at which to travel minimum speed
            #     self.min_speed,  # minimum speed
            #     8.,  # distance at which to travel maximum speed
            #     self.max_speed,  # maximum speed
            # )
            self.drive_msg.drive.speed = (self.min_speed + self.max_speed) * .5
            self.drive_pub.publish(self.drive_msg)
        self._last_error = data.error
        self._last_time = time

    def angle2range(self, laser_scan, angle):
        """Extracts the range at the given angle from the LaserScan message

        :param laser_scan: a LaserScan message
        :param angle: angle in radians
        """
        i = int(round((angle - laser_scan.angle_min) / laser_scan.angle_increment))
        return laser_scan.ranges[i]
    
    def scan_callback(self, data):
        self._front_distance = self.angle2range(data, 0)
        update_rate = 1.  # Hz
        alpha = (1. - exp(-self._smoothed_period * update_rate))
        td = (self._front_distance - self._smoothed_front_distance)
        self._smoothed_front_distance += alpha * td
    
    def odom_callback(self, data):
        self._speed = data.twist.twist.linear.x


if __name__ == '__main__':
    CarControl(
#        Kp=1.2,  # single-sided wall following
#        Kd=.008,  # single-sided wall following
        Kp=1.8,  # double-sided wall following
        Kd=.03,  # double-sided wall following
        min_speed=1.0,
        max_speed=3.0,
        d_error_alpha=.1,
    )
