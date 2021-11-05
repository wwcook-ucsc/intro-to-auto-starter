#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from lab4.msg import PIDInput
from math import pi, cos, sin, atan2


class DistFinder(object):

    TARGET_LEFT = 0
    TARGET_RIGHT = 1
    TARGET_BOTH = 2

    def __init__(self, lookahead_theta, target_side, target_distance=None):
        """Node that publishes wall following error

        :param lookahead_theta: secondary angle to look ahead of orthogonal to the car
        :param target_side: one of TARGET_LEFT, TARGET_RIGHT, or TARGET_BOTH
        :param target_distance: distance to maintain from the wall if TARGET_LEFT or TARGET_RIGHT
        """
        if target_side == self.TARGET_LEFT or target_side == self.TARGET_RIGHT:
            if target_distance is None:
                raise ValueError('target_distance must be specified for single-sided wall following')
        elif target_side == self.TARGET_BOTH:
            if target_distance is not None:
                raise ValueError('target_distance cannot be specified for dual-sided wall following')
        else:
            raise ValueError('invalid value for target_side: {}'.format(target_side))
        self._theta = lookahead_theta
        self._target_side = target_side
        self._target_distance = target_distance

        rospy.init_node('dist_finder')

        self.pid_input_msg = PIDInput()
        self.pid_input_publisher = rospy.Publisher('/pid_input', PIDInput, queue_size=10)

        self.scan_subscriber = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        if target_side == self.TARGET_LEFT:
            info = 'dist_finder running left wall following at distance %.3f' % target_distance
        elif target_side == self.TARGET_RIGHT:
            info = 'dist_finder running right wall following at distance %.3f' % target_distance
        elif target_side == self.TARGET_BOTH:
            info = 'dist_finder running balanced wall following'
        rospy.loginfo(info)
        rospy.spin()
    
    def scan_callback(self, data):
        error = 0.0
        if self._target_side in [self.TARGET_LEFT, self.TARGET_BOTH]:
            ortho_range = self.angle2range(data, pi / 2)
            ahead_range = self.angle2range(data, pi / 2 - self._theta)
            distance = self.calculate_distance(ortho_range, ahead_range, self._theta, 0.0)
            error += distance
            if self._target_distance is not None:
                error -= self._target_distance
        if self._target_side in [self.TARGET_RIGHT, self.TARGET_BOTH]:
            ortho_range = self.angle2range(data, -pi / 2)
            ahead_range = self.angle2range(data, -pi / 2 + self._theta)
            distance = self.calculate_distance(ortho_range, ahead_range, self._theta, 0.0)
            error -= distance
            if self._target_distance is not None:
                error += self._target_distance
        if self._target_side == self.TARGET_BOTH:
            error /= 2
        self.pid_input_msg.error = error
        self.pid_input_publisher.publish(self.pid_input_msg)
    
    def calculate_distance(self, ortho_range, ahead_range, theta, project_distance):
        """Calculates the distance to the wall.

        This function can be used to calculate the distance to the left or right wall

        :param ortho_range: range in a direction perpendicular to the car
        :param ahead_range: range in a direction ahead of the car
        :param theta: angle between the two ranges
        :param project_distance: distance to move the car forward when projecting to compensate for delay
        """
        alpha = atan2(ahead_range * cos(theta) - ortho_range,
                      ahead_range * sin(theta))
        distance = ortho_range * cos(alpha)
        distance += project_distance * sin(alpha)
        return distance

    def angle2range(self, laser_scan, angle):
        """Extracts the range at the given angle from the LaserScan message

        :param laser_scan: a LaserScan message
        :param angle: angle in radians
        """
        i = int(round((angle - laser_scan.angle_min) / laser_scan.angle_increment))
        return laser_scan.ranges[i]


if __name__ == '__main__':
    try:
        DistFinder(30 * pi / 180, DistFinder.TARGET_BOTH)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        rospy.logerr(e)
    finally:
        rospy.loginfo('dist_finder shut down')
