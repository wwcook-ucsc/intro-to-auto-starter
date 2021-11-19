#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from lab5.msg import PidInput
import numpy as np


class GapFinder(object):

    def __init__(self, car_width, disparity_threshold, fov=270, min_fov=120, angle_rate=1.0, blinder_amount=0.0):
        """
        :param car_width: width of car, used for dilating obstacles
        :param disparity_threshold: difference between consecutive ranges to
                                    consider a disparity
        :param fov: LiDAR field of view to keep. None for default fov given by /scan.
        :param min_fov: minimum LiDAR field of view at all times
        :param blinder_amount: amount of erosion to apply to FOV when turning
        """
        self.half_width = car_width * 0.5
        self.disp_thresh = disparity_threshold
        self.fov = fov * np.pi / 180.0
        self.min_fov = min_fov * np.pi / 180.0
        self.blinder_amount = blinder_amount

        self.avg_ang_vel = 0.0
        self.ang_vel_rate = angle_rate

        rospy.init_node('gap_finder')

        self.pid_input_msg = PidInput()
        self.pid_input_publisher = rospy.Publisher('/pid_input', PidInput, queue_size=10)

        self.filtered_scan_publisher = rospy.Publisher('/filtered_scan', LaserScan, queue_size=10)

        self.scan = None
        self.scan_subscriber = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        rospy.loginfo('gap_finder running')

        self.rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if self.scan is None:
                self.rate.sleep()
                continue

            data = self.get_latest_scan()
            self.run_disparity_loop(data)

            self.rate.sleep()
    
    def scan_callback(self, data):
        self.scan = data
    
    def odom_callback(self, data):
        z = data.twist.twist.angular.z
        self.avg_ang_vel += self.ang_vel_rate * (z - self.avg_ang_vel)
    
    def get_latest_scan(self):
        """Copy scan data fast to reduce races
        """
        data = LaserScan()

        data.header.seq = self.scan.header.seq
        data.header.stamp = self.scan.header.stamp
        data.header.frame_id = self.scan.header.frame_id
        data.angle_min = self.scan.angle_min
        data.angle_max = self.scan.angle_max
        data.angle_increment = self.scan.angle_increment
        data.time_increment = self.scan.time_increment
        data.scan_time = self.scan.scan_time
        data.range_min = self.scan.range_min
        data.range_max = self.scan.range_max
        data.ranges = list(self.scan.ranges)
        data.intensities = list(self.scan.intensities)

        return data
    
    def run_disparity_loop(self, data):
        ranges = np.array(self.scan.ranges, dtype='float32')

        angle_min, angle_inc = data.angle_min, data.angle_increment
        filtered_ranges = self.erode_gaps(ranges, angle_inc)

        # data.ranges = list(filtered_ranges)
        # self.filtered_scan_publisher.publish(data)

        # print('angle_min, angle_inc:', (angle_min, angle_inc))

        new_angle_min = -0.5 * self.fov
        new_angle_max = 0.5 * self.fov
        if self.avg_ang_vel >= 0.0:  # turning left
            new_angle_max -= self.avg_ang_vel * self.blinder_amount
            new_angle_max = max(0.5 * self.min_fov, new_angle_max)
        else:  # turning right
            new_angle_min -= self.avg_ang_vel * self.blinder_amount
            new_angle_min = min(-0.5 * self.min_fov, new_angle_min)
#        print('fov min, max:', (new_angle_min, new_angle_max))
        filtered_ranges, angle_min, angle_max = self.limit_fov(
            filtered_ranges, new_angle_min, new_angle_max, angle_min, angle_inc)
        # print('new_angle_min, new_angle_inc:', (angle_min, angle_inc))
        target_range_i = np.argmax(filtered_ranges)
        target_angle = self.index2angle(target_range_i, angle_min, angle_inc)
        # print('target angle, distance:', (target_angle * 180. / np.pi,
        #                                   filtered_ranges[target_range_i]))

        self.pid_input_msg.steering_error = target_angle
        self.pid_input_msg.target_speed = 2.5
        self.pid_input_publisher.publish(self.pid_input_msg)

        data.ranges = list(filtered_ranges)
        data.angle_min = angle_min
        data.angle_increment = angle_inc
        data.angle_max = angle_min + angle_inc * len(data.ranges)
        self.filtered_scan_publisher.publish(data)
    
    def erode_gaps(self, ranges, angle_inc):
        diffs = np.abs(np.diff(ranges))
#        print('min, max, mean diff:', (diffs.min(), diffs.max(), diffs.mean()))
        disp_i = np.arange(len(diffs))[diffs >= self.disp_thresh]
        dists = np.minimum(ranges[disp_i],
                           ranges[disp_i + 1])
        filtered_ranges = ranges.copy()
        for i, dist in zip(disp_i, dists):
            half_window_size_rads = self.half_width / dist
            half_window_size = half_window_size_rads / angle_inc
            half_window_size = int(np.ceil(half_window_size))
            min_i = i - half_window_size
            max_i = i + half_window_size + 2
            filtered_ranges[min_i:max_i] = np.minimum(
                filtered_ranges[min_i:max_i],
                dist,
            )
        return np.minimum(filtered_ranges, ranges)

    def limit_fov(self, ranges, target_angle_min, target_angle_max, angle_min, angle_inc):
        """Trims the ranges array to have the desired min and max angles.
        """
        min_i = int(round(self.angle2index(target_angle_min, angle_min, angle_inc)))
        max_i = int(round(self.angle2index(target_angle_max, angle_min, angle_inc)))
        angle_min = self.index2angle(min_i, angle_min, angle_inc)
        angle_max = self.index2angle(max_i, angle_min, angle_inc)
        return ranges[min_i:max_i], angle_min, angle_max
    
    def angle2index(self, theta, angle_min, angle_increment):
        return (theta - angle_min) / angle_increment
    
    def index2angle(self, i, angle_min, angle_increment):
        return angle_min + i * angle_increment


if __name__ == '__main__':
    try:
        GapFinder(
            car_width=1.5,
            disparity_threshold=0.5,
#            disparity_threshold=0.0,  # dilates all obstacles. do not use with blinder
            fov=210.0,
            min_fov=150.0,
            angle_rate=0.02,
            blinder_amount=0.5,
        )
    except KeyboardInterrupt:
        pass
    except Exception as e:
        rospy.logerr(e)
    finally:
        rospy.loginfo('gap_finder shut down')
