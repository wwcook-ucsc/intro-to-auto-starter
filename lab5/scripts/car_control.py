#!/usr/bin/env python
import rospy
from lab5.msg import PidInput
from ackermann_msgs.msg import AckermannDriveStamped


class CarControl(object):

    def __init__(self, Kp, Kd):
        rospy.init_node('car_control')

        self.Kp = Kp
        self.Kd = Kd

        self._last_error = None

        self.drive_msg = AckermannDriveStamped()
        self.drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=10)

        self.pid_input_sub = rospy.Subscriber('pid_input', PidInput, self.pid_input_callback)

        rospy.loginfo('car_control is running')

        rospy.spin()

    def pid_input_callback(self, data):
        if self._last_error is not None:
            d_error = data.steering_error - self._last_error
            steering_angle = self.Kp * (data.steering_error + self.Kd * d_error)
        else:
            steering_angle = self.Kp * data.steering_error
        self.drive_msg.drive.steering_angle = steering_angle
        self.drive_msg.drive.speed = data.target_speed
        self.drive_pub.publish(self.drive_msg)


if __name__ == '__main__':
    try:
        CarControl(
            Kp=1.0,
            Kd=1.0,
        )
    except KeyboardInterrupt:
        pass
    except Exception as e:
        rospy.logerr(e)
    finally:
        rospy.loginfo('car_control shut down')
