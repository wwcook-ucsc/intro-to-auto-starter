#!/usr/bin/python
import rospy
from lab2_prelab.msg import Greeting


class HelloNode(object):

    def __init__(self, hello_name):
        rospy.init_node('hello_node', anonymous=False)
        self.hello_pub = rospy.Publisher('say_hello', Greeting, queue_size=10)

        rate = rospy.Rate(10)

        greeting_msg = Greeting()
        while not rospy.is_shutdown():
            greeting_msg.my_greeting = 'Hello, {}! ({})'.format(hello_name, rospy.get_time())
            self.hello_pub.publish(greeting_msg)
            rate.sleep()


if __name__ == '__main__':
    try:
        HelloNode('World')
    except Exception as e:
        rospy.logerr(e)
    except:
        pass
