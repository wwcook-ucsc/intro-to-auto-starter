#!/usr/bin/env python

# import the rospy api to access ros functionality
import rospy
# import the Pose message from the turtlesim packages' msg folder
from turtlesim.msg import Pose

def pose_callback(data):
    rospy.loginfo('x: %f, y: %f',data.x, data.y)


if __name__ == '__main__':
    # this is the "main" for python, and is what will be called when 
    # we run the node. Here we want to call the turtle pose 
    # listener.
    
     # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('turtle_listener', anonymous=True)

    # We create a new Subscriber instance to "subscribe" or listen to 
    # the data being published at the /turtle1/pose topic. The second
    # paramter indicates we want Pose messages. The last parameter
    # is what ROS should call if we get a message. In this case, 
    # we have passed our pos_callback function. So everytime we get 
    # a message on this topic, rospy will call pose_callback(data) with
    # data as the message. 
    rospy.Subscriber('/turtle1/pose', Pose, pose_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
