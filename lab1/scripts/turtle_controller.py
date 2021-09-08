#!/usr/bin/env python
import rospy
from math import atan2, cos, pi, sin, sqrt
# remember the Twist message from the prelab?
# here we import it from geometry_msgs to gain
# access to the data structure. 
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

# This is a python class 
class TurtleController():
    # This is the class constructor. When this class is instantiated,
    # this function will be called automatically. __init__ is 
    # a reserved method in python classes dedicated to being
    # the contructor. In this function we initialize 
    # attributes of the class and setup other important structures. 
    # The 'self' keyword allows us to create variables inside our
    # class scope, so all of our functions are able to access them. We 
    # specify 'self' as a parameter to all our functions to gain access 
    # to the self scope, but the parameter is passed automatically 
    # when python runs.
    def __init__(self):
        # first we initialize the node with a name, "turtle_controller"
        # remember, we can only initialize a node ONCE per rospy process.
        # anonymous is False, so the turtle controller name will be sent
        # to the ROS master node for registration
        rospy.init_node("turtle_controller", anonymous=False)
        
        # use loginfo() to log messages to the console through the rospy process
        rospy.loginfo("Press Ctrl+C to end the program")

        # assign a function to rospy to enable Ctrl+C shutdown functionality
        rospy.on_shutdown(self.shutdown)

        # Now we create a new object of the Publisher type. This will be our handle 
        # to be able to publish message at the cmd_vel topic. we can
        # call publish() on our publisher handle to send our messages.
        # the general initialization of a rospy.Publisher is:
        # xyz_pub = rospy.Publisher("/topic_name", MessageType, queue_size=10)
        # We add _pub to our variables to indicate they are Publishers
        self.cmd_vel_pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)


        # We initialize the Twist message data structure like so:
        move_twst = Twist()
        # The twist message, as we saw in the prelab, has a linear and angular component (Vector3),
        # each with x, y, and z variables. 
        # type(move_twst) is Twist
        # type(move_twst.linear) is Vector3
        # type(move_twst.linear.x) is float

        # now we set the x and z values to these values to move in a circle motion
        # we are applying some linear speed in units/second. Positive values
        # will move the turtle forward and negative backwards
        move_twst.linear.x = 2.0

        # we are applying some value to the z component to turn at x radians/second
        move_twst.angular.z = 1.8

        # we define a rate to recieve messages at per second. In other words,
        # our run rate is 10Hz. To be clear, this 10 has nothing to do 
        # with the queue_size 10 defined in the publisher.
        rate = rospy.Rate(10)
        rospy.loginfo('Set Rate to 10hz')

        # We can run the main loop of the Node while we don't get a Ctrl+C input
        while not rospy.is_shutdown():
            
            # we publish our message with our handle
            self.cmd_vel_pub.publish(move_twst)
            
            # sleep for 10Hz (0.1s) and loop again
            rate.sleep() 

    # The shutdown method is called when the user inputs Ctrl+C
    def shutdown(self):
        # log some info
        rospy.loginfo("Stopping turtle controller")
        
        # passing an empty twist message will stop the turtle movement
        self.cmd_vel_pub.publish(Twist())
        
        # buffer with rospy a little to make sure the turtle stops
        rospy.sleep(1)

# this is the "main" in python
if __name__ == "__main__":
    # run the turtle controller by initializing the class. We use
    # a try except block (similar to try-catch) to make sure 
    # we can run the turtle controller node safely
    try:
        TurtleController()
    except Exception as e:
        rospy.logerr(e)
    except:
        rospy.loginfo("End of turtle_controller")
        