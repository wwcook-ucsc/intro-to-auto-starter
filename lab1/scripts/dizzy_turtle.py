#!/usr/bin/env python
import rospy
from math import atan2, cos, pi, sin, sqrt
# remember the Twist message from the prelab?
# here we import it from geometry_msgs to gain
# access to the data structure. 
from geometry_msgs.msg import Twist

# TODO: SET FORMULA CONSTANTS HERE
ANGULAR_Z = 0
V = 0

# Define the DizzyTurtle class
class DizzyTurtle():
    def __init__(self):
        # first we initialize the node with a name, "dizzy_turtle"
        # remember, we can only initialize a node ONCE per rospy process.
        # anonymous is False, so the node name will be sent
        # to the ROS master node for registration
        rospy.init_node("dizzy_turtle", anonymous=False)
        
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

	    #TODO: CREATE INSTANCE OF TWIST MESSAGE TYPE
  

        # we define a rate to recieve messages at per second. In other words,
        # our run rate is 10Hz. To be clear, this 10 has nothing to do 
        # with the queue_size 10 defined in the publisher.
        rate = rospy.Rate(10)
        rospy.loginfo('Set Rate to 10hz')

	    #TODO: RECORD THE START TIME HERE FOR ELAPSED CALCULATION

        # We can run the main loop of the Node while we don't get a Ctrl+C input
        while not rospy.is_shutdown():
	        # TODO: CALCULATE vx and vy WITH SPIRAL FORMULA

            # TODO: ASSIGN VALUES TO TWIST
            
            # TODO: PUBLISH TWIST MESSAGE TO CMD_VEL WITH PUBLISHER HANDLE
            
            # sleep for 10Hz (0.1s) and loop again
            rate.sleep() 

    # The shutdown method is called when the user inputs Ctrl+C
    def shutdown(self):
        # log some info
        rospy.loginfo("Stopping dizzy_turtle")
        
        # passing an empty twist message will stop the turtle movement
        self.cmd_vel_pub.publish(Twist())
        
        # buffer with rospy a little to make sure the turtle stops
        rospy.sleep(1)

# this is the "main" in python
if __name__ == "__main__":
    # run the dizzy_turtle by initializing the class. We use
    # a try except block (similar to try-catch) to make sure 
    # we can run the turtle controller node safely
    try:
        DizzyTurtle()
    except Exception as e:
        rospy.logerr(e)
    except:
        rospy.loginfo("End of dizzy_turtle")
        
