#!/usr/bin/env python
import rospy
from math import atan2, cos, pi, sin, sqrt
# remember the Twist message from the prelab?
# here we import it from geometry_msgs to gain
# access to the data structure. 
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


# Define the TurtleFetch class
class TurtleFetch():
    def __init__(self, x, y, tol):

        # save the class parameters as 'self' variables
        self.x_d = x
        self.y_d = y
        self.tolerance = tol

        # first we initialize the node with a name, "turtle_fetch"
        # remember, we can only initialize a node ONCE per rospy process.
        # anonymous is False, so the node name will be sent
        # to the ROS master node for registration
        rospy.init_node("turtle_fetch", anonymous=False)
        
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


        # TODO: CREATE AND INSTANCE OF self.pose AND ASSIGN IT TO AN EMPTY Pose TYPE
     

        rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)

	    #TODO: CREATE INSTANCE OF TWIST MESSAGE TYPE


        # we define a rate to recieve messages at per second. In other words,
        # our run rate is 10Hz. To be clear, this 10 has nothing to do 
        # with the queue_size 10 defined in the publisher.
        rate = rospy.Rate(10)
        rospy.loginfo('Set Rate to 10hz')

        # We can run the main loop of the Node while we don't get a Ctrl+C input
        while not rospy.is_shutdown():
            # TODO: CHECK IF CURRENT x AND y ARE WITHIN THE TOLERANCE AND PUBLISH THE EMPTY TWIST IF THEY ARE
                # TODO: BREAK OUT OF THE TOLERANCE CHECK IF STATEMENT AFTER PUBLISHING THE TWIST
                

	        # TODO: CALCULATE vx 
          
            # TODO: CALCULATE wz
          
            # TODO: ASSIGN vx TO Twist CREATED EARLIER

          
            # TODO: ASSIGN wz TO Twist CREATED EARLIER
           
            # TODO: PUBLISH Twist WITH self.cmd_vel_pub.publish()
            
            # sleep for 10Hz (0.1s) and loop again
            rate.sleep() 

    # this function will be called each time there is a message on the 
    # /turtle1/pose topic. The data parameter will be of Pose type
    def pose_callback(self, data):
        # TODO: SAVE 'data' IN THE self.pose VARIABLE WE CREATED IN __init__()
       
        pass

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
    user_x = float(input("Enter X [0-10]: "))
    # TODO Ask for y, and tolerance values:


    # run the TurtleFetch by initializing the class. We use
    # a try except block (similar to try-catch) to make sure 
    # we can run the turtle controller node safely
    try:
        # TODO: PASS CLASS PARAMTERS TO INITIALIZATION:
        TurtleFetch(user_x, )
    except Exception as e:
        rospy.logerr(e)
    except:
        rospy.loginfo("End of turtle_fetch")
        
