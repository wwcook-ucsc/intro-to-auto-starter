#!/usr/bin/env python
#^ required shebang line


# import rospy libaray for ROS Support
import rospy
# import Greeting message data type. 
from lab1_prelab.msg import Greeting

# HelloNode class definition
class HelloNode:
	def __init__(self):
		# constructor function will initialize HelloNode
		rospy.init_node('hello_node', anonymous=True)
		# create a publisher handle which will publish Greeting messages
		# to the say_hello topic
		self.pub = rospy.Publisher('say_hello', Greeting, queue_size=10)
	
	def run(self):
		# this function will run the main loop where we create our message
		# and publish to our custom topic

		# set a rate to run the loop at
		rate = rospy.Rate(10) # 10hz
		while not rospy.is_shutdown():
			# create an instance of our greeting message
			greeting_msg = Greeting()
			# set the my_greeting field to a string with hello world and the
			# current ros time
			greeting_msg.my_greeting = "hello world %s" % rospy.get_time()
			
			# publish the message with our publisher handle
			self.pub.publish(greeting_msg)
			# sleep for the rate we set (10hz)
			rate.sleep()
    
# main function -- this is what python will run when the Node is executed
if __name__ == '__main__':
	# create an object of the HelloNode class we defined above
	hello_node = HelloNode()

	# try running the run() function where we publish our message
	try:
		hello_node.run()
	except rospy.ROSInterruptException:
		pass
