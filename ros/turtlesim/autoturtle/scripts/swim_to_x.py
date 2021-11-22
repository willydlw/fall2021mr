#!/usr/bin/env python

# Description: Turtle swims to user-input x location, then stops

import rospy				
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose 
from math import sqrt

from std_srvs.srv import Empty         # for reset service

class TurtleSwim():
   def __init__(self):

      # Create a node with name 'swim_to_x'
      # log level DEBUG will show all messages
      # anonymous=True ensures this is a unique node 
      self.node_name = 'swim_to_x'
      rospy.init_node(self.node_name, log_level=rospy.DEBUG, anonymous=True)

      # To move the turtle, this node will pubish a Twist message on 
      # topic /turtle1/cmd_vel. 
      self.cmd_vel = "/turtle1/cmd_vel"
      self.velocity_publisher = rospy.Publisher(self.cmd_vel, Twist, queue_size=1)
 
      # subscribe to topic '/turtle1/pose', message type Pose
      # callback function update_pose
      self.pose_topic = "/turtle1/pose"
      self.pose_subscriber = rospy.Subscriber(self.pose_topic, Pose, self.update_pose)

      # Keys CTRL + c will stop this script
      # register the class function to be called on shutdown
      rospy.on_shutdown(self.shutdown)
      rospy.loginfo(" Press CTRL+c to stop")

	   # Turtlesim will receive our messages 10 times per second.
      # 10 Hz is fine as long as the processing does not exceed 1/10 second.
      self.rate = rospy.Rate(10)
      rospy.loginfo("Set rate 10Hz")
      	
      # turtle's current pose will be stored here
      # initializing with Pose default values
      self.pose = Pose()  

      # proportional constant for linear velocity control
      self.Kp = 1.0

   def update_pose(self, data):
      """Callback function which is called when a new message
      of type Pose is received by the subscriber.
      """
      self.pose = data
      self.pose.x = round(self.pose.x, 4)
      self.pose.y = round(self.pose.y, 4)

   def euclidean_distance(self, goal_pose):
      """Euclidean distance between current pose and the goal."""
      deltaX = goal_pose.x - self.pose.x 
      deltaY = goal_pose.y - self.pose.y 
      distance = sqrt((deltaX*deltaX) + (deltaY*deltaY))
      return distance

   def linear_velocity(self, goal_pose):
      """
      Calculates linear velocity as a function of distance
      between current position and goal postion, scaled by
      the proportional control constant.
      Note: as written, will always return a positive value.
      """
      return self.Kp * self.euclidean_distance(goal_pose)

   def move_to_goal(self):
      """Moves turtle towards the goal"""
      goal = Pose()
      velocity_message = Twist()
      goalReached = False 

      # get user input for goal location
      goal.x = float(input("set x goal: "))
      if(goal.x < self.pose.x):
         rospy.logwarn("program doesn't handle backward linear velocity")
      goal.y = self.pose.y 
      goal.theta = self.pose.theta 

      # distance tolerance specifies how close turtle must
      # be to the goal point
      distance_tolerance = float(input("set distance tolerance: "))

      while(self.euclidean_distance(goal) > distance_tolerance):
         goal.y = self.pose.y 
         goal.theta = self.pose.theta
   
         # linear velocity
         velocity_message.linear.x = self.linear_velocity(goal)
         velocity_message.linear.y = 0
         velocity_message.linear.z = 0
         
         # angular velocity
         velocity_message.angular.x = 0
         velocity_message.angular.y = 0
         velocity_message.angular.z = 0

         rospy.loginfo("current pose, x: {}, y: {}, theta: {}".\
            format(self.pose.x,self.pose.y, self.pose.theta))


         rospy.loginfo("goal pose, x: {}, y: {}, theta: {}".\
            format(goal.x,goal.y, goal.theta))

         rospy.loginfo("velocity.message.linear.x: {}".format(velocity_message.linear.x))
         
         # publish the velocity message
         self.velocity_publisher.publish(velocity_message)

         # publish at the desired rate
         self.rate.sleep()

      # stop turtle after reaching goal
      velocity_message.linear.x = 0
      self.velocity_publisher.publish(velocity_message)

      if(not goalReached):
         goalReached = True 
         rospy.loginfo("Goal Reached!")
         rospy.loginfo("x: {}, y:{}, theta: {}".\
         format(self.pose.x, self.pose.y, self.pose.theta))
         rospy.sleep(1)
         rospy.wait_for_service('reset')
         clear_bg = rospy.ServiceProxy('reset', Empty)
         clear_bg()
         rospy.sleep(1)
         rospy.loginfo("what did reset do?")

      # spin goes into an infinite loop
      # ctrl + c will stop node
      rospy.spin()

   def shutdown(self):
      # You can stop turtlesim_move by publishing an empty Twist message
      rospy.loginfo("Shutdown, Stopping the turtle")

      self.velocity_publisher.publish(Twist())     

      # Give it some time to stop
      rospy.sleep(1)



if __name__ == '__main__':
    # Try and Except.
    # If an error is encountered, a try block code execution is stopped and
    # transferred down to the except block.d
    try:
        myturtle = TurtleSwim()
        rospy.sleep(1)
        myturtle.move_to_goal()
    except rospy.ROSInterruptException:
        rospy.loginfo("End of the swim for this Turtle.")

