#!/usr/bin/env python

import rospy
from turtlesim.srv import * 

def teleport_turtle_absolute_client(x, y, theta):
   # wait until the service with no timeout
   rospy.wait_for_service('/turtle1/teleport_absolute')
   try:
      teleport_turtle = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
      teleport_response = teleport_turtle(x,y,theta)
      print("teleport_response")
      print(teleport_response)
   except rospy.ServiceException as e:
      # If a service returns an error for the request, 
      # a rospy.ServiceException will be raised. 
      # The exception will contain any error messages 
      # that the service sent.
      print("Service call failed: %s"%e)

if __name__=="__main__":
   rospy.init_node("my_teleport_node")
   rate = rospy.Rate(10)
 
   print("Calling Teleport Absolute Service!")
   teleport_turtle_absolute_client(8.0, 6.0, 0.0)
   print("Turtle has teleported!")
 
   while not rospy.is_shutdown():
      rate.sleep()