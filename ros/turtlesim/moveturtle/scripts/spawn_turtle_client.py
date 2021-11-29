#!/usr/bin/env python 
 
import rospy
from turtlesim.srv import Spawn, SpawnRequest
 
def spawn_turtle_client(x, y, theta, name):
   # wait until the service is available with no timeout
   rospy.wait_for_service('/spawn')
   try:
      # create a callable instance named spawn_turtle
      spawn_turtle = rospy.ServiceProxy('/spawn', Spawn)
      # call the spawn_turtle instance, 
      # passing the arguments in order to the server
      server_response = spawn_turtle(x, y, theta, name)
      # the reponse message is stored in the name field
      return server_response.name
   except rospy.ServiceException as e:
      # If a service returns an error for the request, 
      # a rospy.ServiceException will be raised. 
      # The exception will contain any error messages 
      # that the service sent.
      print("Service call failed: %s"%e)
 
     
if __name__=="__main__":
   rospy.init_node("my_client_node")
   rate = rospy.Rate(10)
 
   print("Calling Spawn Service!")
   service_response = spawn_turtle_client(2.0, 1.0, 0.0, "Robbie")
   print("Turtle %s has spawned!"%service_response)
 
   while not rospy.is_shutdown():
      rate.sleep()