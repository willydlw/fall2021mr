#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Joy 

def joy_button_listener():

   rospy.init_node("joy_button_listener", anonymous=True)

   # subscribe to the topic joy, message type Joy, 
   # function callback name: joy_callback
   rospy.Subscriber("joy", Joy, joy_callback, queue_size=1)

   # keep node alive until stopped
   rospy.spin()

def joy_callback(msg):

   # print message when button is pressed
   if msg.buttons[0]:
      rospy.loginfo("pressed X, button[0]")
   elif msg.buttons[1]:
      rospy.loginfo("pressed A, button[1]")
   elif msg.buttons[2]:
      rospy.loginfo("pressed B, button[2]")
   elif msg.buttons[3]:
      rospy.loginfo("pressed Y, button[3]")
   elif msg.buttons[4]:
      rospy.loginfo("pressed LB, button[4]")
   elif msg.buttons[5]:
      rospy.loginfo("pressed RB, button[5]")
   elif msg.buttons[6]:
      rospy.loginfo("pressed LT, button[6]")
   elif msg.buttons[7]:
      rospy.loginfo("pressed RT, button[7]")
   elif msg.buttons[8]:
      rospy.loginfo("pressed BACK, button[8]")
   elif msg.buttons[9]:
      rospy.loginfo("pressed START, button[9]")
   elif msg.buttons[10]:
      rospy.loginfo("pressed Button stick left, button[10]")
   elif msg.buttons[11]:
      rospy.loginfo("pressed Button stick right, button[11]")



if __name__ == "__main__":
   try:
      joy_button_listener()
   except rospy.ROSInterruptException:
      pass 