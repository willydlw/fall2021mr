#! /usr/bin/env python

import rospy 
from sensor_msgs.msg import Joy 


class TurtleJoySwim:
   def __init__(self):
      self.updown_axis = 0
      self.leftright_axis = 0
      self.deadman_pressed = 0

      # subscribe to the topic joy, message type Joy, 
      # function callback name: joy_callback
      self.joy_sub = rospy.Subscriber("joy", Joy, self.joy_callback, queue_size=1)

   def joy_callback(self, msg):

      if msg.buttons[4]:
         rospy.loginfo("pressed LB, button[4]")
   
      rospy.loginfo("left stick left/right, axes[0]: {}".format(msg.axes[0]))
      rospy.loginfo("left stick up/down,    axes[1]: {}".format(msg.axes[1]))

      self.leftright_axis = msg.axes[0]
      self.updown_axis = msg.axes[1]
      self.deadman_pressed = msg.buttons[4]


if __name__ == "__main__":
   try:
      rospy.init_node("turtle_joy_swim", anonymous=True)
      t = TurtleJoySwim()
   except rospy.ROSInterruptException:
      pass 
   rospy.spin()