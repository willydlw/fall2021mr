#! /usr/bin/env python

import rospy 
from sensor_msgs.msg import Joy 
from geometry_msgs.msg import Twist


class TurtleJoySwim:
   def __init__(self):
      self.updown_axis = 0
      self.leftright_axis = 0
      self.deadman_pressed = 0      # 0 is False
      
      # subscribe to the topic joy, message type Joy, 
      # function callback name: joy_callback
      self.joy_sub = rospy.Subscriber("joy", Joy, self.joy_callback, queue_size=1)

      # publish the topic /turtle1/cmd_vel
      self.vel_pub = rospy.Publisher("turtle1/cmd_vel", Twist, queue_size=1)
      

   def joy_callback(self, msg):
      
      # LB button
      self.deadman_pressed = msg.buttons[4]

      # left stick, left/right axis
      self.leftright_axis = msg.axes[0]

      # left stick, up/down axis
      self.updown_axis = msg.axes[1]

   
   def update_velocity(self):
      # create object, all values initialized to zero by default
      cmd_vel = Twist()

      if self.deadman_pressed:
         cmd_vel.linear.x = self.updown_axis
         cmd_vel.angular.z = self.leftright_axis

      self.vel_pub.publish(cmd_vel)
      rospy.loginfo("linear.x: {}, angular.z: {}".format(cmd_vel.linear.x, cmd_vel.angular.z))
      

if __name__ == "__main__":
   try:
      rospy.init_node("turtle_joy_swim", anonymous=True)
      t = TurtleJoySwim()
      rate = rospy.Rate(10)         # frequency, Hz

      while not rospy.is_shutdown():
         t.update_velocity()
         rate.sleep()
   except rospy.ROSInterruptException:
      pass 
   