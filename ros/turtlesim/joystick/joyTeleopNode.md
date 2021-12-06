# Writing a Teleoperation Node for a Linux-Supported Joystick

Description: This tutorial covers how to write a teleoperation node and use it to drive the turtle in the turtlesim.

The left joystick, in combination with a deadman's button press, will control the turtle's linear and angular velocity.

The general logic flow:

- subscribe to joy topic
- convert joy


## Step 1 - Modify Joy Button Listener to respond to Axes Input

In the previous tutorial, we wrote a function that subscribed to the topic joy and printed messages indicating which gamepad button was pressed. We will use the left joystick's up/down axis for the turtle's forward/backward linear x velocity. The left joystick's left/right axis will control the turtle's angular z velocity.

We will write an object-oriented solution, creating the class TurtleJoySwim to avoid the use of global variables. Let's start by writing enough code for the joy callback function to print the left joystick's left/right axes and the LB button press.

Create a file named turtle_joy_swim.py in the turtlesim_joy package, scripts directory.

```bash
cd ~/catkin_ws/src/turtlesim_joy/scripts
gedit turtle_joy_swim.py
```

</br>

Add the source code shown below.

```python
#! /usr/bin/env python

import rospy 
from sensor_msgs.msg import Joy 


class TurtleJoySwim:
   def __init__(self):
      self.updown_axis = 0
      self.leftright_axis = 0
      self.deadman_button = 0

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
      self.deadman_button = msg.buttons[4]


if __name__ == "__main__":
   try:
      rospy.init_node("turtle_joy_swim", anonymous=True)
      t = TurtleJoySwim()
   except rospy.ROSInterruptException:
      pass 
   rospy.spin()
```

</br>
Make the script executable.

```bash
chmod +x turtle_joy_swim.py
```

</br>

Create a launch file.

```xml
<launch>

  <!-- turtlesim and joy node-->
  <node name="turtlesim" pkg="turtlesim" type="turtlesim_node"/>
  <node name="joy" pkg="joy" type="joy_node"/>

  <!-- button_listener node interfaces gamepad controller to turtlesim -->
  <node 
    name="teleop_joy" 
    pkg="turtlesim_joy" 
    type="turtle_joy_swim.py" 
    output="screen"/>

</launch>
```

</br>
Overlay the catkin workspace environment and then launch the nodes.

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch turtlesim_joy turtlesim_joy_teleop.launch
```

</br>

Press the LB switch to ensure the program detects it. Move the left joystick. Notice the axes values are floating point, in the range [-1, 1]. When running jstest, in the first tutorial, we saw integer values in the range [-32767, 32767]. The joy_node must scale these integers to floating point.

Our goal is to use the axes values to control linear and angular velocities. Using the values directly, without any additional scaling, will result linear velocities in the range [-1, 1] meters/sec and angular velocities in the range [-1, 1] radian per second. One radian is approximately 57.3 degrees. Let's see how the simulated turtle reacts to these values.

</br></br>

## Step 2 - Publish Command Velocity topic

Next, we will remove the print messages from the callback function, create a publisher, and add the update_velocity function. The modified code is shown below.

```python
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
```

</br></br>

Time to run the code and watch the turtle swim. The turtle should not move unless button LB is pressed.

```bash
roslaunch turtlesim_joy turtlesim_joy_teleop.launch 
```

</br></br>
