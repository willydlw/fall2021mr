# Create Joy Button Press Listener

Objective: Write a listener node that subscribes to the /joy topic. When a gamepad button is pressed, print a message indicating which button was pressed.

## Step 1 - Subscribe to /joy topic

We will write a listener node that subscribes to the /joy topic and prints the jostick events and values.

The Logitech F310 gamepad is used for this example. Refer to [joystick mapping](joystickMapping.md) for the button and axes mapping.

Let's make a new package, named turtlesim_joy in our catkin workspace. The dependencies will be rospy, sensor_msgs, and geometry_msgs.

```bash
cd ~/catkin_ws/src/
catkin_create_pkg turtlesim_joy rospy sensor_msgs geometry_msgs
cd ~/catkin_ws
catkin_make
source devel/setup.bash
cd ~/catkin_ws/src/turtlesim_joy/
mkdir scripts
cd scripts
```

Use your text editor to create a file named joy_button_listener.py Import rospy and sensor_msgs. Define a function named joy_button_listener that initializes the node and subscribes to the topic joy.

</br>

```python
import rospy
from sensor_msgs.msg import Joy 

def joy_button_listener():

   rospy.init_node("joy_button_listener", anonymous=True)

   # subscribe to the topic joy, message type Joy, 
   # function callback name: joy_callback
   rospy.Subscriber("joy", Joy, joy_callback, queue_size=1)

   # keep node alive until stopped
   rospy.spin()
```

</br></br>

## Step 2- Create the callback function

The function will print a message when one of the buttons is pressed.  

</br>

```python
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
```

</br>
Make the program executable.

```bash
sudo chmod +x joy_button_listener.py 
```

</br></br>

## Step 3 - Create launch File

Now, let's write a launch file to run the following nodes:
- joy_node
- joy_button_listener

</br>

```bash
cd ~/catkin_ws/src/turtlesim_joy/
mkdir launch
cd launch
gedit test_joy_listener.launch
```

```xml
<launch>

  <!-- joy node-->
  <node name="joy" pkg="joy" type="joy_node"/>

  <!-- button_listener node  -->
  <node 
    name="joy_button_listener" 
    pkg="turtlesim_joy" 
    type="joy_button_listener.py" 
    output="screen"/>

</launch>
```

## Time to launch the nodes

Make sure your gamepad is plugged into the computer before launching the nodes.

```bash
roslaunch turtlesim_joy test_joy_button_listener.launch
```

Press the different buttons to verify the mapping is correct. If you're using a different gamepad controller, adjust the code for your controller. Press ctrl + c to stop the program.

In the next [tutorial](joyTeleopNode.md), we'll write a node that controls turtlesim movement.  
