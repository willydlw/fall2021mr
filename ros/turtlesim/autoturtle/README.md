# TurtleSim Example - swim_to_x.py

The node serves as a starting point for moving the turtlesim turtle to a specified position. As is, it will only move the turtle in the forward x direction. It also provides an example of calling a rosservice to reset the simulation.  

Each python node starts with the following code to make it execute as a python script.

```python
#!/usr/bin/env python
```

Next, we import the required libraries. rospy is required to create the ROS node. Turtlesim 

```python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose 
from math import sqrt
```

## Class TurtleSwim

ROS does not require that nodes be written as classes, but we may choose to think of a node as an abstraction of an object and write an object-oriented solution.


### def __init__(self)

"__init__" is a reseved method in python classes. It is known as a constructor in object oriented concepts. This method called when an object is created from the class and it allows the class to initialize the attributes of the class.

self represents the instance of the class. By using the "self" keyword we can access the attributes and methods of the class in python.</br></br>

```python
def __init__(self):
```

Within the class init method, we call the rospy.init_node method. swim_to_x is the name of the node sent to the master. You can only have one node in a rospy process, so you can only call rospy.init_node() once. The default log_level is info and higher. In this example, we set it to debug level to provide an example of how to initialize the logging level. Debug level will show all messages, as it is the lowest level.

The anonymous keyword argument is mainly used for nodes where you normally expect many of them to be running and don't care about their names (e.g. tools, GUIs). It adds a random number to the end of your node's name, to make it unique. Unique names are more important for nodes like drivers, where it is an error if more than one is running. If two nodes with the same name are detected on a ROS graph, the older node is shutdown.</br></br>

```python
      self.node_name = 'swim_to_x'
      rospy.init_node(self.node_name, log_level=rospy.DEBUG, anonymous=True)
```

</br></br>

Next, we create a publisher for the topic cmd_vel and a subscriber to the topic /turtlesim/pose. 

Publisher general usage:

```python
pub = rospy.Publisher('topic_name', message class, queue_size=10)
```

Subscriber general usage:

```python
sub = rospy.Subscriber('topic_name', message class, callback function)
```

```python
      # To move the turtle, this node will pubish a Twist message on 
      # topic /turtle1/cmd_vel. 
      self.cmd_vel = "/turtle1/cmd_vel"
      self.velocity_publisher = rospy.Publisher(self.velocity_topic, Twist, queue_size=1)
 
      # subscribe to topic '/turtle1/pose', message type Pose
      # callback function update_pose
      self.pose_topic = "/turtle1/pose"
      self.pose_subscriber = rospy.Subscriber(self.pose_topic, Pose, self.update_pose)
```

</br></br>

The remaining code in the __init__ method registers the class function to call upon shutdown, sets the publish rate, initializes the remaining class attributes.</br></br>

```python
      # Keys CTRL + c will stop this script
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
```

</br></br>
The remaining class functions are well-documented enough that students are expected to understand them.
</br></br>

## Running the ROS Node

Make it executable

```bash
chmod +x go_to_goal_x_node.py 
```

</br></br>
Open a terminal window and start ros master

```bash
roscore
```

</br></br>
Open a second terminal and start the turtlesim_node

```bash
rosrun turtlesim turtlesim_node
```

</br></br>
Open a third terminal window in the catkin workspace

```bash
cd ~/catkin_ws
source ~/catkin_ws/devel/setup.bash
rosrun autoturtle swim_to_x.py
```

</br></br>
To change the logger level of the turtlesim_node:

```bash
rosservice call /turtlesim/set_logger_level ros.turtlesim DEBUG
```

</br></br>
To reset turtlesim

```bash
rosservice info /reset 
rosservice call /reset
```
