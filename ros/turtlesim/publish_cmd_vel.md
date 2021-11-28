# Publish cmd_vel topic

Description: move the turtle around the turtlesim window by publishing velocity commands.

## Start Simulation

Start ROS Master and  by opening a terminal windows and typing:

```bash
roscore
```

In the second terminal, run the executable turtlesim_node with the command: 

```bash
rosrun turtlesim turtlesim_node
```

## Publish from Command Line

With the simualtion running, open a third terminal and type the following commands

```bash
rostopic list
rostopic info /turtle1/cmd_vel
```

</br>

The first commands shows us a list of active topics. We are intertested in the topic /turtle1/cmd_vel. The second command displays the following:

```bash
Type: geometry_msgs/Twist

Publishers: None

Subscribers: 
 * /turtlesim 
```

This tells us that the /turtlesim node is subscribing to the topic /turtle1/cmd_vel, there is no node publishing that topic and that the message type is geometry_msgs/Twist.

The ROS wiki, http://wiki.ros.org/turtlesim, provides the following information for topic turtleX/cmd_vel.

"The linear and angular command velocity for turtleX. The turtle will execute a velocity command for 1 second then time out. Twist.linear.x is the forward velocity, Twist.linear.y is the strafe velocity, and Twist.angular.z is the angular velocity."

</br>

To see the Twist message format, type:

```bash
rosmsg show geometry_msgs/Twist
```

The output shows that the message format allows six floating-point values which determine the linear and angular velocity of the turtle:

```bash
geometry_msgs/Vector3 linear
  float64 x
  float64 y
  float64 z
geometry_msgs/Vector3 angular
  float64 x
  float64 y
  float64 z
```

</br> </br>

## rostopic pub

We can publish topics from the command line with the rostopic pub command. The general command form is

```bash
rostopic pub [topic name] [message type] [arguments]
```

Let's start by moving the turtle in the forward x direction. Enter the command shown below. We are publishing the topic: /turtle1/cmd_vel. The message type is geometry_msgs/Twist. The option -1 says to only publish this message once. This is followed by the linear and angular velocity arguments of the Twist message fields. The data arguments are actually in YAML syntax, which is described in the YAML Command Line documentation at http://wiki.ros.org/ROS/YAMLCommandLine

```bash
rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -1 '[2.0, 0.0, 0.0]' '[0.0, 0.0, 0.0]'
```

The terminal output:

```bash
publishing and latching message for 3.0 seconds
```

You should see the turtle has moved in the positive x direction, as shown in the screenshot below.

![forward x](./images/forwardx.png)

</br>

The turtle was originally spawned at x = 5.54, y = 5.54, theta = 0.0. Use rostopic echo to see its current position. The option -n 1 says to echo it once.

```bash
rostopic echo -n 1 /turtle1/pose
```

</br>

The turtle's x position is now 7.56, with y and theta remaining the same. Why did the turtle move 2 meters in the x direction? Our published command said to move at a linear velocity of 2 meters/sec. The ros wiki indicates that any published commands are executed for one second only. Thus, our turtle moved 2 meters in that one second time interval. 

```bash
x: 7.56044435501
y: 5.544444561
theta: 0.0
linear_velocity: 0.0
angular_velocity: 0.0
```

</br>

We can move the turtle in the reverse x direction, by negating the linear x velocity. Enter the following command:

```bash
rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -1 '[-2.0, 0.0, 0.0]' '[0.0, 0.0, 0.0]'
```

</br>

We now see the turtle moving to the left, negative x direction, and stopping near its starting position of x = 5.54.

![backward x](./images/backwardx.png)

</br> </br>

### angular z velocity

We can make the turtle rotate by giving it an angular velocity about the z-axis. In the simulation, imagine the z-axis as coming out of the display, perpendicular to the xy plane.

Enter the command, to see the turtle rotate about its own center. The -r 1 says to publish this command at a rate of 1 Hz. The angular speed, 1.8, is in units of radians/second.

```bash
rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -r 1 '[0.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'
```

Type Ctrl + C to stop publishing the command and stop the turtle's motion. Reset the simulation with the command


</br>

```bash
rosservice call /reset
```

This places the turtle back in its orginal location and redraws the background, removing the turtle pen lines. Services are discussed in detail in the next unit.

Finally, to move the turtle in a circular, type the command:

```bash
rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -r 1 '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'
```

![circle](./images/circle.png)

</br> </br>

To learn more about the rostopic pub options, type ```rostopic pub -h```.

</br> </br>


## Creating a python publisher node

Let's start by writing a python script that causes the turtle to swim in a circle. We'll start by creating a new package named moveturtle that has dependencies: rospy, std_msgs, geometry_msgs.

Packages are created in the catkin workspace src directory. If you do not have a catkin workspace, follow these [instructions](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) to create one.

Change directories into our workspace src folder:

```bash
cd ~/catkin_ws/src/
```

Now create the package:

```bash
catkin_create_pkg moveturtle std_msgs rospy geometry_msgs
```

This will create a directory named moveturtle that contains two files: CMakeLists.txt, package.xml and a directory named src.

Build the package and overlay the environment to let ros know about our new package.

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

Create a new directory named scripts inside the moveturtle directory.

```bash
cd ~/catkin_ws/src/moveturtle
mkdir scripts
cd scripts
```

In the scripts directory, use your favorite text editor to create a file named move_circle.py.

### Implementation

The first line of every python program makes it an executable script. This means we can run the program with just the program name and do not have to run it with python myprogram.py.

```python
#! /usr/bin/env python
```

Next, import all the packages used in the program. rospy is a ROS-python library that contains different functions like creating a node, getting time, creating a publisher, etc. 

```python
import rospy
from geometry_msgs.msg import Twist 
import sys 
```

Let's define a function named swim_circle that has one parameter: radius.

```python
def swim_circle(radius):
   # initialize the node with the name move_circle
   # when setting anonymous as true, we can have
   # multiple instances of this node running if needed
   rospy.init_node('move_circle', anonymous=True)

   # create a publisher for the topic name 
   # /turtle1/cmd_vel that publishes Twist messages
   # The queue_size determines how many Twist messages
   # are stored in the queue before they are overwritten
   # The number 10 is used in many ros examples.
   pub = rospy.Publisher('/turtle1/cmd_vel', 
                           Twist, queue_size=10)

   # rate will control the execution time of the loop
   rate = rospy.Rate(10)

   # create a Twist() message object
   vel = Twist()
```

Next, inside the function we create a while loop that allows the turtle to swim in a circle until the node is shutdown with Ctrl + C. Inside the while loop, we set the linear x velocity to the radius value. The angular.z velocity is hard-coded to 1 radian/second. All other velocity fields are set to zero.

The message is then published. The rospy.loginfo() function publishes the radius each loop iteration.
rate.sleep() is added at the end. The rate object keeps track of the time since the last rate.sleep() was executed and sleeps for the correct amount of time to maintain a 10Hz frequency.

```python
   while not rospy.is_shutdown():
      vel.linear.x = radius
      vel.linear.y = 0.0
      vel.linear.z = 0.0
      vel.angular.x = 0.0
      vel.angular.y = 0.0
      vel.angular.z = 1.0
      rospy.loginfo("radius = %f", radius)
      pub.publish(vel)
      rate.sleep()
```

Finally, the function is called from this section of code. The radius argument is extracted from the command line arguments and passed to the swim_circle function.

```python
if __name__ == '__main__':
   try:
      swim_circle(float(sys.argv[1]))
   except rospy.ROSInterruptException:
      pass 
```

### Execution of move_circle.py

The file must be given executable privileges. Type the following:

```bash
chmod +x move_turtle.py
```

Next, we start the simulation running. Open a terminal and start ROS Master

```bash
roscore
```

In another terminal, start the simulation.

```bash
rosrun turtlesim turtlesim_node
```

In a third terminal, start the move_circle node.

```bash
source catkin_ws/devel/setup.bash
rosrun moveturtle move_circle.py 2.0
```

Yeah! The turtle is now swimming in a circle.

![swim circle](./images/swim_circle.png)

</br></br>

You can watch the turtle's position change with the command ```rostopic echo /turtle1/pose``` 

Type Ctrl + C to stop the program.

Try running the program again with different radius values.

</br></br>

## Next Tutorial: Go To Goal

In the next example [go to goal](go_to_goal.md), we will write an example to move the turtle to a goal location.
