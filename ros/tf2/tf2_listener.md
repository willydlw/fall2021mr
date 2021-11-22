# Writing a tf2 listener (python)

Description: This tutorial teaches you how to use tf2 to get access to frame transformations.

https://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20listener%20%28Python%29 

In the previous tutorial, we created a tf2 broadcaster to publish the pose of a turtle to tf2. Here, we create a listener to start using tf2.

</br>

## 1. How to create a tf2 listener

Navigate to the package created in the previous tutorial.

```bash
roscd learning_tf2
cd scripts
```

Use your text editor to create a new file called turtle_tf2_listener.py. Copy the following code into that file.

```python
#!/usr/bin/env python  
import rospy

import math
import tf2_ros
import geometry_msgs.msg
import turtlesim.srv          # Spawn service

if __name__ == '__main__':
   rospy.init_node('tf2_turtle_listener')

   # To listen for transforms using ROS: Construct an 
   # instance of a class that implements tf2_ros.BufferInterface.

   # tf2_ros.Buffer is the standard implementation 
   # which offers a tf2_frames service that can respond to 
   # requests with a tf2_msgs.FrameGraph.
   tfBuffer = tf2_ros.Buffer()

   # Once the listener is created, it starts receiving 
   # tf2 transformations over the wire, and buffers them 
   # for up to 10 seconds.
   listener = tf2_ros.TransformListener(tfBuffer)

   # Use the turtlesim package spawn service to add a turtle
   # to the simulation
   rospy.wait_for_service('spawn')


   # rospy.SerivceProxy('service_name', my_package.srv.service_file_name)
   # returns a callable instance of the service
   spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
   
   # get the name from the parameter server
   turtle_name = rospy.get_param('turtle', 'turtle2')
   print("parameter turtle_name: {}".format(turtle_name))

   try:
      # call the service, passing the required arguments (x,y,theta,name)
      spawner_response = spawner(4, 2, 0, turtle_name)
      print("spawner_response.name: %s"%spawner_response.name)
      print("both names should be the same")
   except rospy.ServiceException, e:
      rospy.fatal("Service did not process request: %s"%str(e))

   # create a publisher for the command velocity topic
   turtle_vel = rospy.Publisher('%s/cmd_vel' % turtle_name, geometry_msgs.msg.Twist, queue_size=1)

   rate = rospy.Rate(10.0)
   while not rospy.is_shutdown():
      try:
         # lookupTransform(target_frame, source_frame, time)
         # returns geometry_msgs.msg.TransformStamped 
         trans = tfBuffer.lookup_transform(turtle_name, 'turtle1', rospy.Time())
      except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
         rate.sleep()
         continue


      msg = geometry_msgs.msg.Twist()
      # angular velocity is a fixed proportional constant times
      # the heading error. Heading error is the difference between
      # the turtle's current pose and the desired goal pose.
      msg.angular.z = 4 * math.atan2(trans.transform.translation.y, trans.transform.translation.x)
      
      # linear velocity is a fixed proportional constant times
      # the difference between the turtle's current pose and 
      # the desired goal pose.
      msg.linear.x = 0.5 * math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)

      turtle_vel.publish(msg)

      rate.sleep()
```

</br></br>

Don't forget to make the node executable.

```bash
chmod +x nodes/turtle_tf2_listener.py
```

</br></br>

## Code explanation - lookupTransform(target_frame, source_frame, time)

Returns the transform from source_frame to target_frame at time. Raises one of the exceptions if the transformation is not possible.

Parameters:

- target_frame - transformation target frame in tf, string
- source_frame - transformation source frame in tf, string
- timee - time of the transformation, use rospy.Time() to indicate most recent common time

Returns:

- position as a translation (x,y,z) and orientation as a quaternion (x,y,z,w). 
- return type: geometry_msgs.msg.TransformStamped  

Raises:

- tf2.ConnectivityException
- tf2.LookupException
- tf2.ExtrapolationException

</br>
Note that a time of zero means latest common time, so: 

```t.lookupTransform("a","b",rospy.Time())``` 

will return the transform from “a” to “b” at the latest time available in all transforms between “a” and “b”.  

</br></br>

## Running the listener

Use a text editor to open launch file, start_demo.launch and add the following lines:

```xml
<launch>
    ...
    <node pkg="learning_tf2" type="turtle_tf2_listener.py" 
          name="listener" output="screen"/>
  </launch>
```

</br>

```bash
roslaunch learning_tf2 start_demo.launch 
```

</br></br>

## Checking the Results

To see if things work, simply drive around the first turtle using the arrow keys (make sure your terminal window is active, not your simulator window), and you'll see the second turtle following the first one!

When the turtlesim starts up you may see:

```bash
[ERROR] [1418082761.220546623]: "turtle2" passed to lookupTransform argument target_frame does not exist.
[ERROR] [1418082761.320422000]: "turtle2" passed to lookupTransform argument target_frame does not exist.
```

This happens because our listener is trying to compute the transform before turtle 2 is spawned in turtlesim and broadcasting a tf2 frame.

</br></br>