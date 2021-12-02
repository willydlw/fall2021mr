# Reading Laser Scan Data

Hitachi-LG LDS
The TurtleBot 3 laser distance sensor rotates in a continuous 360 degrees to collect 2D distance data.

Basic specifications for the Hitcahi-LG device can be found at http://wiki.ros.org/hls_lfcd_lds_driver?action=AttachFile&do=view&target=LDS_Basic_Specification.pdf.

</br>

![Hitachi Laser Distance Sensor](./images/hitachi_laser.jpg)

</br></br>

| Spec | LDS |
| --- | --- |
| Distance range (meters) | 0.120 – 3.5 |
|    |    |
| Distance accuracy (meters) |    |
| 0.120 – 0.499 | ± 0.015 |
| 0.500 – 3.500 | ± 5.0% |
|    |    |
| Distance precision (meters) |    |
| 0.120 – 0.499 | ± 0.010 |
| 0.500 – 3.500 | ± 3.5%
|    |    |
| Rotation |    |
| Scan rate (rpm) | 300 ± 10 |
| Angular range (degrees) | 360 |
| Angular resolution (degrees) | 1 |

</br></br>


## scan topic

Running the autonmous navigation simulation from the previous tutorial, we see that gazebo publishes the laser data in the scan topic.

```bash
rostopic info /scan
```

Produces the following output:

```bash
Type: sensor_msgs/LaserScan

Publishers: 
 * /gazebo (http://localhost:34401/)

Subscribers: 
 * /turtlebot3_drive (http://localhost:39133/)
 ```

</br>

```bash
rosmsg show sensor_msgs/LaserScan 
```

shows us the message structure:

```bash
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
float32 angle_min
float32 angle_max
float32 angle_increment
float32 time_increment
float32 scan_time
float32 range_min
float32 range_max
float32[] ranges
float32[] intensities
```

</br>

We need this information to write a python subscriber program to read the laser data.

## Writing Laser Scan Data Subscriber Node

Let's create a new package, named laser_tutorial, in a catkin workspace, with the following commands.

```bash
cd ~/catkin_ws/src/
catkin_create_pkg laser_tutorial rospy sensor_msgs
cd ~/catkin_ws
catkin_make
source /devel/setup.bash
```

Next, create a scripts directory in the laser_tutorial package.

```
cd ~/catkin_ws/src/laser_tutorial 
mkdir scripts
cd scripts
```

Use your text editor to read a file named laser_listener.py in the scripts directory. The code is a bare-bones listener that prints data from every tenth scan message. 

```python
#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan 
import numpy

class LaserScanListener:
   def __init__(self):
      self.angle_min = 0.0                      # radians
      self.angle_max = 6.28318977356            # radians
      self.angle_increment = 0.0175019223243    # radians
      self.range_min = 0.119999997318           # meters
      self.range_max = 3.5                      # meters
      self.range_data = []
      self.count = 0

      self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)


   def scan_callback(self, msg):
      self.count = msg.header.seq
      self.range_data = msg.ranges
      
      if(self.count % 10) == 0:
         print("scan count: {}".format(self.count))
         print("range data")
         print(self.range_data)


         
if __name__ == "__main__":
   rospy.init_node('laser_listener', anonymous=True)
   rospy.loginfo("creating LascerScanListener")
   ls = LaserScanListener()
   rospy.spin()
   
```

</br>

```bash
chmod +x laser_listener.py 
rosrun --debug laser_tutorial laser_listener.py 
```

After about 2 seconds, the tenth scan will be read, and the range data will be displayed.