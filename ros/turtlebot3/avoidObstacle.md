# Obstacle Avoidance Using Laser Scan Data

The following is a simple method for determining a heading away from obstacles.


## Step 1 - Launch a turtlebot3 gazebo world simulation

The stage_2 gazebo simulation world is used for this example.

```bash
roslaunch turtlebot3_stage_2.launch
``` 

![stage 2 world](./images/stage_2_world.png)

</br></br>


## Step 2 - Create Avoid Obstacle ROS node

```python
```


Make the script executable.

```bash
chmod +x avoid_obstacle.py
```

Run the script.

```bash
rosrun avoid_obstacle.py
```

</br>
Example console output and polar plot are shown below.

```bash
[INFO] [1638905950.310838, 391.162000]: distance threshold: 2.000000
[INFO] [1638905950.315753, 391.167000]: heading vector magnitude: 1.00
[INFO] [1638905950.320298, 391.171000]: xheading: 1.00, yheading: -0.00, angle: -0.10 degrees
```

</br></br>

![polar plot](./images/polar_plot_stage_2_world.png)

</br></br>

Type ctrl + c to stop the node.
</br></br>

## Step 3 - Test in node in empty world

Shutdown the stage 2 world and then launch the empty world. Place obstacles in various locations to observe the obstacle avoidance headings calculated.

</br>

![box added](./images/box_added.png)

</br></br>

Run the polar plot program again. The plot screenshot below shows that angle 0 degrees corresponds to the robot's x-axis.

```bash
rosrun laser_tutorial laser_polar_plot.py
```
</br>

![box plot](./images/box_plot.png)

</br></br>

Experiment by add other obstacles to the world to see that 90 degrees is to the robot's left, 180 degrees is behind the robot, and 270 degrees is to the robot's right.

The screenshot below shows an obstacle placed approximately 2.5 meters in front of the turtlebot. The distance threshold in the config file was changed to 3.5 meters.

![front obstacle](./images/front_obstacle.png)

</br></br>

Run the avoid obstacle node. The output is shown below. Notice that it computes a rotation of almost 180 degrees to travel in the opposite direct from the detected obstacle.

```bash
[INFO] [1638906541.787151, 380.123000]: distance threshold: 3.500000
[INFO] [1638906541.793634, 380.130000]: heading vector magnitude: 1.00
[INFO] [1638906541.797595, 380.134000]: xheading: -0.99, yheading: 0.10, angle: 174.12 degrees
```
</br>

![front obstacle](./images/front_obstacle.png)

</br></br>

Now, change the distance threshold to 2.0. The obstacle is beyond the filtered range, so the robot does not change course. It calculates a heading of straight ahead.

```bash
[INFO] [1638906741.360553, 578.523000]: distance threshold: 2.000000
[INFO] [1638906741.365284, 578.527000]: heading vector magnitude: 1.00
[INFO] [1638906741.369651, 578.531000]: xheading: 1.00, yheading: 0.00, angle: 0.00 degrees
```

</br>

![front obstacle 2](./images/front_obstacle_threshold_2.png)

</br></br>

Set the distance threshold back to 3.5 and adding two more obstacles, as shown below.

![obstacle 3](./images/obstacles_3.png)

</br></br>

The avoid obstacle heading is -13.79 degrees, as shown below.

```bash
[INFO] [1638907131.855165, 966.726000]: distance threshold: 3.500000
[INFO] [1638907131.859939, 966.731000]: xheading: 0.97, yheading: -0.24, angle: -13.79 degrees
```

</br>

![plot obstacle 3](./images/plot_obstacles_3.png)

</br></br>