# Differential Drive Robot Behavior

Two powered wheels, one on each side of robot. May have other passive
wheels for stability.

- Both wheels turn at same speed in the same direction, robot moves straight in that direction.
- One wheel turns faster than the other, robot turns in an arc toward the slower wheel.
- When wheels turn in opposite directions, robot turns in place.
</br></br>

## Instantaneous Center of Curvature 

- If the robot is moving in a curve, there is a center of that curve at that moment, known as the Instantaneous Center of Curavature (ICC).

- If r is the radius of the curve (measured to the center of the robot) and L is the distance between the wheels, then the rate of rotation (omega, denoted w) around the ICC is related to the velocity of the wheels by

   - right wheel velocity: 
      - vr = w(r + L/2)
   - left wheel velocity:  
      - vl = w(r - L/2)
   - angular velocity &omega; is defined as the positional velocity divided by the radius:
      - d&theta;/dt = V/r

</br>
We can solve for &omega; and r using the wheel velocity equations.

1. Subtract the two equations: vr - vl

   vr - vl = &omega;(r + L/2) - &omega;(r - L/2)
   vr - vl = &omega;L

   &omega; = (vr - vl)/L

2. Add the two equations: vr + vl

   vr + vl = &omega;(r + L/2) + &omega;(r - L/2)
   vr + vl = 2&omega;r

   r = (vr + vl)/(2&omega;)

   Sub for &omega;,

   r = (L/2)(vr + vl)/(vr - vl)

</br></br>

### Motion Summary

- angular velocity is the difference in wheel speeds over their distance apart
   - &omega; = (vr - vl)/L
- If vr = vl, then &omega; = 0, the robot moves straight
- If vr = -vl, then r is 0 and the robot spins in place
</br></br>

## Robot Pose

The robot is, at any one time at a location x, y, facing a direction which forms some angle &theta; with the x-axis of the reference frame.

Define &theta; = 0 as the robot facing along the positive x-axis.

As the robot moves, its local coordinate frame moves with it, so &theta; is the angle between the reference frame x-axis and the local frame x-axis.

Pose(x, y, &theta;)

</br></br>

## Forward Kinematic Problem

Given a robot at some pose, moving at some angular velocity &omega; during a time period dt, determine the new pose for the robot.

Robot Pose: x(t), y(t), &theta;(t)
Robot Velocity: &omega;(t), V(t)

Calculte ICC (instantaneous center of curvature) location, given the radius of curvature distance between the ICC point and the robot's reference point.

The robot's heading vector is tangential to the curve being traversed at that moment in time. The radial line segment, r, from the robot's position, p is perpendicular to the robot's heading vector.

dx - difference between ICC x coordinate and robot x coordinate
dy - difference between ICC y coordinate and robot y coordinate

dx = -r sin(&theta;)
dy =  r cos(&theta;)

To calculate the robot's new location, start at the reference frame, translate out to the original position p0. Rotate to the current pose. Translate to the ICC. Rotate around the ICC by w. Translate back out to new position.

(1) x(t) = r cos(&theta;) sin(&omega; dt) + r sin(&theta;)cos(&omega; dt) + x - r sin(&theta;)</br>

(2) y(t) = r sin(&theta;) sin(&omega; dt) - r cos(&theta;)cos(&omega; dt) + y + r cos(&theta;)</br>

(3) &theta;(t) = &theta; + &omega; dt</br></br>

## Find pose in terms of wheel velocities 

Problem Statement:

Given angular velocity &omega;, dt find robot pose x, y, &theta;

The robot's pose: x,y,&theta; depend on the robot's velocity V(t)
and the angular rotational velocity w(t).

x(t) = Integral of V(t)cos(&theta;(t)) dt</br>
y(t) = Integral of V(t)sin(&theta;(t)) dt</br>
&theta;(t) = Integral of w(t) dt</br>

over the limits of integration [0, t]</br>

For the differential drive robot, V(t) is the average of the right and left wheel velocities: vl, vr.</br>

V(t) = (vr(t) + vl(t))/ </br>

Subsituting for V(t) in the integration equations,

(4) x(t) = 1/2 Integral [vr(t)+vl(t)] cos[&theta;(t)] dt </br>
(5) y(t) = 1/2 Integral [vr(t)+vl(t)] sin[&theta;(t)] dt </br>
(6) &theta;(t) = Integral of w(t) dt </br>

over limits of integration [0, t]</br></br>

We assume that vl(t), vr(t) are constant over the limits
of integration 0,t.

We can now write eqution 4 as  

> (7) &theta;(t) = Integral of w(t) dt  
   >> &theta;(t) = Integral of (vr - vl)/L dt from 0 to t  
   >>&theta;(t) = (vr - vl)/L Integral of 1 dt from 0 to t  
   >> &theta;(t) = t(vr - vl)/L    


Substitute for &theta;(t) into equations 2 and 3,  

> (8) x(t) = 1/2 Integral [vr(t)+vl(t)] cos[&theta;(t)] dt from 0 to t  
   >> x(t) = (vr+vl)/2 Integral cos[ (vr-vl)t/L] dt from 0 to t
   >>x(t) = (L/2)(vr+vl)/(vr-vl) sin[t(vr-vl)/L] from 0 to t
   >>x(t) = (L/2)(vr+vl)/(vr-vl) (sin[t(vr-vl)/L] - sin[0(vr-vl)/L])
   >> x(t) = (L/2)(vr+vl)/(vr-vl) sin[t(vr-vl)/L]  


> (9) y(t) = (L/2)(vr+vl)/(vr-vl) [ 1 - cos[t(vr-vl)/L]]

Equations were derived with assumption period in time began at
time 0. At time 0, the robot's pose is {0,0,0}. These equations
give us the pose of the robot with respect to the robot's pose
at the beginning of the time period.


### Special Case vr = vl  

We assumed that vr was not equal to vl. Otherwise r would be infinite and these equations are not valid.  

When vr = vl, &theta;(t) is no longer a function of time as &theta; does not change.  

In terms of the local coordinate frame, &theta;(t) = 0. Subsitute that into equations (4) and (5)  

Since vr = vl, then vr = vl = v.  

> x(t) = 1/2 Integral [vr(t)+vl(t)] cos[&theta;(t)] dt  
> x(t) = 1/2 Integral [v(t)+v(t)] cos[0] dt  
> x(t) = v dt  

> y(t) = 1/2 Integral [vr(t)+vl(t)] sin[&theta;(t)] dt  
> y(t) = 0


**Equations when vl = vr**

(10) x(t) = v dt  
(11) y(t) = 0  
(12) &theta;(t) = 0  

</br></br>

### Solution Algorithm

Initial robot pose(0,0,0)

Divide the time the robot is moving into time periods where the wheel velocities are constant.

For each time period:
   Use equations 7, 8, 9 to calculate the new pose (x,y,&theta;)
   Calculate the transform from the reference frame to the new frame
      Reference Transformed to New = Reference Transformed Current x Current Transformed New

   This is the new pose of the robot in the reference frame.
   Store it as the current frame.
      Reference Transformed Current = Reference Transformed New

</br></br>
For the special case of equal wheel velocities, combine (10), (11),
(12) with the transform matrix to yield  
  
(13) x = v cos(&theta;) dt  
(14) y = v sin(&theta;) dt  
(15) &theta; = &theta;  
  
where x,y,&theta; is the pose in the reference frame
