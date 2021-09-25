'''
 Develop a simulation of a two-wheeled differential drive robot at various time intervals. 
 
 Simulate for
   straight-line motion
   left turn motion
   right turn motion.
   
   An example of left turn behavior might be making the right-side velocity 10% greater 
   than the average speed and the left velocity 10% less than the average speed 
   (right speed + left speed)/2. 
   
   Simulate several seconds of motion. 
   
   Assume the sampling time interval is 0.1 seconds. 
   
   Plots:
      x versus t
      y versus t
      heading versus t
      y versus x 
      
   Assumptions: Robot has following physical characteristics 
      Wheel to wheel width 0.7 m
      Wheel radius 0.15
      Robot reference point centered between wheels
      Reference point velocity 1 m/s
      Initial pose: x = 0, y = 0 at time t = 0
      Constant velocity, zero acceleration
'''

''' Theoretical Background

   If the robot is moving in a curve, there is a center of that curve at that moment,
   known as the Instantaneous Center of Curvature (or ICC). 
   
   We talk about the instantaneous center, because we’ll analyze this at each instant- 
   the curve may, and probably will, change in the next moment.

   If r is the radius of the curve (measured to the middle of the robot) and l is
   the distance between the wheels, then the rate of rotation (ω) around the ICC is
   related to the velocity of the wheels by:

   vr = w (r + l/2)
   vl = w (r - l/2)

   Comes from relationship, angular velocity w is dtheta/dt = V/r
   where V is positional velocity divided by the radius

   Intuitively, the farther you are from the center of rotation, the faster you need to 
   move to get the same angular velocity. If you travel at π radians per second for 1 second, 
   you should travel a distance of half the circumference, or πr. Since this was in one second, 
   the velocity was πr per second. So π radians per second equals πr velocity, so v = ωr. 
   Once we have those equations, we can solve for r or ω:

   vr = wr + wl/2
   vl = wr - wl/2

   vr - vl = (wr + wl/2) - (wr - wl/2) = 2wl/2 = wl

   w = (vr-vl)/l

   Add the two equations vr + vl to solve for r

   vr + vl = (wr + wl/2) + (wr - wl/2) = 2wr

   2wr = vr + vl  

   r = (vr+vl)/(2w)   subsitute (vr-vl)/l for w

   r = (l/2) * (vr+vl)/(vr-vl)

'''

import math
import numpy as np 
import matplotlib.pyplot as plt 

# Global constants
WHEEL_RADIUS = 0.15          # meters
WW_WIDTH = 0.70              # wheel to wheel width, meters   
ROBOT_VELOCITY = 1.0         # reference point velocity, meters/sec


def sim_robot_motion(simNumber,  simTime, dt, speed_diff):
   ''' 
   simNumber is the simulation number used for the figure title
   simTime is total simulation time [s]
   dt - simulation time step        [s]
   speed_diff - integer value for percent difference in wheel speeds
   '''
   numSamples = int(simTime/dt) 
   t = np.arange(0, simTime, dt)
   x = np.zeros(numSamples)               # x position, [m]
   y = np.zeros(numSamples)               # y position, [m]
   theta = np.zeros(numSamples)           # heading, [rad]

   # loop control
   n = 0

   # calc wheel velocities [m/s]
   vl = ROBOT_VELOCITY * ((100.0 - speed_diff)/100.0)
   vr = ROBOT_VELOCITY * ((100.0 + speed_diff)/100.0)

   # omega, w - angular velocity about ICC point
   w = (vr - vl)/ WW_WIDTH                # [rad/sec]


   # radius of curvature, r = (L/2)(vr + vl)/(vr - vl)
   if (vr - vl) != 0:
      rc = (WW_WIDTH/2) * (vr+vl)/(vr+vl)   # [m]
   else:
      rc = 0


   while n < np.size(t):
      if rc != 0:
         '''
         (7) theta(t)= Integral of w(t) dt
                  = Integral of (vr - vl)/L dt from 0 to t
                  = (vr - vl)/L Integral of 1 dt from 0 to t
                  = t(vr - vl)/L 
         '''
         theta[n] = w * t[n]
         '''
         (8) x(t) = 1/2 Integral [vr(t)+vl(t)] cos[theta(t)] dt from 0 to t
               = (vr+vl)/2 Integral cos[ (vr-vl)t/L] dt from 0 to t
               = (L/2)(vr+vl)/(vr-vl) sin[t(vr-vl)/L] from 0 to t
               = (L/2)(vr+vl)/(vr-vl) [ sin[t(vr-vl)/L] - sin[0(vr-vl)/L]]
               = (L/2)(vr+vl)/(vr-vl) sin[t(vr-vl)/L]

         (9) y(t) = (L/2)(vr+vl)/(vr-vl) [ 1 - cos[t(vr-vl)/L]]
         '''
         x[n] = rc * np.sin(theta[n])
         y[n] = rc * (1 - np.cos(theta[n]))
      else:
         '''
         Equations when vl = vr:

         (10) x(t) = v dt
         (11) y(t) = 0
         (12) theta(t) = 0
         '''
         x[n] = (vr+vl)/2*t[n] 
         y[n] = 0
         theta[n] = 0
         
      n = n + 1

   displaySimulationInfo(simNumber, speed_diff, vl, vr, w, rc)
   plotSimulationData(simNumber, speed_diff, x,y,t,theta)


def plotSimulationData(simNumber, speed_diff, x,y,t,theta):

   # plot simulation results
   size = 8
   figure_title = 'Simulation {}, Wheel Speed Difference: {}% \
                  '.format(simNumber, speed_diff)
   fig,axs = plt.subplots(2,2)
   fig.canvas.set_window_title(figure_title)
   axs[0, 0].plot(x,y, 'b')    
   axs[0, 0].set_title("Robot path",fontsize=size)
   axs[0, 0].set_xlabel('X displacement [m]',fontsize=size)
   axs[0, 0].set_ylabel('Y displacement [m]',fontsize=size)
  
   axs[0, 1].plot(t,x, 'g')         
   axs[0, 1].set_title("X position vs time",fontsize=size)
   axs[0, 1].set_xlabel('Time [sec]',fontsize=size)
   axs[0, 1].set_ylabel('X displacement [m]',fontsize=size)
    
   axs[1, 1].plot(t,y, 'r')         
   axs[1, 1].set_title("Y position vs time",fontsize=size)
   axs[1, 1].set_xlabel('Time [sec]',fontsize=size)
   axs[1, 1].set_ylabel('Y displacement [m]',fontsize=size)
    
   axs[1, 0].plot(t,theta*180/np.pi, color='orange') 
   axs[1, 0].set_title("Heading vs time",fontsize=size)
   axs[1, 0].set_xlabel('Time [sec]',fontsize=size)
   axs[1, 0].set_ylabel('Heading [deg]',fontsize=size)
   fig.tight_layout()
   
def displaySimulationInfo(simNumber, speed_diff, vl, vr, w, rc):
   print("Simulation {}".format(simNumber))
   print("   Wheel speed difference: {} %".format(speed_diff))
   print("   Wheel velocity")
   print("      left:  {:6.3f} [m/s]".format(vl))
   print("      right: {:6.3f} [m/s]".format(vr))
   print("   Angular Velocity about ICC: {:8.3f} [rad/s]".format(w))
   print("   Radius of Curvature:        {:8.3f} [m]\n\n".format(rc))

   
def displayProgramInfo(simTime, dt):
   print("\nHomework 1 - Differential Drive Robotics Simulation\n")
   print("Assumptions:")
   print("   Robot reference point velocity is constant, {} [m/s]".format(ROBOT_VELOCITY))
   print("   Reference point, centered between wheels")
   print("   Wheel to wheel width, {:.2f} [m]".format(WW_WIDTH))
   print("   Both wheels revolve in same direction")

def main():
   # percent difference between right and left wheel velocities
   speed_diff = [-10, 0, 10]

   dt = 0.1                         # sampling time interval, [s]
   simTime = 25.0                   # simulation end time, [s]

   # loop variables
   k = 0

   displayProgramInfo(simTime,dt)

   while k < np.size(speed_diff):      
      sim_robot_motion(k, simTime, dt, speed_diff[k])
      k = k + 1
      print("\n\n")
   plt.show() 
   

if __name__ == "__main__":
   main()