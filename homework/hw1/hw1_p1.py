
'''
A circular, two-wheeled, differential drive steered robot is to maintain 
a velocity of 1 meter per second. This is the reference point velocity, 
with the reference point located at the center of the circle.

The user inputs a desired radius of curvature. 
Interpret the values as follows.
   Positive values are radius of curvature in meters. This indicates turning to the left.
   Negative values are radius of curvature in meters. This indicates turning to the right. 
   A zero value indicates a zero curvature. Treat this as forward straight-line travel. 
   We know that straight line travel has an infinite radius of curvature, but do not want 
   the user to have to enter -infinity or +infinity.

   Because 0 is used to represent straight-line motion, this program does not support
   turning in place, at a radius of zero, by spinning wheels in opposite directions at
   the same rotational speed.

   Program also does not support radial distances that are less than the radial distance
   from the robot center to each wheel. This is mathematically calculable, but is it 
   physically achievable?


The robot’s width from wheel to wheel is 0.70 meters, with the center reference point 
midway between the wheels. The radius of each wheel is 0.15 meters.

Calculate and print the left-side- and right-side-wheel velocities in units of meters 
per second and rotational velocities in revolutions per second.
'''

from os import name 
from subprocess import call 
from time import sleep
from math import pi 

# Global constants
WHEEL_RADIUS_METERS = 0.15
ROBOT_WIDTH_METERS = 0.70           # wheel to wheel width   
ROBOT_LINEAR_VELOCITY = 1.0         # reference point velocity, meters/sec

MIN_RADIUS_CURVATURE_METERS = ROBOT_WIDTH_METERS/2;



def calc_velocity(roc):
   # distance
   rwROC = roc + (ROBOT_WIDTH_METERS/2.0)
   lwROC = roc - (ROBOT_WIDTH_METERS/2.0)

   # robot angular velocity - all points on rigid body have same angular velocity
   robotOmega = ROBOT_LINEAR_VELOCITY / roc    # rad/sec = [m/s][1/m]

   # tangential wheel velocity
   lwTanVel = robotOmega * lwROC       # m/s = [rad/sec][m]
   rwTanVel = robotOmega * rwROC       # m/s = [rad/sec][m]

   return robotOmega, lwTanVel, rwTanVel


def display_menu():
   print("\nHomework 1, Problem 1")
   print("\nCalculate wheel velocities for radius of curavature\n")
   print("Radius should be distance from robot center to rotation point")
   print("Distances in range [{:.2f}, {:.2f}] are not supported\
         \n" .format(-MIN_RADIUS_CURVATURE_METERS, MIN_RADIUS_CURVATURE_METERS))
   print("Positive radius is left turn")
   print("Negative radius is right turn")
   print("Zero value is straight line motion\n")
   #print("Valid Range: [+-[{:.2f},{:.2f}] ".format(MIN_RADIUS_CURVATURE_METERS, MAX_RADIUS_CURVATURE_METERS))


def clear():
   # Source code reference https://www.geeksforgeeks.org/clear-screen-python/ 
   # make call for specific operating system
   _ = call('clear' if name == ('posix') else 'cls')


def get_radius_curvature():
   while True:
      try:
         display_menu()
         radius_curvature_meters = float(input("Enter radius in meters: "))
         break
      except ValueError:
         print("non-numeric entry, try again ...")
         sleep(2)                                        # seconds
         clear()
   return radius_curvature_meters    


def run_again():
   status = input("\nRun again (y/n) ")
   if status.lower() == 'y':
      return True
   else:
      return False 


def display_results(roc, omega, lwVel, rwVel, lwOmega, rwOmega, lwRev, rwRev):
   clear()
   print("\n{: <12} {: <10}  {: <10}  {: <10}  {: <10}  {: <10}\
         ".format("","", "Angular","","Wheel","Wheel"))
   print("{: <12} {: <10}  {: <10}  {: <10}  {: <10}  {: <10}\
         ".format("","  ICC", "Velocity","Tangential","Rotational", "Rotational"))
   print("{: <12} {: <10}  {: <10}  {: <10}  {: <10}  {: <10}\
         ".format("Robot"," Radius", "about ICC","Velocity","Velocity","Velocity"))
   print("{: <12} {: <10}  {: <10}  {: <10}  {: <10}  {: <10}\
         ".format("Reference","(meters)", "(rad/sec)", "(m/s)", "(rad/sec)", "(rev/sec)"))
   print("{: <12} {: <10}  {: <10}  {: <10}  {: <10}  {: <10}\
         \n".format("==========","==========","==========","==========","==========","=========="))
   print("{: <12} {: >10.2f}  {: >10}  {: >10.2f}  {: >10.2f}  {: >10.2f}\
         ".format("left wheel", (roc - ROBOT_WIDTH_METERS/2.0),"",lwVel, lwOmega, lwRev))
   print("{: <12} {: >10.2f}  {: >10.2f}  {: >10.2f}\
         ".format("center", roc, omega, ROBOT_LINEAR_VELOCITY))
   print("{: <12} {: >10.2f}  {: >10}  {: >10.2f}  {: >10.2f}  {: >10.2f}\
         ".format("right wheel", (roc + ROBOT_WIDTH_METERS/2.0),"",rwVel, rwOmega, rwRev))
   

def main():
   keep_running = True 
   
   while(keep_running):
      clear()
      roc = get_radius_curvature()                 # radius of curve from robot center, meters
      if(roc == 0):
         lwVel = ROBOT_LINEAR_VELOCITY
         rwVel = ROBOT_LINEAR_VELOCITY
         omega = 0
      elif(abs(roc) >= ROBOT_WIDTH_METERS/2):
         omega,lwVel,rwVel = calc_velocity(roc)
      else: 
         print("\nICC radius cannot be smaller than robot radius, try again")
         sleep(2)
         continue 

      # rotational wheel velocity
      lwOmega = lwVel / WHEEL_RADIUS_METERS        # rad/sec = [m/s][1/m]
      rwOmega = rwVel / WHEEL_RADIUS_METERS

      # rotational wheel velocity, rev/sec    1 rev = 2pi rad
      lwRev = lwOmega / (2.0 * pi)         #  [rev/sec] = [rad/sec][rev/rad] 
      rwRev = rwOmega / (2.0 * pi)         #  [rev/sec] = [rad/sec][rev/rad]  

      display_results(roc, omega, lwVel, rwVel, lwOmega, rwOmega, lwRev, rwRev)
      keep_running = run_again()


if __name__ == "__main__":
   main()