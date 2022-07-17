#!/usr/bin/env python
import math
import time
import rospy
import numpy as np
import tf
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import *
from nav_msgs.msg import *
from gazebo_msgs.msg import *


####################################
############## Class ##############
class TurtleClass:
    ### Initialization ###
    def __init__(self): #whatever 'n the class call self
        self.rate = rospy.Rate(20) # Main loop rate (Hz)
        self.turtle_name = 'turtle' # this is the leader
        self.pose = np.array([[0.0],[0.0],[0.0]]) # 3-by-1 pose array
        self.velo = np.array([[0.0],[0.0],[0.0]]) # 3-by-1 pose array
        self.scan = list(range(1, 360))
        self.heading = None
        # Estimation variables
        self.Ts = 0.1
        self.mup = np.array([[0.0],[0.0],[0.0]]) # predicted state
        self.mu = np.array([[-2.0],[-1.0],[0.2]]) # estimated state 
        self.x_Ekf_Pub = Twist()
        self.G = np.identity(3) # Linearized model matrix
        self.H = np.array([[0.0,0.0,0.0],[0.0,0.0,0.0]])  # Linearized measurement matrix
        self.S_cov = 0.1*np.identity(3) # coverience
        self.R_cov = 0.01*np.identity(3) # process coverience
        self.Q_cov = 0.01*np.identity(2) #  measurement coverience

        # Measurement Vector
        self.y = np.array([[0.0],[0.0]])
        self.y_md1 = np.array([[0.0],[0.0]])
        

        # Subscribers
        # Gazebo states
        self.subGazStates = rospy.Subscriber('/gazebo/model_states', ModelStates, self.cbGazStates, queue_size=1) 
        # IMU
       # self.subIMU = rospy.Subscriber('/imu', Imu, self.cbImu, queue_size=1)
        #lidar
        self.subScan = rospy.Subscriber('/scan', LaserScan, self.findDistanceBearing, queue_size=1)
        #odom
       # self.subScan = rospy.Subscriber('/odom', Odometry, self.cbodom, queue_size=1)
        ### Publications ###
        self.pub_turtle = rospy.Publisher("cmd_vel", Twist, queue_size=10)

        rospy.on_shutdown(self.onshutdown)
        self.cbcom()

    def onshutdown(self):
            self.pub_turtle.publish(Twist())

    #publishing command as velocity  calback function

    def cbcom(self):
        
        while not rospy.is_shutdown():
            self.pub_turtle = rospy.Publisher("cmd_vel", Twist, queue_size=10)
            msg = Twist()
            msg.linear.x = 0.04 #publish linear x velocity
            msg.angular.z = 0.04

            self.pub_turtle.publish(msg)

            self.rate.sleep()

    # Gazebo states callback
    def cbGazStates(self, msg):
        if not msg == None:
            name = msg.name
            for i in range(0,len(name)):
                # Get Turtlebot pose
                if name[i] == 'turtlebot3_burger':
                 # Position
                    self.pose[0] = msg.pose[i].position.x
                    self.pose[1] = msg.pose[i].position.y
                    self.pose[2] = msg.pose[i].position.z
         #           # Orientation
                    quater = (msg.pose[i].orientation.x, msg.pose[i].orientation.y, msg.pose[i].orientation.z, msg.pose[i].orientation.w)
                    euler = tf.transformations.euler_from_quaternion(quater) # Convert the orientation to Euler (3x1)
                    self.heading = euler[2]
                    print("Turtle pose: x=%.2f" % self.pose[0], "y=%.2f" % self.pose[1],"head=%.2f" % self.heading)

    

    # IMU callback
   # def cbImu(self,msg):
#

                    # Velocity:
                    
     #               self.velo[0] = msg.angular_velocity.x
     #               self.velo[1] = msg.angular_velocity.y
      #              self.velo[2] = msg.angular_velocity.z
      #              print("Turtle vel: x=%.2f" % self.velo[0], "y=%.2f" % self.velo[1],"z=%.2f" % self.velo[2])

    #def ekf(self):
     #   self.mup = self.motion_model(v,omega)
       # self.Sp =
        #measurement update
       # K = 
       # self.mu = 
       # self.S = 


    #def motion_model(self,v,om):
      #  self.mup = self.motion_model(v,omega)
        #self.Sp =
        #measurement update
       # K = 
       # self.mu = 
       # self.S = 



#


    # LIDAR callback
    def findDistanceBearing(self,msg):
#

                    
                    self.scan = msg.ranges        
                   # print("a= " , self.scan)
                    for i in range(360):
                     #  print("a= " , self.scan[x]) 
                     if  i>1 and i<359:
                         if self.scan[i] >0 and self.scan[i]<100:
                              if self.scan[i-1] >0 and self.scan[i-1]<100:
                                  if self.scan[i+1] >0 and self.scan[i+1]<100:
                                    self.idx_bearing=i
                                    self.bearing=(i/180.)*np.pi
                                    self.dist=self.scan[i]
                                    print('bearing', self.bearing)
                                    print('distance', self.dist)
                    else:
                          if self.scan[i] >0 and self.scan[i]<100:
                            self.idx_bearing=i
                            self.bearing=(i/180.)*np.pi
                            self.dist=self.scan[i]
                            print('bearing', self.bearing)
                            print('distance', self.dist)
                           





    # ODOM CALLBACK
    #def cbodom(self,msg):
  #      rospy.loginfo(msg)





####### Main Loop #######
def main():
    ####### Initializations #######
    rospy.init_node('turtle_node', anonymous=True)
    # Instantiate object
    robot = TurtleClass()
    # Loop until an interrupt
    while not rospy.is_shutdown():
        ### Run
        ### Publish messages
        #robot.pub_all()
        # Maintain the loop rate
        robot.rate.sleep()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass