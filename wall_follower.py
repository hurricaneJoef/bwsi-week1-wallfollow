#!/usr/bin/env python

import numpy as np

import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollower:
    # Import ROS parameters from the "params.yaml" file.
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT
    SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic")
    DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic")
    SIDE = rospy.get_param("wall_follower/side")
    VELOCITY = rospy.get_param("wall_follower/velocity")
    DESIRED_DISTANCE = 5#rospy.get_param("wall_follower/desired_distance")
    kp=1
    ki=.0
    kd=1
    lastsum=0
    def __init__(self):
        # Initialize your publishers and
        # subscribers
        self.data = None    
        self.angle = 0
        self.cmd = AckermannDriveStamped()
        self.laser_sub = rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.scan, queue_size=1)
        self.drive_pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=1)

    def scan(self, data):
        #stores the lidar data so you can work with it
        self.data = data

    #calls function that controls driving
        self.drive()
    
    def drive(self):
        """controls driving"""

    #gets the angle required
        self.angle = self.find_wall()

    #sets speed and driving angle
        self.cmd.drive.speed = self.VELOCITY
        self.cmd.drive.steering_angle = self.angle

        #publishes the command
        self.drive_pub.publish(self.cmd)
    def pid(self):
        #wall on right
        pos=min(self.data.ranges[49:99])
        p=(self.DESIRED_DISTANCE-pos)
        d=np.sin((49-self.data.ranges.index(max(self.data.ranges)))*0.0471238898)#rads/point
        self.lastsum+=p
        i=self.lastsum
        print ("kp is {0}. ki is {1}. kd is {2}. the pos is {3} the p is{4} the i is{5} the d is{6}.".format(self.kp,self.ki,self.kd,pos, p,i,d))
        return (p*self.kp+d*self.kd+i*self.ki)
    def find_wall(self):
    # if lidar data has not been received, do nothing
        if self.data == None:
            return 0

    ## TO DO: Find Alg for Wall Following ##
    
        """Lidar data is now stored in self.data, which can be accessed
        using self.data.ranges (in simulation, returns an array).
        Lidar data at an index is the distance to the nearest detected object
        self.data.ranges[0] gives the leftmost lidar point
        self.data.ranges[99] gives the rightmost lidar point
        self.data.ranges[49] gives the forward lidar point
        """
        tempAngle=self.pid()
        
    #returns the output of your alg, the new angle to drive in
        return tempAngle

if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()
