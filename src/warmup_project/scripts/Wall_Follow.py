#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

import cv2
import math
import numpy

dist_to_wall = 1
desired_angle = 90
target = 1.0
pub = None

mode = "wall-follo"

def scan_received(msg):
    """ Callback function for msg of type sensor_msgs/LaserScan """
    global distance_to_wall
    if len(msg.ranges) != 360:
        print 'unexpcted laser scan message, not correct length'
        return

    if mode == "wall-follow":
        vel_cmd = wall_follow(msg)
    else: vel_cmd = obstacle_avoid(msg)
    pub.publish(vel_cmd)

def wall_follow(msg):
    # follows wall counter clockwise

    valid_distances = []
    valid_angles = []
    for i in range(360):
        if msg.ranges[i] > 0.1 and msg.ranges[i] < 7.0:
            valid_distances.append(msg.ranges[i])
            valid_angles.append(i)

    if len(valid_distances) < 3:
        print "wall_follow: no valid distances"
        return Twist(angular=Vector3(z=0),linear=Vector3(x=0))

    closest_object = min(valid_distances)
    closest_object_angle = valid_angles[valid_distances.index(closest_object)]
    
    # determines P angle coefficient based on how far the closest part of the wall is to 90 point of the robot
    P_ang = (-desired_angle+closest_object_angle) * math.pi / 180 # convert degrees to radians
    
    # handles angle wrapping
    if P_ang > math.pi: P_ang = P_ang-(math.pi)
    elif P_ang < -math.pi: P_ang = P_ang +(math.pi)

    # determies P coefficient based on how close the robot is to the wall
    P_dist = (closest_object - dist_to_wall)* math.pi/5
    if P_dist > math.pi: P_dist = P_dist-(math.pi)
    elif P_dist < -math.pi: P_dist = P_dist +(math.pi)

    # sends turning command to robot based on the combined desire to be parallel to the wall and only 1 meter to wall
    return Twist(angular=Vector3(z=P_ang+P_dist),linear=Vector3(x=.3))

def obstacle_avoid(msg):
    # Robot continues on strait path but slightly veers to avoid object

    object_distances = []
    first_angle = -1
    last_angle = 0

    create_object = False
    object_list = [] 
    

    for i in range(360):
        wrapping_case = False
        if msg.ranges[i] > 0.1 and msg.ranges[i] < 7.0:  # test if current point is valid measurment
            cart_point = polar_to_cart(msg.ranges[i],i)  # convert first point to cartesian
            if first_angle == -1: 
                first_angle = i  # if first point of object, set variable
                wrapping_case = True
            if i == 359: next_point = msg.ranges[0] # test if wrapping case
            else: next_point = msg.ranges[i+1]
            if next_point > 0.1 and next_point < 7.0: # test if next point is a valid measurment
                cart_point_next = polar_to_cart(msg.ranges[i],i) # convert second point to cartesian
                object_distances.append(msg.ranges[i])
                if dist_btw_points(cart_point, cart_point_next) > .3:
                    last_angle = i
                    create_object = True
            else: # if next point is not valid, end object
                create_object = True
        if create_object == True and len(object_distances) > 1:
            center_ang = (last_angle-first_angle)/2 + first_angle
            if wrapping_case == True:   # Deals with wrapping case for option by adjusting information from first object instead of adding new one
                old_width = object_list[0].width
                width = (360 - first_angle) + old_width
                center_angle = object_list[0].center_angle - (360 - first_angle)/2
                if center_angle < 0: center_angle = center_angle + 360
                if min(object_distances) < object_list[0].closest_point: object_list[0].closest_point = min(object_distances)
                object_list[0].width = width
                object_list[0].center_angle = center_angle

            else:   # create new object object
                obj = Object(avg_Dist = numpy.mean(object_distances), closest_point = min(object_distances), 
                             width = (last_angle - first_angle), center_angle = center_ang) #create object object
                object_list.append(obj)
            # reset necessary variables
            first_angle = -1
            create_object = False
    
    emergency_list = [] # list for objects in danger zone in front of robot
    for o in object_list:
        # if the object is in the trajectory, add to list
        if o.center_angle < 15 or o.center_angle > 360-15:
            emergency_list.append([o.avg_Dist,o.center_angle,o.width])

    # order object information by closest object in danger zone
    if emergency_list == []:
        return Twist(angular=Vector3(z=0),linear=Vector3(x=.3))

    emergency_list.sort()
    # Distance of center angle of object to center trajectory of robot
    # the 15 is so that the closer the object is 0, the larger the P_angle is
    if emergency_list[0][1] > 15:
        P_angle = emergency_list[0][1] - 360
    else: P_angle = - 15 - emergency_list[0][1]


    P_angle_rad = P_angle * math.pi / 180  # convert to radians

    # turn away from closest object
    return Twist(angular=Vector3(z= P_angle_rad),linear=Vector3(x=.2))


def dist_btw_points(point1,point2):
    #calculates the distance between two cartesian points
    return math.sqrt((point2[0]-point1[0])**2 + (point2[1]-point1[1])**2)


def polar_to_cart(r,theta):
    return (math.cos(theta)*r,math.sin(theta)*r)

def cart_to_polar(x,y):
    return (math.atan(y/x), math.sqrt(x**2 + y**2))


def set_target_distance(new_distance):
    """ call back function for the OpenCv Slider to set the target distance """
    global target
    target = new_distance/100.0

def wall_withslider():
    global pub
    """ Main run loop for wall with slider """
    cv2.namedWindow('UI')
    cv2.createTrackbar('distance', 'UI', int(target*100), 300, set_target_distance)
    rospy.init_node('approach_wall', anonymous=True)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('scan', LaserScan, scan_received)
    print "hello"
    r = rospy.Rate(10)
    while not(rospy.is_shutdown()):
        #if distance_to_wall != -1:
        cv2.waitKey(10)
        r.sleep()

class Object():
    def __init__(self, avg_Dist, closest_point, width, center_angle):
        self.avg_Dist = avg_Dist
        self.closest_point = closest_point
        self.width = width
        self.center_angle = center_angle


        
if __name__ == '__main__':
    try:
        wall_withslider()
    except rospy.ROSInterruptException: pass
