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

dist_to_wall = 1
desired_angle = 90
target = 1.0
pub = None

mode = "wall-follow"

def scan_received(msg):
    """ Callback function for msg of type sensor_msgs/LaserScan """
    global distance_to_wall
    if len(msg.ranges) != 360:
        print 'unexpcted laser scan message, not correct length'
        return

    if mode == "wall-follow":
        vel_cmd = wall_follwow(msg)
    else: vel_cmd = obstacle_avoid(msg)
    pub.publish(vel_cmd)

def wall_follow(msg):
    valid_distances = []
    valid_angles = []
    for i in range(360):
        if msg.ranges[i] > 0.1 and msg.ranges[i] < 7.0:
            valid_distances.append(msg.ranges[i])
            valid_angles.append(i)

    closest_object = min(valid_distances)
    closest_object_angle = valid_angles[valid_distances.index(closest_object)]
    
    P_ang = (-desired_angle+closest_object_angle) * math.pi / 180 # convert degrees to radians
    
    #handles angle wrapping
    if P_ang > math.pi: P_ang = P_ang-(math.pi)
    elif P_ang < -math.pi: P_ang = P_ang +(math.pi)

    P_dist = (closest_object - dist_to_wall)* math.pi/5
    if P_dist > math.pi: P_dist = P_dist-(math.pi)
    elif P_dist < -math.pi: P_dist = P_dist +(math.pi)
    print P_dist

    return Twist(angular=Vector3(z=P_ang+P_dist),linear=Vector3(x=.3))

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
        
if __name__ == '__main__':
    try:
        wall_withslider()
    except rospy.ROSInterruptException: pass
