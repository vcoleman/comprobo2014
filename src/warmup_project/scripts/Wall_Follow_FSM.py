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

# pos is right hand rule

import rospy 
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
import cv2

distance_to_wall = -1
target = 1.0
pub = None

class Wall_Find():
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome_wall_found','outcome_user_end'])

    def execute(self, userdata):
        pass

class Wall_Aquire():
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome_wall_aquired','outcome_wall_gone'])

    def execute(self, userdata):
        pass

class Wall_Follow():
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome_user_end','outcome_wall_gone'])

    def execute(self, userdata):
        pass

class Teleop():
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome_user_start'])

    def execute(self, userdata):
        pass

def scan_received(msg):
    """ Callback function for msg of type sensor_msgs/LaserScan """
    global distance_to_wall
    if len(msg.ranges) != 360:
        print 'unexpcted laser scan message, not correct length'
        return

    valid_msgs = 0.0
    sum_valid = 0.0
    for i in range(5):
        if msg.ranges[i] > 0.1 and msg.ranges[i] < 7.0:
            valid_msgs += 1
            sum_valid += msg.ranges[i]
            print msg.ranges[i]
    if valid_msgs > 0:
        distance_to_wall = sum_valid / valid_msgs
        pub.publish(Twist(linear=Vector3(x=.4*(distance_to_wall-target))))
    else:
        distance_to_wall = -1

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
    r = rospy.Rate(10)
    while not(rospy.is_shutdown()):
        #if distance_to_wall != -1:
        cv2.waitKey(10)
        r.sleep()

def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome_kill'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('WALL_FIND', Wall_Follow(), 
                               transitions={'outcome_wall_gone':'WALL_FIND', 
                                            'outcome_user_end':'TELEOP'})
        smach.StateMachine.add('WALL_AQUIRE', Wall_Aquire(), 
                               transitions={'outcome_wall_aquired':'WALL_FOLLOW',
                                            'outcome_wall_gone':''
                                            'outcome_user_end':'TELEOP'})

        smach.StateMachine.add('WALL_FOLLOW', Wall_Find(), 
                               transitions={'outcome_wall_found':'WALL_AQUIRE',
                                            'outcome_user_end':'TELEOP'})

        smach.StateMachine.add('TELEOP', Teleop(), 
                               transitions={'outcome_user_start':'WALL_FIND'})

    # Execute SMACH plan
    outcome = sm.execute()
        
# if __name__ == '__main__':
#     try:
#         wall_withslider()
#     except rospy.ROSInterruptException: pass

    '''

    #!/usr/bin/env python

import roslib#; roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros

# define state Foo
class Foo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state FOO')
        if self.counter < 5:
            self.counter += 1
            return 'outcome1'
        else:
            return 'outcome2'


# define state Bar
class Bar(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome2','outcome3'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state BAR')
        if self.counter < 2:
            self.counter += 1
            return 'outcome2'
        else:
            return 'outcome3'        



# main
def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4', 'outcome5'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('FOO', Foo(), 
                               transitions={'outcome1':'BAR', 
                                            'outcome2':'outcome4'})
        smach.StateMachine.add('BAR', Bar(), 
                               transitions={'outcome2':'FOO',
                                            'outcome3':'outcome5'})

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()

'''
