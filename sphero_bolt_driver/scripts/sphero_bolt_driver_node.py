#!/usr/bin/env python3

# ROS libraries
import rospy
from geometry_msgs.msg import Vector3, Twist
from std_msgs.msg import Float64

# Sphero libraries
from spherov2 import scanner
from spherov2.sphero_edu import SpheroEduAPI, EventType
from spherov2.types import Color

# Other libraries
import numpy
import random
import math

# Pre-defined global variables

_max_heading = 30
_max_speed = 30
_target_heading = 0
_target_speed = 0


# Insert your code inside the function process_data
def process_data():
    # You can acces the value of the following variables:
    # If pressed key (q) _qwe.x = 1.0 ; (w) _qwe.y = 1.0; (e) _qwe.z = 1.0. Otherwise, they all equal to 0.0
    # If pressed key (a) _asd.x = 1.0 ; (s) _asd.y = 1.0; (d) _asd.z = 1.0. Otherwise, they all equal to 0.0
    # If click in the interface, _pose.x = vertical coordinate ;  _pose.y = horizontal coordinate. Otherwise, they all equal to -1.0
    # If click out of the interface, or the mouse leaves the interface, _pose.x = _pose.y = -1.0

    global _qwe, _asd, _pose, _cmd_vel

    # Outputs: replace the value 0.0 with your output
    # linear velocity, [-3.0,3.0] (+-1.5 m/s)
    _cmd_vel.linear.x = 0.0
    # angular velocity, [-6.0,6.0] (+-3.0 rad/s)
    _cmd_vel.angular.z = 0.0
    # camera pan, [-1.57,1.57] (rad)
    _cam_pan.data = 0.0
    # camera tilt, [-1.57,1.57] (rad)
    _cam_tilt.data = 0.0

    # Cleaning keyboard input data after being processed, if you want
    # to keep it, you must store it in a global variable.
    # Other variables are not cleaded here.
    clean_input()

##############################################################
##############################################################

def update_state():
    global _qwe, _asd, _pose, _cmd_vel

# Cleans data after being processed
def clean_input():
    global _target_heading, _target_speed
    print(_target_heading)
    print(_target_speed)
    _target_heading = 0
    _target_speed = 0

# Define callbacks and functions to process incoming messages
def cmd_vel_callback(msg):
    global _target_heading, _target_speed, _max_heading, _max_speed

    if -_max_heading <= msg.angular.z <= _max_heading:
        _target_heading = msg.angular.z
    else :
        if msg.angular.z > 0 :
            _target_heading = _max_heading
        else :
            _target_heading = -_max_heading

    if -_max_speed <= msg.linear.x <= _max_speed:
        _target_speed = msg.linear.x
    else :
        if msg.angular.z > 0 :
            _target_speed = _max_speed
        else :
            _target_speed = -_max_speed


def asd_callback(msg):
    global _asd
    _asd = msg

def pose_callback(msg):
    global _pose
    _pose = msg

    
# Main

if __name__ == '__main__':

    # Add here the name of the ROS node. In ROS, node names must be unique.
    rospy.init_node('sphero_bolt_driver_node')

    # Subscribe to the topics and associate the corresponding callback functions
    sub_target_speed = rospy.Subscriber('sphero/cmd_vel', Twist, cmd_vel_callback)
    #sub_asd = rospy.Subscriber('robot/teleoperation/key_asd/', Vector3, asd_callback)
    #sub_pose = rospy.Subscriber('robot/teleoperation/mouse_pose/', Vector3, pose_callback)

    # Publish messages
    #pub_vel = rospy.Publisher('/robot/cmd_vel/', Twist, queue_size=10)
    #pub_pan = rospy.Publisher('/robot/joint_pan_position_controller/command', Float64, queue_size = 1)
    #pub_tilt = rospy.Publisher('/robot/joint_tilt_position_controller/command', Float64, queue_size = 1)

    toy = scanner.find_toy(toy_name="SB-31F6")

    with SpheroEduAPI(toy) as _droid:
        
        rate=rospy.Rate(10)

        while not rospy.is_shutdown():

            _droid.reset_aim()

            frontLed_color = Color(random.randint(0, 255), 0, random.randint(0, 255))
            
            _droid.set_front_led(frontLed_color)
            _droid.set_heading(round(_target_heading))
            _droid.set_speed(round(_target_speed))

            #pub_vel.publish(_cmd_vel)
            #pub_pan.publish(_cam_pan)
            #pub_tilt.publish(_cam_tilt)

            rate.sleep()
