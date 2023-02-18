#!/usr/bin/env python3

# ROS libraries
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import Illuminance

# Sphero libraries
from spherov2 import scanner
from spherov2.sphero_edu import SpheroEduAPI
from spherov2.types import Color

# Sphero utils
import sb_utils

# Other libraries
import math

# Pre-defined global variables

_b_speed = 0.2
_b_heading = math.pi

_target_max_speed   = min(sb_utils.MAX_BOLT_VEL, max(-sb_utils.MAX_BOLT_VEL, _b_speed))  # Max BOLT speed 1.5 m/s
_target_max_heading = min(sb_utils.MAX_BOLT_HEADING, max(-sb_utils.MAX_BOLT_HEADING, _b_heading)) # Bounding target heading to [-pi, pi] rad

_target_speed   = 0
_target_heading = 0
_led_color_front = Color(0, 0, 0)
_led_color_back  = Color(0, 0, 0)

_illumniance_msg = Illuminance()

# Data should be moved to a robot state
def update_state():
    # Check if past speed is the same, if so, do not update
    # Heading might require continuous restart
    # Implement updates with a FSM
    return 

# Cleans data after being processed
def clean_input():
    # Check if past heading or past vel is the same, if so, do not update
    return 

# Define callbacks and functions to process incoming messages
def cmd_vel_callback(msg):
    global _target_speed, _target_heading
    _target_speed = ms_to_speed(msg.linear.x)
    _target_heading = rad_to_heading(msg.angular.z)
    return

def led_color_front_callback(msg):
    global _led_color_front
    _led_color_front = Color(round(msg.r), round(msg.g), round(msg.b))
    return

def led_color_back_callback(msg):
    global _led_color_back
    _led_color_back = Color(round(msg.r), round(msg.g), round(msg.b))
    return

# Util methods 

def ms_to_speed(linear_x):
    bound_speed = min(max(0, linear_x), _target_max_speed) 
    sphero_speed = round((bound_speed * 255)/sb_utils.MAX_BOLT_VEL)  
    return sphero_speed 
    
def rad_to_heading(angular_z):
    bound_heading = min(_target_max_heading, max(-_target_max_heading, angular_z))
    sphero_heading = round(math.degrees(bound_heading))
    return sphero_heading
    
# Main

if __name__ == '__main__':

    # Add here the name of the ROS node. In ROS, node names must be unique.
    rospy.init_node('sphero_bolt_driver_node')

    # Subscribe to the topics and associate the corresponding callback functions
    sub_target_velocity = rospy.Subscriber('sphero/cmd_vel', Twist, cmd_vel_callback)
    sub_led_color_front = rospy.Subscriber('sphero/led_color_front', ColorRGBA, led_color_front_callback)
    sub_led_color_back = rospy.Subscriber('sphero/led_color_back', ColorRGBA, led_color_back_callback)

    # Publish messages
    pub_illuminance = rospy.Publisher('sphero/illuminance', Illuminance, queue_size = 1)

    toy = scanner.find_toy(toy_name="SB-D760")

    with SpheroEduAPI(toy) as _droid:
        
        rate=rospy.Rate(1) # Robot stops when updating velocity, even to the same value

        while not rospy.is_shutdown():

            _droid.reset_aim()
            
            _droid.set_front_led(_led_color_front)
            _droid.set_back_led(_led_color_back)

            print(_droid.get_luminosity()) 
            print(_droid.get_luminosity_direct()) 

            _droid._SpheroEduAPI__speed = _target_speed
            _droid._SpheroEduAPI__heading =_target_heading
            _droid._SpheroEduAPI__update_speed()

            _illumniance_msg.illuminance = _droid.get_luminosity()['ambient_light']
            pub_illuminance.publish(_illumniance_msg)

            rate.sleep()
