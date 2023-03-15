#!/usr/bin/env python3

"""sphero_bolt_control_node.py: a script to 
operate a Sphero BOLT robot with the Robot 
Operating System (ROS). The node is a match for
sphero_bolt_driver_node.py
"""

### ROS libraries
import rospy
import tf
from geometry_msgs.msg import Twist
from std_msgs.msg import ColorRGBA, Float32
from sensor_msgs.msg import Illuminance, Imu

### Other libraries
import math

### Credits
__author__     = "David Garzón Ramos"
__copyright__  = "Copyright 2023, IRIDIA - Université libre de Bruxelles"
__credits__    = ["David Garzón Ramos", "Florian Noussa Yao", "Mauro Birattari"]
__license__    = "MIT"
__version__    = "0.1"
__maintainer__ = "David Garzón Ramos"
__email__      = "david.garzon.ramos@ulb.be"
__status__     = "Prototype"

class SpheroControl():

    ### Constants

    PERIOD_CONTROL_CALLBACK: float = 0.25
    MAX_BOLT_SPEED: float = 1.5 # m/s
    MAX_BOLT_HEADING: float = math.pi # rad
    GRAVITY: float = 9.80665 # m/s2
    DARK_THRESHOLD: float = 100

    ## States

    RANDOM_WALK: int = 0 
    STOP: int = 1

    ## Transitions

    SHAKE: int = 0
    DARK: int = 1

    ### Init

    def __init__(self) -> None:

        # init ROS node
        rospy.init_node("sphero_bolt_control_node")

        self.cmd_vel        = Twist()
        self.led_front_rgb  = ColorRGBA()
        self.led_back_rgb   = ColorRGBA()
        self.led_matrix_rgb = ColorRGBA()
        self.illuminance    = Illuminance()
        self.imu            = Imu()
        self.encoders_vel   = Twist()
        self.vertical_acc   = Float32()

        # FSM configuration

        # Init state
        self.state = self.RANDOM_WALK

        # Transitions 
        self.transitions = {

            # Outgoing transitions RANDOM_WALK (transition -> state)
            self.RANDOM_WALK: {
                self.DARK: self.STOP },

            # Outgoing transitions STOP (transition -> state)
            self.STOP: {
                self.SHAKE: self.RANDOM_WALK }
        }

        self.setup_sphero_parameters()

    ### Setup BOLT 

    def setup_sphero_parameters(self) -> None: 
        
        # LEDs parameters
        init_leds = rospy.get_param('init_leds', False)
        init_matrix = rospy.get_param('init_matrix', False)
    
        init_led_front_rgb = rospy.get_param('led_front_rgb', [0, 0, 0])
        self.led_front_rgb.r = init_led_front_rgb[0]
        self.led_front_rgb.g = init_led_front_rgb[1]
        self.led_front_rgb.b = init_led_front_rgb[2]

        init_led_back_rgb  = rospy.get_param('led_back_rgb', [0, 0, 0])
        self.led_back_rgb.r = init_led_back_rgb[0]
        self.led_back_rgb.g = init_led_back_rgb[1]
        self.led_back_rgb.b = init_led_back_rgb[2]

        init_led_matrix_rgb = rospy.get_param('led_matrix_rgb', [0, 0, 0])
        self.led_matrix_rgb.r = init_led_matrix_rgb[0]
        self.led_matrix_rgb.g = init_led_matrix_rgb[1]
        self.led_matrix_rgb.b = init_led_matrix_rgb[2]

        # TODO Self-calibrate Illuminance 
        # Rotate, register, save the value.         

        self.create_ros_publishers()
        self.create_ros_subscribers()

        if init_matrix:
            self.publish_matrix()

        if init_leds:
            self.publish_leds()

        # Control update timer
        rospy.Timer(
            rospy.Duration(self.PERIOD_CONTROL_CALLBACK), self.control_loop_callback)

    ### Create subscribers

    def create_ros_subscribers(self) -> None:

        # Ambient light
        rospy.Subscriber('sphero/illuminance', Illuminance, 
                         self.illuminance_callback, queue_size=1)
        # IMU
        rospy.Subscriber('sphero/imu', Imu, 
                         self.imu_callback, queue_size=1)      
        # Encoders velocity
        rospy.Subscriber('sphero/velocity', Twist,
                         self.velocity_callback, queue_size=1)
        
        # Vertical acceleration  
        rospy.Subscriber('sphero/vertical_acc', Float32,
                         self.vertical_acc_callback, queue_size=1)

    ### Subscribers

    def illuminance_callback(self, msg) -> None:
        self.illuminance = msg

    def imu_callback(self, msg) -> None:
        self.imu = msg

    def velocity_callback(self, msg) -> None:
        self.encoders_vel = msg

    def vertical_acc_callback(self, msg) -> None:
        self.vertical_acc = msg

    ### Create publishers

    def create_ros_publishers(self) -> None:

        # Velocity
        self.pub_cmd_vel = rospy.Publisher('sphero/cmd_vel', 
                                               Twist, queue_size=1)
        
        # Front LED
        self.pub_led_front_rgb = rospy.Publisher('sphero/led_front_rgb', 
                                               ColorRGBA, queue_size=1)
        
        # Back LED
        self.pub_led_back_rgb = rospy.Publisher('sphero/led_back_rgb', 
                                               ColorRGBA, queue_size=1)
        
        # Complete LED matrix
        self.pub_led_matrix_rgb = rospy.Publisher('sphero/led_matrix_rgb', 
                                               ColorRGBA, queue_size=1)

    ### Publishers

    def publish_cmd_vel(self):
        self.pub_cmd_vel.publish(self.cmd_vel)

    def publish_leds(self):
        self.pub_led_front_rgb.publish(self.led_front_rgb)
        self.pub_led_back_rgb.publish(self.led_back_rgb)

    def publish_matrix(self):
        self.pub_led_matrix_rgb.publish(self.led_matrix_rgb)
    
    ### Utils
    
    def clear_sphero_velocity(self):
        self.cmd_vel = Twist()

    def clear_sphero_leds(self):
        self.led_front_rgb = ColorRGBA()
        self.led_back_rgb = ColorRGBA()

    def clear_sphero_matrix(self):
        self.led_matrix_rgb = ColorRGBA()

    ### Transitions

    def evaluate_transition(self, index):
        if index == self.SHAKE:
            return self.transition_shake()
        elif index == self.DARK:
            return self.transition_dark()            
        return False
    
    def transition_shake(self):
        if (abs(self.imu.linear_acceleration.x) > self.GRAVITY
            or abs(self.imu.linear_acceleration.y) > self.GRAVITY):
            return True
        return False
    
    def transition_dark(self):
        if (self.illuminance.illuminance < self.DARK_THRESHOLD):
            return True
        return False
    
    ### States
    
    def evaluate_state(self, index):
        if index == self.RANDOM_WALK:
            self.state_randomwalk()
        elif index == self.STOP:
            self.state_stop()
        return False
    
    def state_randomwalk(self):
        self.led_front_rgb.r = 0
        self.led_front_rgb.g = 25
        self.led_front_rgb.b = 0

    def state_stop(self):
        self.led_front_rgb.r = 25
        self.led_front_rgb.g = 0
        self.led_front_rgb.b = 0

    ### Controls 
    
    def iterate_fsm(self):
        for t in self.transitions[self.state]:
            condition_met = self.evaluate_transition(t)
            if (condition_met):
                self.state = self.transitions[self.state][t]
                break
        self.evaluate_state(self.state)

    def control_loop_callback(self, event=None):
        self.iterate_fsm()
        self.publish_cmd_vel()   
        self.publish_leds()
        self.publish_matrix()

### Main             
        
if __name__ == "__main__":

    try:
        sphero_control = SpheroControl()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Keyboard interrupted")
        exit()

