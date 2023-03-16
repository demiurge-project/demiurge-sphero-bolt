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
from std_msgs.msg import ColorRGBA, Float32, Bool
from sensor_msgs.msg import Illuminance, Imu

### Other libraries
import math
import numpy

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

    ### Modules parameters
    DARK_THRESHOLD: float = 40
    BRIGHT_THRESHOLD: float = 500

    ## States

    B_RANDOM_WALK: int = 0 
    B_IDLE: int = 1
    B_FOLLOW: int = 2
    B_BROADCAST: int = 3
    B_SEARCH: int = 4

    ## Transitions

    T_SHAKE: int = 0
    T_DARK: int = 1
    T_BRIGHT: int = 2
    T_SIGNAL: int = 3
    T_RESTART: int = 4
    T_STOP: int = 5
    T_RANDOM_S: int = 6
    T_RANDOM_L: int = 7

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
        self.ir_signal      = Bool()
        self.restart        = Bool()
        self.stop           = Bool()
        self.follow         = Bool()
        self.broadcast      = Bool()

        # FSM configuration

        # Init state
        self.state = self.B_IDLE

        # Transitions 
        self.transitions = {

            # Outgoing transitions RANDOM_WALK (transition -> state)
            self.B_RANDOM_WALK: {
                self.T_DARK: self.B_BROADCAST,
                self.T_SHAKE: self.B_FOLLOW,
                self.T_RANDOM_L: self.B_SEARCH,
                self.T_SIGNAL: self.B_FOLLOW,
                self.T_STOP: self.B_IDLE
            },

            # Outgoing transitions IDLE (transition -> state)
            self.B_IDLE: {
                self.T_SHAKE: self.B_RANDOM_WALK,
                self.T_RESTART: self.B_RANDOM_WALK
            }, 
            
            # Outgoing transitions FOLLOW (transition -> state)
            self.B_FOLLOW: {
                self.T_SHAKE: self.B_RANDOM_WALK,
                #self.T_RANDOM: self.B_RANDOM_WALK,
                self.T_STOP: self.B_IDLE,
                self.T_RESTART: self.B_RANDOM_WALK
            },

            # Outgoing transitions BROADCAST (transition -> state)
            self.B_BROADCAST: {
                self.T_SHAKE: self.B_RANDOM_WALK,
                #self.T_RANDOM: self.B_RANDOM_WALK, 
                self.T_STOP: self.B_IDLE,
                self.T_RESTART: self.B_RANDOM_WALK
            }, 

            # Outgoing transitions SEARCH (transition -> state)
            self.B_SEARCH: {
                self.T_SHAKE: self.B_RANDOM_WALK,
                self.T_SIGNAL: self.B_FOLLOW,
                self.T_RANDOM_L: self.B_RANDOM_WALK,            
            }
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
        
        # IR signal
        rospy.Subscriber('sphero/ir_signal', Bool,
                         self.ir_signal_callback, queue_size=1)
        
        # Restart
        rospy.Subscriber('sphero/restart', Bool,
                         self.restart_callback, queue_size=1)
        
        # Stop
        rospy.Subscriber('sphero/stop', Bool,
                         self.stop_callback, queue_size=1)

    ### Subscribers

    def illuminance_callback(self, msg) -> None:
        self.illuminance = msg

    def imu_callback(self, msg) -> None:
        self.imu = msg

    def velocity_callback(self, msg) -> None:
        self.encoders_vel = msg

    def vertical_acc_callback(self, msg) -> None:
        self.vertical_acc = msg

    def ir_signal_callback(self, msg) -> None:
        self.ir_signal = msg

    def restart_callback(self, msg) -> None:
        self.restart = msg

    def stop_callback(self, msg) -> None:
        self.stop = msg

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
        
        # Complete LED matrix
        self.pub_follow = rospy.Publisher('sphero/follow', 
                                               Bool, queue_size=1)
        
        # Complete LED matrix
        self.pub_broadcast = rospy.Publisher('sphero/broadcast', 
                                               Bool, queue_size=1)

    ### Publishers

    def publish_cmd_vel(self):
        self.pub_cmd_vel.publish(self.cmd_vel)

    def publish_leds(self):
        self.pub_led_front_rgb.publish(self.led_front_rgb)
        self.pub_led_back_rgb.publish(self.led_back_rgb)

    def publish_matrix(self):
        self.pub_led_matrix_rgb.publish(self.led_matrix_rgb)

    def publish_follow(self):
        self.pub_follow.publish(self.follow)

    def publish_broadcast(self):
        self.pub_broadcast.publish(self.broadcast)
    
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
        if index == self.T_SHAKE:
            return self.transition_shake()
        elif index == self.T_DARK:
            return self.transition_dark()
        elif index == self.T_BRIGHT:
            return self.transition_bright()
        elif index == self.T_SIGNAL:
            return self.transition_signal()
        elif index == self.T_RANDOM_S:
            return self.transition_random(0.01)
        elif index == self.T_RANDOM_L:
            return self.transition_random(0.2)
        elif index == self.T_RESTART:
            return self.transition_restart()
        elif index == self.T_STOP:
            return self.transition_stop()          
        return False
    
    def transition_shake(self):
        if (abs(self.imu.linear_acceleration.x) > self.GRAVITY
            or abs(self.imu.linear_acceleration.y) > self.GRAVITY):
            print("TRANSITION: SHAKE")
            self.ir_signal.data = False
            return True
        return False
    
    def transition_dark(self):
        if (self.illuminance.illuminance < self.DARK_THRESHOLD):
            print("TRANSITION: DARK")
            return True
        return False
    
    def transition_bright(self):
        if (self.illuminance.illuminance > self.BRIGHT_THRESHOLD):
            print("TRANSITION: BRIGHT")
            return True
        return False
    
    def transition_signal(self):
        if (self.ir_signal.data == True):
            self.ir_signal.data = False
            print("TRANSITION: SIGNAL")
            return True
        return False
    
    def transition_random(self, p_eval):
        resolve =  numpy.random.choice(numpy.arange(0, 2), 
                                       p=[1-p_eval, p_eval])
        if resolve == True:
            print("TRANSITION: RANDOM")
        return  resolve
    
    def transition_restart(self):
        if (self.restart.data == True):
            self.restart.data = False
            print("TRANSITION: RESTART")
            return True
        return False
    
    def transition_stop(self):
        if (self.stop.data == True):
            self.stop.data = False
            print("TRANSITION: STOP")
            return True
        return False
    
    ### States
    
    def evaluate_state(self, index):
        if index == self.B_RANDOM_WALK:
            self.state_randomwalk()
        elif index == self.B_IDLE:
            self.state_idle()
        elif index == self.B_FOLLOW:
            self.state_follow()
        elif index == self.B_BROADCAST:
            self.state_broadcast()
        elif index == self.B_SEARCH:
            self.state_search()
        return False
    
    def state_randomwalk(self):
        print("STATE: RANDOM WALK")
        self.led_front_rgb.r = 0
        self.led_front_rgb.g = 25
        self.led_front_rgb.b = 0
        self.follow = False
        self.broadcast = False

    def state_idle(self):
        print("STATE: IDLE")
        self.led_front_rgb.r = 25
        self.led_front_rgb.g = 0
        self.led_front_rgb.b = 0
        self.follow = False
        self.broadcast = False
        self.clear_sphero_velocity()
        
    def state_follow(self):
        print("STATE: FOLLOW")
        self.led_front_rgb.r = 0
        self.led_front_rgb.g = 25
        self.led_front_rgb.b = 25
        self.broadcast = False
        self.follow = True

    def state_broadcast(self):
        print("STATE: BROADCAST")
        self.led_front_rgb.r = 25
        self.led_front_rgb.g = 25
        self.led_front_rgb.b = 0
        self.follow = False
        self.broadcast = True

    def state_search(self):
        print("STATE: SEARCH")
        self.led_front_rgb.r = 25
        self.led_front_rgb.g = 0
        self.led_front_rgb.b = 25
        self.broadcast = False
        self.follow = True

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
        self.publish_follow()
        self.publish_broadcast()

### Main             
        
if __name__ == "__main__":

    try:
        sphero_control = SpheroControl()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Keyboard interrupted")
        exit()

