#!/usr/bin/env python3

"""sphero_bolt_control_node.py: a script to 
operate a Sphero BOLT robot with the Robot 
Operating System (ROS). The node is a match for
sphero_bolt_driver_node.py
"""

### ROS libraries
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import ColorRGBA, Float32, Bool
from sensor_msgs.msg import Illuminance, Imu

### Other libraries
import math
import numpy
import statistics

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
    C_RED = ColorRGBA(255, 0, 0, 1)
    C_GREEN = ColorRGBA(0, 255, 0, 1)
    C_BLUE = ColorRGBA(6, 1, 15, 1)
    C_CYAN = ColorRGBA(0, 255, 255, 1)
    C_MAGENTA = ColorRGBA(255, 0, 255, 1)
    C_YELLOW = ColorRGBA(255, 255, 0, 1)
    C_BLACK = ColorRGBA(0, 0, 0, 1)
    C_L_GREEN = ColorRGBA(1, 15, 6, 1)

    ### Modules parameters
    DARK_THRESHOLD: float = 20
    BRIGHT_THRESHOLD: float = 500
    SHAKE_THRESHOLD = 1.5 * GRAVITY
    MIN_ROTATION = 0.8* MAX_BOLT_HEADING
    BOLT_SPEED = 0.3* MAX_BOLT_SPEED
    STUCK_ACC_X = 0.75 * GRAVITY
    STUCK_ACC_y = 0.5 * GRAVITY
    STATE_T_FILTER = 4
    STATE_S_FILTER = 10
    DARK_T_THRESHOLD = 0.25
    SHAKE_T_THRESHOLD = 1
    N_SHAKES_THRESHOLD = 4

    ## States

    B_BALLISTIC: int = 0 
    B_IDLE: int = 1
    B_ROTATE: int = 2
    B_AGGREGATE: int = 3
    B_SPREAD: int = 4 

    ## Transitions

    T_SHAKE: int = 0 # TODO: add a transition that diff shake and stuck
    T_DARK: int = 1
    T_BRIGHT: int = 2
    T_RESTART: int = 3
    T_STOP: int = 4
    T_RANDOM_S: int = 5
    T_STUCK: int = 6 
    T_M_STATE: int = 7
    T_M_SLEEP: int = 8  

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
        self.restart        = Bool()
        self.stop           = Bool()
        self.aggregate      = Bool()
        self.spread         = Bool()
        self.master_m_state  = Bool()
        self.master_r_state  = Bool()
        self.master_m_sleep  = Bool()
        self.master_r_sleep  = Bool()
        self.t_last_trns     = rospy.get_time()
        self.t_last_shake    = rospy.get_time()
        self.t_last_dark     = 0
        self.in_dark         = False
        self.n_shakes        = 0

        self.acc_filter_points = [0,0,0,0,0,0,0,0]
        # FSM configuration

        # Init state
        self.state = self.B_IDLE

        # Transitions 
        self.transitions = {

            # Outgoing transitions RANDOM_WALK (transition -> state)
            self.B_BALLISTIC: {
                self.T_STUCK: self.B_ROTATE,
                #self.T_STOP: self.B_IDLE,
                self.T_SHAKE: self.B_BALLISTIC,
                self.T_M_STATE: self.B_AGGREGATE,
                self.T_DARK: self.B_BALLISTIC,
                self.T_M_SLEEP: self.B_IDLE           
            },

            # Outgoing transitions IDLE (transition -> state)
            self.B_IDLE: {
                self.T_RESTART: self.B_BALLISTIC,
                self.T_SHAKE: self.B_IDLE,
                self.T_M_STATE: self.B_BALLISTIC,
                self.T_DARK: self.B_IDLE,
                self.T_M_SLEEP: self.B_IDLE 
            }, 

            # Outgoing transitions SEARCH (transition -> state)
            self.B_ROTATE: {
                self.T_SHAKE: self.B_BALLISTIC,
                self.T_RANDOM_S: self.B_BALLISTIC,
                self.T_STOP: self.B_IDLE,
                self.T_RESTART: self.B_BALLISTIC,
                self.T_M_STATE: self.B_AGGREGATE,
                self.T_DARK: self.B_ROTATE,
                self.T_M_SLEEP: self.B_IDLE   
            },

            self.B_AGGREGATE: {
                self.T_STOP: self.B_IDLE,
                self.T_RESTART: self.B_BALLISTIC,
                self.T_SHAKE: self.B_AGGREGATE,
                self.T_M_STATE: self.B_SPREAD,
                self.T_DARK: self.B_AGGREGATE,
                self.T_M_SLEEP: self.B_IDLE             
            },

            self.B_SPREAD: {
                self.T_STOP: self.B_IDLE,
                self.T_RESTART: self.B_BALLISTIC,
                self.T_SHAKE: self.B_SPREAD,
                self.T_M_STATE: self.B_BALLISTIC,
                self.T_DARK: self.B_SPREAD,
                self.T_M_SLEEP: self.B_IDLE
            }      
        }

        self.setup_sphero_parameters()

    ### Setup BOLT 

    def setup_sphero_parameters(self) -> None: 

        init_leds = rospy.get_param('robot_no', False)       

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

        self.illuminance.illuminance = 300

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

        # Restart
        rospy.Subscriber('sphero/restart', Bool,
                         self.restart_callback, queue_size=1)
        
        # Master state change  
        rospy.Subscriber('/master/sphero/state_change', Bool,
                         self.master_state_callback, queue_size=1)

        # Master sleep trigger  
        rospy.Subscriber('/master/sphero/sleep', Bool,
                         self.master_sleep_callback, queue_size=1)
        

    ### Subscribers

    def illuminance_callback(self, msg) -> None:
        self.illuminance = msg

    def imu_callback(self, msg) -> None:
        self.imu = msg

    def velocity_callback(self, msg) -> None:
        self.encoders_vel = msg

    def vertical_acc_callback(self, msg) -> None:
        self.vertical_acc = msg

    def restart_callback(self, msg) -> None:
        self.restart = msg

    def stop_callback(self, msg) -> None:
        self.stop = msg

    def master_state_callback(self, msg) -> None:
        self.master_m_state = msg

    def master_sleep_callback(self, msg) -> None:
        self.master_m_sleep = msg

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
        
        # Aggregation
        self.pub_aggregate = rospy.Publisher('sphero/aggregate', 
                                               Bool, queue_size=1)
        
        # Dispersion
        self.pub_spread = rospy.Publisher('sphero/spread', 
                                               Bool, queue_size=1)

        # New state
        self.pub_r_state = rospy.Publisher('/master/sphero/state_change', 
                                               Bool, queue_size=1)

        # New state
        self.pub_r_sleep = rospy.Publisher('/master/sphero/sleep', 
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
    
    def publish_aggregate(self):
        self.pub_aggregate.publish(self.aggregate)

    def publish_spread(self):
        self.pub_spread.publish(self.spread)
    
    ## For Master
    def publish_r_state(self):
        self.pub_r_state.publish(self.master_r_state)

    def publish_r_sleep(self):
        self.pub_r_sleep.publish(self.master_r_sleep)
    
    ### Utils
    
    def clear_sphero_velocity(self):
        self.cmd_vel = Twist()

    def clear_sphero_leds(self):
        self.led_front_rgb = self.C_BLACK
        self.led_back_rgb = self.C_BLACK

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
        elif index == self.T_RANDOM_S:
            return self.transition_random(0.3)
        elif index == self.T_RESTART:
            return self.transition_restart()
        elif index == self.T_STOP:
            return self.transition_stop()
        elif index == self.T_STUCK:
            return self.transition_stuck()     
        elif index == self.T_M_STATE:
            return self.transition_master_state()
        elif index == self.T_M_SLEEP:
            return self.transition_master_sleep()  
        return False
    
    def transition_shake(self):
        magnitude = abs(math.sqrt(self.imu.linear_acceleration.x**2 + 
                    self.imu.linear_acceleration.y**2 +
                    self.imu.linear_acceleration.z**2) - self.GRAVITY)
        if (magnitude > self.SHAKE_THRESHOLD):
            #self.t_last_shake = rospy.get_time()
            #and rospy.get_time() - self.t_last_shake < self.SHAKE_T_THRESHOLD):
            print("Number of shakes: ", self.n_shakes)
            if (rospy.get_time()-self.t_last_shake < self.SHAKE_T_THRESHOLD):
                self.n_shakes = self.n_shakes + 1
            else:
                self.n_shakes = 1

            if(self.n_shakes > self.N_SHAKES_THRESHOLD):
                #self.led_front_rgb = self.C_BLUE
                #self.t_last_shake = rospy.get_time()
                print("TRANSITION: SHAKE")
                self.master_r_state = True
                self.pub_r_state.publish(self.master_r_state)
                self.master_r_state = False
                self.n_shakes = 0
                self.t_last_shake = rospy.get_time()
                return True
            self.t_last_shake = rospy.get_time()
        return False
    
    def transition_dark(self):
        if (0 < self.illuminance.illuminance < self.DARK_THRESHOLD):
            print("IN THRESHOLD*****************")
            if (self.in_dark != True):
                print("START COUNT *****************")
                self.t_last_dark = rospy.get_time()
                self.in_dark = True
                return False
            print("***** Elapsed time: ", rospy.get_time() - self.t_last_dark)
            if (rospy.get_time() - self.t_last_dark > self.DARK_T_THRESHOLD):
                self.master_r_sleep = True                        
                self.pub_r_sleep.publish(self.master_r_sleep)
                self.master_r_sleep = False
                print("TRANSITION: DARK")
                self.t_last_dark = rospy.get_time()
                return True
        else:
            self.in_dark = False
            self.t_last_dark = rospy.get_time()
            return False
        return False
    
    def transition_bright(self):
        if (self.illuminance.illuminance > self.BRIGHT_THRESHOLD):
            print("TRANSITION: BRIGHT")
            return True
        return False
    
    def transition_random(self, p_eval):
        rnd =  numpy.random.choice(numpy.arange(0, 2),
                                    p=[1-p_eval, p_eval])
        if rnd == True:
            print("TRANSITION: RANDOM")
        return  rnd
    
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
    
    def transition_stuck(self):
        if (abs(self.encoders_vel.linear.x) < 1 or
            abs(self.imu.linear_acceleration.x) > self.STUCK_ACC_X or
            abs(self.imu.linear_acceleration.y) > self.STUCK_ACC_y):
                print("TRANSITION: STUCK")
                return True
        return False
    
    def transition_master_state(self):
        if (self.master_m_state.data == True and 
                rospy.get_time() - self.t_last_trns > self.STATE_T_FILTER):
            #self.led_front_rgb = self.C_L_GREEN
            self.master_m_state.data = False
            self.t_last_trns = rospy.get_time()
            print("TRANSITION: MASTER STATE")
            return True
        return False

    def transition_master_sleep(self):
        if (self.master_m_sleep.data == True and 
                rospy.get_time() - self.t_last_trns > self.STATE_T_FILTER):
            #self.led_front_rgb = self.C_L_GREEN
            self.master_m_sleep.data = False
            self.t_last_trns = rospy.get_time()
            print("TRANSITION: MASTER SLEEP")
            return True
        return False
    
    ### States
    
    def evaluate_state(self, index):
        if index == self.B_BALLISTIC:
            self.state_ballistic()
        elif index == self.B_IDLE:
            self.state_idle()
        elif index == self.B_ROTATE:
            self.state_rotate()
        elif index == self.B_AGGREGATE:
            self.state_aggregate()
        elif index == self.B_SPREAD:
            self.state_spread()
        return False
    
    def state_ballistic(self):
        #print("STATE: BALLISTIC")
        self.led_matrix_rgb = self.C_GREEN
        self.aggregate = False
        self.spread = False
        self.cmd_vel.linear.x = self.BOLT_SPEED
        self.cmd_vel.angular.z = 0

    def state_idle(self):
        #print("STATE: IDLE")
        self.led_matrix_rgb = self.C_RED
        self.aggregate = False
        self.spread = False
        self.clear_sphero_velocity()
        
    def state_aggregate(self):
        #print("STATE: AGGREGATE")
        self.led_matrix_rgb = self.C_CYAN
        self.follow = False
        self.broadcast = False
        self.aggregate = True
        self.spread = False 

    def state_spread(self):
        #print("STATE: SPREAD")
        self.led_matrix_rgb = self.C_MAGENTA
        self.follow = False
        self.broadcast = False
        self.aggregate = False
        self.spread = True

    def state_rotate(self):
        #print("STATE: ROTATE")
        self.led_matrix_rgb = self.C_YELLOW     
        rotate = numpy.random.uniform(-self.MAX_BOLT_HEADING, 
                                      self.MAX_BOLT_HEADING)
        if numpy.sign(rotate) >= 0:
            rotate = max(rotate, self.MIN_ROTATION)
        else:
            rotate = min(rotate, -self.MIN_ROTATION)

        self.cmd_vel.linear.x = 0 
        self.cmd_vel.angular.z = rotate 

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
        self.publish_aggregate()
        self.publish_spread()

### Main             
        
if __name__ == "__main__":

    try:
        sphero_control = SpheroControl()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Keyboard interrupted")
        exit()

