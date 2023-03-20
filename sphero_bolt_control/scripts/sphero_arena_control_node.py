#!/usr/bin/env python3

"""sphero_arena_control_node.py: a script to 
control MoCA with the input of the Sphero BOLT 
robot. Based on the the Robot Operating System (ROS).
The node is a match for sphero_bolt_control_node.py
"""

### ROS libraries
import rospy
from std_msgs.msg import ColorRGBA, Bool

### Other libraries
import os

### Credits
__author__     = "David Garzón Ramos"
__copyright__  = "Copyright 2023, IRIDIA - Université libre de Bruxelles"
__credits__    = ["David Garzón Ramos", "Florian Noussa Yao", "Mauro Birattari"]
__license__    = "MIT"
__version__    = "0.1"
__maintainer__ = "David Garzón Ramos"
__email__      = "david.garzon.ramos@ulb.be"
__status__     = "Prototype"

class SpheroArenaControl():

    ### Constants

    PERIOD_CONTROL_CALLBACK = 0.05

    ### Modules parameters
    STATE_T_FILTER = 4

    ## States

    B_BALLISTIC: int = 0 
    B_IDLE: int = 1
    B_ROTATE: int = 2
    B_AGGREGATE: int = 3
    B_SPREAD: int = 4 

    ## Transitions

    T_SHAKE: int = 0 
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
        rospy.init_node("sphero_arena_control_node")

        self.restart        = Bool()
        self.stop           = Bool()
        self.master_m_state  = Bool()
        self.master_m_sleep  = Bool()
        self.t_last_trns     = rospy.get_time()
        self.trans_color     = False

        # FSM configuration

        # Init state
        self.state = self.B_IDLE

        # Transitions 
        self.transitions = {

            # Outgoing transitions RANDOM_WALK (transition -> state)
            self.B_BALLISTIC: {
                self.T_M_STATE: self.B_AGGREGATE,
                self.T_M_SLEEP: self.B_IDLE           
            },

            # Outgoing transitions IDLE (transition -> state)
            self.B_IDLE: {
                self.T_RESTART: self.B_BALLISTIC,
                self.T_M_STATE: self.B_BALLISTIC,
                self.T_M_SLEEP: self.B_IDLE 
            }, 

            # Outgoing transitions SEARCH (transition -> state)
            self.B_ROTATE: {
            },

            self.B_AGGREGATE: {
                self.T_STOP: self.B_IDLE,
                self.T_RESTART: self.B_BALLISTIC,
                self.T_M_STATE: self.B_SPREAD,
                self.T_M_SLEEP: self.B_IDLE             
            },

            self.B_SPREAD: {
                self.T_STOP: self.B_IDLE,
                self.T_RESTART: self.B_BALLISTIC,
                self.T_M_STATE: self.B_BALLISTIC,
                self.T_M_SLEEP: self.B_IDLE
            }      
        }

        self.setup_sphero_parameters()

    ### Setup BOLT 

    def setup_sphero_parameters(self) -> None:       

        self.create_ros_subscribers()

        # Control update timer
        rospy.Timer(
            rospy.Duration(self.PERIOD_CONTROL_CALLBACK), self.control_loop_callback)

    ### Create subscribers

    def create_ros_subscribers(self) -> None:
        
        # Master state change  
        rospy.Subscriber('/master/sphero/state_change', Bool,
                         self.master_state_callback, queue_size=1)

        # Master sleep trigger  
        rospy.Subscriber('/master/sphero/sleep', Bool,
                         self.master_sleep_callback, queue_size=1)
        

    ### Subscribers

    def master_state_callback(self, msg) -> None:
        self.master_m_state = msg

    def master_sleep_callback(self, msg) -> None:
        self.master_m_sleep = msg
    
    ### Utils
    
    def clear_arena_leds(self):
        cmd = "curl -X POST -H \"Content-Type: application/json\" -d \'{\"arena\":{\"edges\":1,\"blocks\":1,\"leds\":216,\"color\":\"omit\",\"brightness\": 95,\"block\":[{\"index\":[1],\"color\":\"none\"}]}}\' http://localhost:8080/arena-handler/api/v1.0/state"'ls -l'
        os.system(cmd)

    ### Transitions

    def evaluate_transition(self, index):
        if index == self.T_RESTART:
            return self.transition_restart()
        if index == self.T_STOP:
            return self.transition_stop() 
        elif index == self.T_M_STATE:
            return self.transition_master_state()
        elif index == self.T_M_SLEEP:
            return self.transition_master_sleep()  
        return False
    
    def transition_restart(self):
        if (self.restart.data == True):
            self.restart.data = False
            self.trans_color = False
            print("TRANSITION: RESTART")
            return True
        return False
    
    def transition_stop(self):
        if (self.stop.data == True):
            self.stop.data = False
            self.trans_color = False
            print("TRANSITION: STOP")
            return True
        return False
    
    def transition_master_state(self):
        if (self.master_m_state.data == True and 
                rospy.get_time() - self.t_last_trns > self.STATE_T_FILTER):
            self.master_m_state.data = False
            self.t_last_trns = rospy.get_time()
            self.trans_color = False
            print("TRANSITION: MASTER STATE")
            return True
        return False

    def transition_master_sleep(self):
        if (self.master_m_sleep.data == True and 
                rospy.get_time() - self.t_last_trns > self.STATE_T_FILTER):
            self.master_m_sleep.data = False
            self.t_last_trns = rospy.get_time()
            self.trans_color = False
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
        print("STATE: BALLISTIC")
        if self.trans_color == False:
            cmd = "curl -X POST -H \"Content-Type: application/json\" -d \'{\"arena\":{\"edges\":1,\"blocks\":1,\"leds\":216,\"color\":\"omit\",\"brightness\": 95,\"block\":[{\"index\":[1],\"color\":\"green\"}]}}\' http://localhost:8080/arena-handler/api/v1.0/state"
            os.system(cmd)
            self.trans_color = True

    def state_idle(self):
        print("STATE: IDLE")
        if self.trans_color == False:
            cmd = "curl -X POST -H \"Content-Type: application/json\" -d \'{\"arena\":{\"edges\":1,\"blocks\":1,\"leds\":216,\"color\":\"omit\",\"brightness\": 95,\"block\":[{\"index\":[1],\"color\":\"red\"}]}}\' http://localhost:8080/arena-handler/api/v1.0/state"
            os.system(cmd)
            self.trans_color = True
        
    def state_aggregate(self):
        print("STATE: AGGREGATE")
        if self.trans_color == False:
            cmd = "curl -X POST -H \"Content-Type: application/json\" -d \'{\"arena\":{\"edges\":1,\"blocks\":1,\"leds\":216,\"color\":\"omit\",\"brightness\": 95,\"block\":[{\"index\":[1],\"color\":\"cyan\"}]}}\' http://localhost:8080/arena-handler/api/v1.0/state"
            os.system(cmd)
            self.trans_color = True

    def state_spread(self):
        print("STATE: SPREAD")
        if self.trans_color == False:
            cmd = "curl -X POST -H \"Content-Type: application/json\" -d \'{\"arena\":{\"edges\":1,\"blocks\":1,\"leds\":216,\"color\":\"omit\",\"brightness\": 95,\"block\":[{\"index\":[1],\"color\":\"magenta\"}]}}\' http://localhost:8080/arena-handler/api/v1.0/state"
            os.system(cmd)
            self.trans_color = True

    def state_rotate(self):
        print("STATE: ROTATE")
        return 

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

### Main             
        
if __name__ == "__main__":

    try:
        sphero_arena_control = SpheroArenaControl()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Keyboard interrupted")
        exit()