#!/usr/bin/env python3

"""sphero_bolt_driver_node.py: a wrapper to 
operate a Sphero BOLT robot with the Robot 
Operating System (ROS) and the spherov2 
Python library.
"""

### ROS libraries
import rospy
import tf
from geometry_msgs.msg import Twist
from std_msgs.msg import ColorRGBA, Float32, Bool
from sensor_msgs.msg import Illuminance, Imu

### Sphero libraries
import os, sys
scriptPath = os.path.realpath(os.path.dirname(sys.argv[0]))
os.chdir(scriptPath)
sys.path.append("../../spherov2.py")
from spherov2 import scanner
from spherov2.sphero_edu import SpheroEduAPI
from spherov2.types import Color

### Other libraries
import math
import random

### Credits
__author__     = "David Garzón Ramos"
__copyright__  = "Copyright 2023, IRIDIA - Université libre de Bruxelles"
__credits__    = ["David Garzón Ramos", "Florian Noussa Yao", "Mauro Birattari"]
__license__    = "MIT"
__version__    = "0.1"
__maintainer__ = "David Garzón Ramos"
__email__      = "david.garzon.ramos@ulb.be"
__status__     = "Prototype"

class SpheroBolt():

    ### Constants

    PERIOD_PUBLISH_CALLBACK: float = 0.1
    PERIOD_CONTROL_CALLBACK: float = 0.25
    MAX_BOLT_SPEED = 1.5 # m/s
    MAX_BOLT_HEADING = math.pi # rad
    GRAVITY = 9.80665 # m/s2
    LED_MATRIX_BLACK = ["000000" for _ in range(64)]
    NO_SIGNAL: int = 00000

    ### Init

    def __init__(self) -> None:

        # init ROS node
        rospy.init_node("sphero_bolt_driver_node")

        self.target_speed   = 0
        self.target_heading   = 0
        self.led_color_front  = Color(0, 0, 0)
        self.led_color_back   = Color(0, 0, 0)
        self.follow         = False
        self.broadcast      = False
        self.ir_signal      = self.NO_SIGNAL
        self.aggregate      = False
        self.spread         = False

        self.setup_sphero_parameters()

    ### Setup BOLT 

    def setup_sphero_parameters(self) -> None: 

        #robot number
        robot_no = rospy.get_param('robot_no', 0)
        self.robot_no = robot_no

        # Velocities parameters
        self.boundary_speed     = float(rospy.get_param('speed_limit', 0.2))
        self.target_max_speed   = min(self.MAX_BOLT_SPEED, 
                                 max(-self.MAX_BOLT_SPEED, self.boundary_speed))  
        
        self.boundary_heading   = float(rospy.get_param('turning_limit', math.pi/2))
        self.target_max_heading = min(self.MAX_BOLT_HEADING, 
                                 max(-self.MAX_BOLT_HEADING, self.boundary_heading))
        
        # LEDs parameters
        init_leds = rospy.get_param('init_leds', False)
        init_matrix = rospy.get_param('init_matrix', False)
    
        init_led_front_rgb = rospy.get_param('led_front_rgb', [0, 0, 0])
        self.led_front_rgb = Color(init_led_front_rgb[0], init_led_front_rgb[1], init_led_front_rgb[2])

        init_led_back_rgb  = rospy.get_param('led_back_rgb', [0, 0, 0])
        self.led_back_rgb  = Color(init_led_back_rgb[0], init_led_back_rgb[1], init_led_back_rgb[2])

        init_led_matrix_rgb = rospy.get_param('led_matrix_rgb', [0, 0, 0])             
        self.led_matrix_rgb = Color(init_led_matrix_rgb[0], init_led_matrix_rgb[1], init_led_matrix_rgb[2])
        self.led_matrix_rgb_new = self.led_matrix_rgb
        
        init_led_matrix_hex  = rospy.get_param('led_matrix_hex', self.LED_MATRIX_BLACK)
        self.led_matrix_hex = init_led_matrix_hex

        if init_matrix:
            droid.set_main_led(self.led_matrix_rgb)
            self.set_sphero_matrix_drawing()

        if init_leds:
            self.set_sphero_leds()

        self.create_ros_publishers()
        self.create_ros_subscribers()

        # Publishing timer
        rospy.Timer(
            rospy.Duration(self.PERIOD_PUBLISH_CALLBACK), self.publisher_callback)

        # Control update timer
        rospy.Timer(
            rospy.Duration(self.PERIOD_CONTROL_CALLBACK), self.control_loop_callback)

    ### Create subscribers

    def create_ros_subscribers(self) -> None:

        # Velocity 
        rospy.Subscriber('sphero/cmd_vel', Twist, 
                         self.cmd_vel_callback, queue_size=1)
        # Front LED
        rospy.Subscriber('sphero/led_front_rgb', ColorRGBA, 
                         self.led_front_rgb_callback, queue_size=1)      
        # Back LED 
        rospy.Subscriber('sphero/led_back_rgb', ColorRGBA,
                         self.led_back_rgb_callback, queue_size=1)
        
        # Complete LED matrix  
        rospy.Subscriber('sphero/led_matrix_rgb', ColorRGBA,
                         self.led_matrix_rgb_callback, queue_size=1)
        
        # Complete LED matrix  
        rospy.Subscriber('sphero/follow', Bool,
                         self.follow_callback, queue_size=1)
        
        # Complete LED matrix  
        rospy.Subscriber('sphero/broadcast', Bool,
                         self.broadcast_callback, queue_size=1)
        
        # Aggregation 
        rospy.Subscriber('sphero/aggregate', Bool,
                         self.aggregate_callback, queue_size=1)
        
        # Dispersion  
        rospy.Subscriber('sphero/spread', Bool,
                         self.spread_callback, queue_size=1)
        
    ### Subscribers

    def cmd_vel_callback(self, msg) -> None:
        self.target_speed = self.ms_to_speed(msg.linear.x)
        self.target_heading = self.rad_to_heading(msg.angular.z)

    def led_front_rgb_callback(self, msg) -> None:
        self.led_front_rgb = Color(round(msg.r), round(msg.g), round(msg.b))

    def led_back_rgb_callback(self, msg) -> None:
        self.led_back_rgb = Color(round(msg.r), round(msg.g), round(msg.b))

    def led_matrix_rgb_callback(self, msg) -> None:
        self.led_matrix_rgb_new = Color(round(msg.r), round(msg.g), round(msg.b))

    def follow_callback(self, msg) -> None:
        if(msg.data != self.follow):
            if (msg.data == True):
                droid.reset_aim()
                self.clear_sphero_velocity()                
                droid.start_ir_follow(6,7)
                self.follow = True
            else:
                droid.reset_aim()
                droid.stop_ir_follow()
                self.follow = False

    def broadcast_callback(self, msg) -> None:
        if(msg.data != self.broadcast):
            if (msg.data == True):
                droid.reset_aim()
                droid.start_ir_broadcast(6,7)
                self.broadcast = True
            else:
                droid.reset_aim()
                droid.stop_ir_broadcast()
                self.broadcast = False
    
    def aggregate_callback(self, msg) -> None:
        val = int(self.robot_no) - 1
        channels = [val%4+val%4,val%4+val%4+1]
        if(msg.data != self.aggregate):
            for i in ['droid.stop_ir_broadcast()',
            'droid.stop_ir_evade()',
            'droid.stop_ir_follow()']:
                try: 
                    eval(i)
                except Exception:
                    pass
            
            #print("Aggregate")
            #print(channels)
            if (msg.data == True):
                droid.reset_aim()
                self.clear_sphero_velocity()         
                droid.start_ir_broadcast(channels[0],channels[1])
                """ for i in ([0,1],[2,3],[4,5],[6,7]):
                    if i != [channels[0],channels[1]]:
                        droid.start_ir_follow(i[0],i[1])
                        print("in following:",i) """
                list = [[0,1],[2,3],[4,5],[6,7]]
                i = list.index(channels)
                droid.start_ir_follow(list[(i+1)%4][0],list[(i+1)%4][1])
                self.aggregate  = True
            else:
                self.aggregate = False
                droid.reset_aim()
                droid.stop_ir_broadcast()
                droid.stop_ir_follow()
        else: 
            if  self.aggregate == True :
                list = [[0,1],[2,3],[4,5],[6,7]]
                i = list.index(channels)
                randy = random.randint(0,25)
                if   randy%5 == 0:
                    droid.start_ir_follow(list[(i+1)%4][0],list[(i+1)%4][1])
                elif randy%5 == 1: 
                    droid.start_ir_follow(list[(i+2)%4][0],list[(i+2)%4][1])
                elif randy%5 == 2: 
                    droid.start_ir_follow(list[(i+3)%4][0],list[(i+3)%4][1])

                
    def spread_callback(self, msg) -> None:
        val = int(self.robot_no) -1 
        channels = [val%4+val%4,val%4+val%4+1]
        if(msg.data != self.spread):
            for i in ['droid.stop_ir_broadcast()',
            'droid.stop_ir_evade()',
            'droid.stop_ir_follow()']:
                try: 
                    eval(i)
                except Exception:
                    pass
            
            #print("Spread")
            #print(channels)
            if (msg.data == True):
                droid.reset_aim()
                self.clear_sphero_velocity()         
                droid.start_ir_broadcast(channels[0],channels[1])
                """ for i in ([0,1],[2,3],[4,5],[6,7]):
                    if i != [channels[0],channels[1]]:
                        droid.start_ir_evade(i[0],i[1]) """
                list = [[0,1],[2,3],[4,5],[6,7]]
                i = list.index(channels)
                droid.start_ir_evade(list[(i+1)%4][0],list[(i+1)%4][1])
                self.spread  = True
            else:
                self.spread = False
                droid.reset_aim()
                droid.stop_ir_broadcast()
                droid.stop_ir_evade()
        else: 
            if  self.spread == True :
                list = [[0,1],[2,3],[4,5],[6,7]]
                i = list.index(channels)
                randy = random.randint(0,10)
                if randy%3 == 0:
                    droid.start_ir_evade(list[(i+1)%4][0],list[(i+1)%4][1])
                elif randy%3 == 1: 
                    droid.start_ir_evade(list[(i+2)%4][0],list[(i+2)%4][1])
                elif randy%3 == 2: 
                    droid.start_ir_evade(list[(i+3)%4][0],list[(i+3)%4][1])
                

    ### Create publishers

    def create_ros_publishers(self) -> None:

        # Ambient light
        self.pub_illuminance = rospy.Publisher('sphero/illuminance', 
                                               Illuminance, queue_size=1)
        
        # IMU
        self.pub_imu = rospy.Publisher('sphero/imu', 
                                               Imu, queue_size=1)
        
        # Encoders velocity
        self.pub_velocity = rospy.Publisher('sphero/velocity', 
                                               Twist, queue_size=1)
        
        # Encoders vertical acceleration
        self.pub_vertical_acc = rospy.Publisher('sphero/vertical_acc', 
                                               Float32, queue_size=1)
        
        # Encoders vertical acceleration
        self.pub_ir_signal = rospy.Publisher('sphero/ir_signal', 
                                               Bool, queue_size=20)
        
    ### Publishers

    def publish_illuminance(self):
        illumniance_msg = Illuminance()
        illumniance_msg.header.stamp = rospy.Time.now()
        illumniance_msg.header.frame_id = 'sphero'
        illumniance_msg.illuminance = droid.get_luminosity()['ambient_light']
        self.pub_illuminance.publish(illumniance_msg)

    
    # Angles must ve revised
    def publish_imu(self):
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = 'sphero'
        imu_msg.linear_acceleration.x = -droid.get_acceleration()['y'] * self.GRAVITY
        imu_msg.linear_acceleration.y = droid.get_acceleration()['x'] * self.GRAVITY
        imu_msg.linear_acceleration.z = droid.get_acceleration()['z'] * self.GRAVITY
        imu_msg.angular_velocity.x = -droid.get_gyroscope()['y'] * math.pi / 180
        imu_msg.angular_velocity.y = droid.get_gyroscope()['x'] * math.pi / 180
        imu_msg.angular_velocity.z = droid.get_gyroscope()['z'] * math.pi / 180
        quaternion = tf.transformations.quaternion_from_euler(droid.get_orientation()['pitch'],
                                                              droid.get_orientation()['roll'],
                                                              droid.get_orientation()['yaw'])
        imu_msg.orientation.x = quaternion[0]
        imu_msg.orientation.y = quaternion[1]
        imu_msg.orientation.z = quaternion[2]
        imu_msg.orientation.w = quaternion[3]
        self.pub_imu.publish(imu_msg)

    def publish_velocity(self):
        velocity_msg = Twist()
        velocity_msg.linear.x = droid.get_velocity()['y']
        velocity_msg.linear.y = droid.get_velocity()['x']
        self.pub_velocity.publish(velocity_msg)
    
    def publish_vertical_acc(self):
        vertical_acc_msg = Float32()
        vertical_acc_msg.data = droid.get_vertical_acceleration() * self.GRAVITY
        self.pub_vertical_acc.publish(vertical_acc_msg)

    def publish_ir_signal(self):
        ir_readings = droid.get_ir_readings()
        if 6 in ir_readings or 7 in ir_readings:
            ir_signal = Bool()
            ir_signal.data = True
            self.pub_ir_signal.publish(ir_signal)

    def publisher_callback(self, event=None):
        self.publish_illuminance()   
        self.publish_imu()
        self.publish_velocity()
        self.publish_vertical_acc()
        #self.publish_ir_signal()
    
    ### Controls 

    def set_sphero_velocity(self):

        update_velocity = False

        # This is not a velocity but a rotation of the specified angle, 
        # which is executed at every time step
        if self.target_heading != 0:
            droid.reset_aim()
            droid._SpheroEduAPI__heading = self.target_heading
            update_velocity = True

        if droid.get_speed != self.target_speed:
            droid._SpheroEduAPI__speed = self.target_speed
            update_velocity = True

        if update_velocity == True:
            droid._SpheroEduAPI__update_speed()
    
    def clear_sphero_velocity(self):
        self.target_speed = 0
        self.target_heading = 0
    
    def set_sphero_leds(self):

        if droid.get_front_led() != self.led_front_rgb:
            droid.set_front_led(self.led_front_rgb)

        if droid.get_back_led() != self.led_back_rgb:
            droid.set_back_led(self.led_back_rgb)

    def clear_sphero_leds(self):
        droid.set_front_led(0, 0, 0)
        droid.set_back_led(0, 0, 0)  

    def set_sphero_matrix(self):
        # droid.get_main_led() updates value with front LED and matrix
        # not suitable to detect changes in color command
        if self.led_matrix_rgb_new != self.led_matrix_rgb:
            self.led_matrix_rgb = self.led_matrix_rgb_new
            droid.set_main_led(self.led_matrix_rgb)

    def set_sphero_matrix_drawing(self):
        count_p = 0
        for piy in range(7,-1,-1):
            for pix in range(0,8):
                if self.led_matrix_hex[count_p] != '000000':
                    # Convert the p hex color string to an integer value
                    hex_int = int(self.led_matrix_hex[count_p], 16)
                    # Extract the red, green, and blue components from the integer value
                    r = (hex_int >> 16) & 0xFF
                    g = (hex_int >> 8) & 0xFF
                    b = hex_int & 0xFF
                    p_color = Color(r, g, b)
                    droid.set_matrix_pixel(pix, piy, p_color)
                count_p = count_p + 1

    def clear_sphero_matrix(self):
        droid.set_main_led(0, 0, 0)

    def control_loop_callback(self, event=None):
        self.set_sphero_leds()
        self.set_sphero_matrix()
        if (self.follow == False ):
            self.set_sphero_velocity()


    ### Utils

    def ms_to_speed(self, linear_x):
        bounded_speed = min(max(0, linear_x), self.target_max_speed) 
        sphero_speed = round((bounded_speed * 255)/self.MAX_BOLT_SPEED)  
        return sphero_speed 
        
    def rad_to_heading(self, angular_z):
        bounded_heading = min(self.target_max_heading, max(-self.target_max_heading, -angular_z))
        sphero_heading = (round(math.degrees(bounded_heading)) + 360) % 360
        return sphero_heading

### Main             
        
if __name__ == "__main__":

    sphero_id = rospy.get_param('sphero_id', 'SB-0000')
    toy = scanner.find_toy(toy_name=sphero_id)

    with SpheroEduAPI(toy) as droid:

        try:
            sphero_bolt = SpheroBolt()
            rospy.spin()

        except rospy.ROSInterruptException:
            rospy.loginfo("Keyboard interrupted")
            exit()
