import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np
import sys, select, termios, tty

msg = """
Control Your Robot!
Press and hold the keys to move around.
Press space key to turn on/off tracking.
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

t/y : turn counterclockwise/clockwise
k : force stop
space key : turn on/off tracking
anything else : stop smoothly

CTRL-C to quit
"""

moveBindings={'i':0, 'u':1, 'j':2, 'm':3, ',':4, '.':5, 'l':6, 'o':7, 'k':None}

keySettings=termios.tcgetattr(sys.stdin)
def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist: key = sys.stdin.read(1)
    else: key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, keySettings)
    return key

class JoySafetyNode(Node):
    def __init__(self):
        super().__init__('joy_safety_node')
        self.get_logger().info('Joy Safety Node Started')
        
        # Declare the parameters for the color detection
        self.declare_parameter('max_linear_speed', 0.15)
        self.declare_parameter('max_angular_speed', 0.75)
        self.declare_parameter('joystick_speed', 0.25)
        self.declare_parameter('joystick_ang_speed', 0.6)
        # variables
        self.tracking_enabled = False
        self.tracking_cmd_vel = None
        self.joystick_state = None
        self.joystick_state_ang = 1
        
        # Create publisher for the control command
        self.pub_control_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        # Create a subscriber to the control command
        self.sub_track_cmd = self.create_subscription(Twist, '/path_cmd_vel', self.tracking_cmd_callback, 10)
    
        # Create timer
        self.timer = self.create_timer(0.005, self.timer_update)
    
    def tracking_cmd_callback(self, msg):
        # self.get_logger().info('Received Tracking Control Command')
        
        # Get the control command
        cmd_linear_speed = [msg.linear.x, msg.linear.y, msg.linear.z]
        cmd_angular_speed = msg.angular.z
        
        # Get the maximum speed
        max_linear_speed = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        max_angular_speed = self.get_parameter('max_angular_speed').get_parameter_value().double_value
        
        # Limit the speed
        cmd_linear_speed = np.clip(cmd_linear_speed, -max_linear_speed, max_linear_speed)
        cmd_angular_speed = np.clip(cmd_angular_speed, -max_angular_speed, max_angular_speed)
        
        # Publish the control command
        cmd_vel = Twist()
        cmd_vel.linear.x = cmd_linear_speed[0]
        cmd_vel.linear.y = cmd_linear_speed[1]
        cmd_vel.linear.z = cmd_linear_speed[2]
        cmd_vel.angular.z = cmd_angular_speed
        self.tracking_cmd_vel = cmd_vel
    
    def timer_update(self):
        
        # Publish the control command
        cmd_vel = Twist()
        car_speed = self.get_parameter('joystick_speed').get_parameter_value().double_value
        car_ang_speed = self.get_parameter('joystick_ang_speed').get_parameter_value().double_value
        
        if self.joystick_state == 8:
            self.tracking_enabled = not self.tracking_enabled
            if self.tracking_enabled:
                print("Switch ON tracking.")
            else:
                print("Switch OFF tracking.")
        elif self.joystick_state is None:
            pass
        else:
            twist_angle = self.joystick_state * np.pi / 4
            cmd_vel.linear.x = car_speed * np.cos(twist_angle)
            cmd_vel.linear.y = car_speed * np.sin(twist_angle)
        if self.joystick_state_ang is None:
            pass
        else:
            cmd_vel.angular.z = car_ang_speed * (self.joystick_state_ang-1)
        
        if self.tracking_enabled and self.tracking_cmd_vel is not None:
            cmd_vel = self.tracking_cmd_vel
       
        self.pub_control_cmd.publish(cmd_vel)
        self.joystick_state = None
        self.joystick_state_ang = None

def main(args=None):
    rclpy.init(args=args)
    joy_safety_node = JoySafetyNode()
    
    print(msg)
    # read from keyboard
    while rclpy.ok():
        rclpy.spin_once(joy_safety_node)
        key = getKey()
        if key == ' ':
            joy_safety_node.joystick_state = 8
        elif key == 't':
            joy_safety_node.joystick_state_ang = 2
        elif key == 'y':
            joy_safety_node.joystick_state_ang = 0
        elif key in moveBindings.keys():
            joy_safety_node.joystick_state = moveBindings[key]
        elif key == '\x03':
            break
    
    joy_safety_node.destroy_node()
    rclpy.shutdown()