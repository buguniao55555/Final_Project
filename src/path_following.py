import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from tf2_ros import TransformException, Buffer, TransformListener
import numpy as np
import math
import helper
from PIL import Image
from Rosmaster_Lib import Rosmaster
import time 
intopix=1.675
inchtom=0.0254
## Functions for quaternion and rotation matrix conversion
## The code is adapted from the general_robotics_toolbox package
## Code reference: https://github.com/rpiRobotics/rpi_general_robotics_toolbox_py
def hat(k):
    """
    Returns a 3 x 3 cross product matrix for a 3 x 1 vector

             [  0 -k3  k2]
     khat =  [ k3   0 -k1]
             [-k2  k1   0]

    :type    k: numpy.array
    :param   k: 3 x 1 vector
    :rtype:  numpy.array
    :return: the 3 x 3 cross product matrix
    """

    khat=np.zeros((3,3))
    khat[0,1]=-k[2]
    khat[0,2]=k[1]
    khat[1,0]=k[2]
    khat[1,2]=-k[0]
    khat[2,0]=-k[1]
    khat[2,1]=k[0]
    return khat

def q2R(q):
    """
    Converts a quaternion into a 3 x 3 rotation matrix according to the
    Euler-Rodrigues formula.
    
    :type    q: numpy.array
    :param   q: 4 x 1 vector representation of a quaternion q = [q0;qv]
    :rtype:  numpy.array
    :return: the 3x3 rotation matrix    
    """
    
    I = np.identity(3)
    qhat = hat(q[1:4])
    qhat2 = qhat.dot(qhat)
    return I + 2*q[0]*qhat + 2*qhat2
######################

def euler_from_quaternion(q):
    w=q[0]
    x=q[1]
    y=q[2]
    z=q[3]
    # euler from quaternion
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(2 * (w * y - z * x))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    return [roll,pitch,yaw]

class PathNode(Node):
    def __init__(self, start, end):
        super().__init__('path_node')
        self.get_logger().info('Path Node Started')
        
        self.bot=Rosmaster()
        self.bot.create_receive_threading()
        # ROS parameters
        self.declare_parameter('world_frame_id', 'odom')

        # Create a transform listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Create publisher for the control command
        self.pub_control_cmd = self.create_publisher(Twist, '/path_cmd_vel', 10)
        # Create a subscriber to the detected object pose
        grid = helper.read_pgm("maps/new_map.pgm")

        grid_expanded_obstacles = helper.take_local(grid, 4, np.min)

        self.result_path = helper.find_route_astar(grid_expanded_obstacles, start, end)
        filtered_path = helper.filter_path(self.result_path)
        self.curve = helper.bezier_curve(filtered_path)    
    
        self.dt = .2
        self.t = self.dt
        # for PID control
        self.kp = 1
        # self.ki = 0.01
        self.kd = 15
        self.prev_err = 0.0
        self.sum_err = 0.0
        self.i=0

        # Create timer, running at 100Hz
        self.timer = self.create_timer(self.dt, self.timer_update)
    def get_current_robot_pose(self):
        current_robot_pose=0
        """
        odom_id = self.get_parameter('world_frame_id').get_parameter_value().string_value
        # Get the current robot pose
        try:
            # from base_footprint to odom
            transform = self.tf_buffer.lookup_transform('base_footprint', odom_id, rclpy.time.Time())
            robot_world_x = transform.transform.translation.x
            robot_world_y = transform.transform.translation.y
            robot_world_z = transform.transform.translation.z
            current_robot_pose=[robot_world_x,robot_world_y,robot_world_z] 
        except TransformException as e:
            self.get_logger().error('Transform error: ' + str(e))
            return
            """
        return current_robot_pose
    
    def timer_update(self):
        current_robot_pose=self.get_current_robot_pose()
        cmd_vel = self.controller(current_robot_pose)
        
        # publish the control command
        self.pub_control_cmd.publish(cmd_vel)

        #################################################
    def controller(self, current_robot_pose):
        # Instructions: You can implement your own control algorithm here
        # feel free to modify the code structure, add more parameters, more input variables for the function, etc.
        if self.t > 30.0:
            self.bot.set_car_motion(0, 0, 0)
            return
        cmd_vel = Twist()
        if self.i>=len(self.result_path)-1:
            self.bot.set_car_motion(0, 0, 0)
            return
        #y=float((self.curve.evaluate((self.t+self.dt)/30)[1]-self.curve.evaluate(self.t/30)[1])*intopix*inchtom/self.dt)
        #x=float((self.curve.evaluate((self.t+self.dt)/30)[0]-self.curve.evaluate(self.t/30)[0])*intopix*inchtom/self.dt)
        y=float((self.result_path[self.i+1][1]-self.result_path[self.i][1])*intopix*inchtom/self.dt)
        x=float((self.result_path[self.i+1][0]-self.result_path[self.i][0])*intopix*inchtom/self.dt)
        x *= 1.05
        y *= 1.05
        self.get_logger().info(str(self.t) + " x: " + str(x) + " y: " + str(y)+"Next"+str(self.result_path[self.i+1]))
        self.bot.set_car_motion(x,y,0)
        
        # pos_x = self.get_position().transform.translation.x
        # pos_y = self.get_position().transform.translation.y

        self.i+=1
        self.t+=self.dt
        return cmd_vel
def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)
    # Create the node
    start_end =[(40,60), (52,120)]
    start_end_2 = [(40, 60), (75, 120)]
    path_node = PathNode(start_end_2[0], start_end_2[1])
    rclpy.spin(path_node)
    # Destroy the node explicitly
    path_node.destroy_node()
    # Shutdown the ROS client library for Python
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    