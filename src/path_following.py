import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from tf2_ros import TransformException, Buffer, TransformListener
import numpy as np
import math
import helper
from PIL import Image
intopix=1.82
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
    def __init__(self):
        super().__init__('path_node')
        self.get_logger().info('Path Node Started')
        
        
        # ROS parameters
        self.declare_parameter('world_frame_id', 'odom')

        # Create a transform listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Create publisher for the control command
        self.pub_control_cmd = self.create_publisher(Twist, '/track_cmd_vel', 10)
        # Create a subscriber to the detected object pose
        grid = helper.read_pgm()

        grid_expanded_obstacles = helper.take_local(grid, 3, np.min)


        start = (40, 33)
        end = (76, 130)

        result_path = helper.find_route_astar(grid_expanded_obstacles, start, end)
        filtered_path = helper.filter_path(result_path)
        self.curve = helper.bezier_curve(filtered_path)    
    
        self.t=0.01
        # for PID control
        self.kp = 1
        # self.ki = 0.01
        self.kd = 15
        self.prev_err = 0.0
        self.sum_err = 0.0

        # Create timer, running at 100Hz
        self.timer = self.create_timer(0.01, self.timer_update)
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
        
        ########### Write your code here ###########
        
        # TODO: Update the control velocity command
        cmd_vel = Twist()
        y=float(self.curve.evaluate(30.0/self.t)[1]*intopix*inchtom/0.01)
        x=float(self.curve.evaluate(30.0/self.t)[0]*intopix*inchtom/0.01)
        self.get_logger().info(str(self.t))
        self.get_logger().info(str("x"))
        self.get_logger().info(str(x))
        self.get_logger().info(str("y"))
        self.get_logger().info(str(y))
        cmd_vel.linear.y = y
        cmd_vel.linear.x = x
        self.t+=0.01
        return cmd_vel
def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)
    # Create the node
    path_node = PathNode()
    rclpy.spin(path_node)
    # Destroy the node explicitly
    path_node.destroy_node()
    # Shutdown the ROS client library for Python
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    