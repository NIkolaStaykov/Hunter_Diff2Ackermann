import math
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rcl_interfaces.srv import GetParameters

from urdf_parser_py.urdf import URDF
import xml.etree.ElementTree as ET

class RobotParams:
    def __init__(self, max_joint_steering_angle: float, max_velocity: float, base_length: float, track_width: float):
        self.max_joint_steering_angle = max_joint_steering_angle
        self.base_length = base_length
        self.track_width = track_width
        self.max_velocity = max_velocity
        # Calculate the maximum steering angle based on the maximum individual tire angle
        self.max_steering_angle = np.arctan(1/(1/np.tan(max_joint_steering_angle) + track_width/(2*base_length)))

class ControlTranformer(Node):
    def __init__(self):
        super().__init__('control_subscriber')

        self.param_client = self.create_client(GetParameters, '/robot_state_publisher/get_parameters')
        self.robot_params = self.get_robot_params()
        self.subscription = self.create_subscription(
            Twist,
            'diff_drive/vel_cmd',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        self.publisher_ackerman = self.create_publisher(
            Twist,
            'ackermann/steering_cmd',
            10
        )

        self.publisher_control = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        self.param_client = self.create_client(GetParameters, '/robot_state_publisher/get_parameters')


    def get_robot_params(self):
        # Get the robot_description parameter from the ROS 2 parameter server
        req = GetParameters.Request()
        req.names = ['robot_description']

        future = self.param_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.done():
            response = future.result()
            if response is not None:
                param_value = response.values[0].string_value
            else:
                print("Failed to get response")
        else:
            self.get_logger().error('Failed to call service get_parameters')
            return None

        root = ET.fromstring(param_value)
        gazebo_plugin = root.find(".//plugin[@name='gazebo_ros_ackermann_drive']")
        max_speed = float(gazebo_plugin.find("max_speed").text)

        # Parse the URDF
        robot = URDF.from_xml_string(param_value)

        # Extract parameters from the URDF
        max_joint_steering_angle = float(robot.joint_map['fr_steer_left_joint'].limit.upper)
        track_width = abs(float(robot.joint_map['fr_steer_left_joint'].origin.xyz[1])) * 2
        
        base_to_front = abs(float(robot.joint_map['fr_steer_left_joint'].origin.xyz[0]))
        base_to_back = abs(float(robot.joint_map['re_left_joint'].origin.xyz[0]))
        base_length = base_to_front + base_to_back

        return RobotParams(max_joint_steering_angle, max_speed, base_length, track_width)
    
    def get_steering_angle(self, lin_vel:float, rot_vel:float) -> float:
        """
        Calculate inner and outer wheel steering angles for Ackermann steering.

        Parameters:
        l (float): Wheelbase (distance between front and rear axles).
        w (float): Track width (distance between left and right wheels).
        phi (float): Steering angle of the car (in radians).

        Returns:
        tuple: Inner wheel angle (phi_i), Outer wheel angle (phi_o).
        """
        l = self.robot_params.base_length
        phi = math.atan2(l * rot_vel, np.linalg.norm(lin_vel)) if lin_vel != 0 else self.robot_params.max_steering_angle * np.sign(rot_vel)
        if lin_vel == 0:
            self.get_logger().warn('Linear velocity is zero, using max steering angle')

        return phi

    def listener_callback(self, msg):
        self.get_logger().info(f'Received Differential Controls: '
                        f'Linear: x={msg.linear.x}| '
                        f'Angular: z={msg.angular.z}')
        lin_vel = msg.linear.x
        rot_vel = msg.angular.z

        # Conver base velocities to steering angles
        phi = self.get_steering_angle(lin_vel, rot_vel)

        # Ensure the steering angle is within the max_steering_angle
        if abs(phi) > self.robot_params.max_steering_angle:
            self.get_logger().warn(f'Steering angle {phi} exceeds max steering angle {self.robot_params.max_steering_angle}, capping it.')
            phi = np.sign(phi) * self.robot_params.max_steering_angle

        # Ensure the linear velocity is within the max_velocity
        if abs(lin_vel) > self.robot_params.max_velocity:
            self.get_logger().warn(f'Linear velocity {lin_vel} exceeds max velocity {self.robot_params.max_velocity}, capping it.')
            lin_vel = np.sign(lin_vel) * self.robot_params.max_velocity

        # Create a new Twist message for the Ackermann steering command
        ackermann_msg = Twist()
        ackermann_msg.linear.x = lin_vel
        ackermann_msg.angular.z = phi
        self.publisher_control.publish(ackermann_msg)
        self.get_logger().info(f'Published Ackermann steering command: '
                        f'Linear: x={ackermann_msg.linear.x}, Angular: z={ackermann_msg.angular.z}')
        
def main():
    rclpy.init()
    node = ControlTranformer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()