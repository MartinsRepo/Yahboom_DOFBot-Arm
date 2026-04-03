#!/usr/bin/env python3
# coding: utf-8
"""
ROS2 Kinematics Service Server for Dofbot Arm
Provides forward kinematics (FK) and inverse kinematics (IK) solutions

This module implements the 'dofbot_kinemarics' ROS2 service.
"""

import rclpy
from rclpy.node import Node
from math import pi, sin, cos, atan2, hypot
from dofbot_info.srv import Kinemarics


class KinematricsServer(Node):
    """ROS2 Service Server for arm kinematics calculations."""
    
    def __init__(self):
        super().__init__('kinemarics_server')
        
        # Create the service
        self.srv = self.create_service(
            Kinemarics,
            'dofbot_kinemarics',
            self.handle_kinemarics_request
        )
        
        # Constants for angle conversion
        self.RA2DE = 180.0 / pi  # radians to degrees
        self.DE2RA = pi / 180.0  # degrees to radians
        
        # TODO: Load actual URDF and kinematics model
        # self.urdf_file = "/home/osboxes/ros2_jazzy_ws/src/dofbot_info/urdf/dofbot.urdf"
        # self.setup_kinematics_model()
        
        self.get_logger().info('Kinemarics server ready')
    
    def handle_kinemarics_request(self, request, response):
        """
        Handle forward and inverse kinematics requests.
        
        Args:
            request: Kinemarics service request containing:
                - kin_name: "fk" for forward kinematics or "ik" for inverse kinematics
                - For FK: cur_joint1 to cur_joint5 (joint angles in degrees)
                - For IK: tar_x, tar_y, tar_z (target position in meters)
            
            response: Kinemarics service response to populate:
                - For FK: x, y, z, roll, pitch, yaw (end-effector pose)
                - For IK: joint1 to joint5 (joint angles in degrees)
        
        Returns:
            True if successful, False otherwise
        """
        try:
            if request.kin_name == "fk":
                self.get_logger().info('Forward kinematics request received')
                return self.forward_kinematics(request, response)
            
            elif request.kin_name == "ik":
                self.get_logger().info('Inverse kinematics request received')
                return self.inverse_kinematics(request, response)
            
            else:
                self.get_logger().warn(f'Unknown kinematics request: {request.kin_name}')
                return False
        
        except Exception as e:
            self.get_logger().error(f'Error handling kinematics request: {e}')
            return False
    
    def forward_kinematics(self, request, response):
        """
        Calculate forward kinematics (joint angles → end-effector pose).
        
        TODO: Integrate with actual KDL-based forward kinematics computation
        using the compiled libdofbot_kinemarics.so library or Python KDL bindings.
        
        Current implementation is a placeholder that returns zero values.
        """
        # Extract joint angles (convert from degrees to radians internally if needed)
        joints = [
            request.cur_joint1,
            request.cur_joint2,
            request.cur_joint3,
            request.cur_joint4,
            request.cur_joint5
        ]
        
        self.get_logger().debug(f'FK Input joints (degrees): {joints}')
        
        # TODO: Call actual kinematics computation
        # For now, return placeholder values
        response.x = 0.0
        response.y = 0.0
        response.z = 0.0
        response.roll = 0.0
        response.pitch = 0.0
        response.yaw = 0.0
        
        self.get_logger().info(f'FK Output: x={response.x:.4f}, y={response.y:.4f}, z={response.z:.4f}')
        return True
    
    def inverse_kinematics(self, request, response):
        """
        Calculate inverse kinematics (end-effector pose → joint angles).
        
        Implements the kinematics logic from the original C++ implementation:
        - Takes target position (tar_x, tar_y, tar_z)
        - Computes gripper offset and workspace constraints
        - Solves for joint angles to reach target
        
        TODO: Integrate with actual KDL-based inverse kinematics computation
        using the compiled libdofbot_kinemarics.so library or Python KDL bindings.
        
        Current implementation is a placeholder that returns zero values.
        """
        self.get_logger().debug(
            f'IK Input: pos=({request.tar_x:.4f}, {request.tar_y:.4f}, {request.tar_z:.4f})'
        )
        
        # Gripper parameters from original implementation
        tool_param = 0.12  # Gripper length in meters
        
        # Compute target pose angles
        roll = 2.5 * request.tar_y * 100 - 207.5
        pitch = 0.0
        yaw = 0.0
        
        # Compute offset angle
        init_angle = atan2(float(request.tar_x), float(request.tar_y))
        
        # Compute gripper projection on hypotenuse
        dist = tool_param * sin((180 + roll) * self.DE2RA)
        
        # Compute hypotenuse length
        distance = hypot(request.tar_x, request.tar_y) - dist
        
        # Compute end-effector position (gripper excluded)
        x = distance * sin(init_angle)
        y = distance * cos(init_angle)
        z = tool_param * cos((180 + roll) * self.DE2RA)
        
        # Special case: front-to-back following (high Z)
        if request.tar_z >= 0.2:
            x = request.tar_x
            y = request.tar_y
            z = request.tar_z
            roll = -90.0
        
        self.get_logger().debug(
            f'IK Computed pose: xyz=({x:.4f}, {y:.4f}, {z:.4f}), rpy=({roll:.2f}, {pitch:.2f}, {yaw:.2f})'
        )
        
        # TODO: Call actual KDL inverse kinematics solver
        # For now, return placeholder joint angles (all 90 degrees = neutral position)
        response.joint1 = 90.0
        response.joint2 = 90.0
        response.joint3 = 90.0
        response.joint4 = 90.0
        response.joint5 = 90.0
        
        self.get_logger().info(
            f'IK Output joints (degrees): {response.joint1:.2f}, {response.joint2:.2f}, '
            f'{response.joint3:.2f}, {response.joint4:.2f}, {response.joint5:.2f}'
        )
        return True


def main(args=None):
    rclpy.init(args=args)
    
    node = KinematricsServer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
