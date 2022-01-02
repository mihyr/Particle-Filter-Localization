#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
 
from geometry_msgs.msg import Point, Pose, Point, Quaternion, Vector3
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from std_msgs.msg import Float64
from rcl_interfaces.srv import GetParameters
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
import numpy as np
from particle_filter.kinematics_helper import motion_model, measurement_model
from particle_filter.msg import LdPose

class CoreFilter(Node):

    def __init__(self):
        super().__init__('core_filter',
                        allow_undeclared_parameters=True,
                        automatically_declare_parameters_from_overrides=False)

        qos = QoSProfile(depth=10)

        # Initialize publisher
        self.publisher_ = self.create_publisher(String, 'particles', qos)

        timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Initialise subscribers
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            qos_profile=qos_profile_sensor_data)

        # Declare parameters, set value in config/params.yaml
        self.declare_parameters(
            namespace='',
            parameters=[
                ('ld_ground_truth.ld1', Parameter.Type.DOUBLE_ARRAY),
                ('ld_ground_truth.ld2', Parameter.Type.DOUBLE_ARRAY),
                ('ld_ground_truth.ld3', Parameter.Type.DOUBLE_ARRAY),
                ('ld_ground_truth.ld4', Parameter.Type.DOUBLE_ARRAY),
                ('map_size.x_range', Parameter.Type.DOUBLE_ARRAY),
                ('map_size.y_range', Parameter.Type.DOUBLE_ARRAY),
                ('particles.max_particles', Parameter.Type.INTEGER)

            ])
        
        self.ld1 = self.get_parameter('ld_ground_truth.ld1').value
        self.ld2 = self.get_parameter('ld_ground_truth.ld2').value
        self.ld3 = self.get_parameter('ld_ground_truth.ld3').value
        self.ld4 = self.get_parameter('ld_ground_truth.ld4').value
        self.max_particles = self.get_parameter('particles.max_particles').value
        # self.get_logger().info('ld1: %s' % self.ld1)
        
        # self.subscription  # prevent unused variable warning
        self.get_logger().info("Particle Filter Core filter node has been initialised.")   
    
    def odom_callback(self, msg):
        pass

def main(args=None):
    rclpy.init(args=args)

    core_filter = CoreFilter()

    rclpy.spin(core_filter)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    core_filter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()