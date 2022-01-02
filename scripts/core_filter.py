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
from particle_filter.kinematics_helper import motion_model, landmark_estimate
from particle_filter.msg import LdPose, Particles
from tf_transformations import quaternion_from_euler


class CoreFilter(Node):

    def __init__(self):
        super().__init__('core_filter',
                        allow_undeclared_parameters=True,
                        automatically_declare_parameters_from_overrides=False)

        qos = QoSProfile(depth=10)

        # Initialize publisher
        self.publisher_ = self.create_publisher(Particles, 'particles', qos)

        timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Initialise subscribers
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            qos_profile=qos_profile_sensor_data)
        
        self.subscription = self.create_subscription(
            LdPose,
            'landmarks_pose',
            self.landmarks_callback,
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
        self.x_range = self.get_parameter('map_size.x_range').value
        self.y_range = self.get_parameter('map_size.y_range').value
        self.max_particles = self.get_parameter('particles.max_particles').value
        # self.get_logger().info('ld1: %s' % self.ld1)
        
        # self.subscription  # prevent unused variable warning
        self.get_logger().info("Particle Filter Core filter node has been initialised.")   
    
    def generate_particles(self,num_particles):
        """
        generate random particles [x,y,theta,weight] of size num_particles, weight = 1/num_particles
        """
        particles = []
        # Set equal weights
        weight = 1/num_particles

        for i in range(num_particles):
            x = np.random.uniform(self.x_range[0], self.x_range[1])
            y = np.random.uniform(self.y_range[0], self.y_range[1])
            theta = np.random.uniform(0, 2*np.pi)
            particles.append([x,y,theta,weight])
    
        
        return particles

    def particle_publisher(self, particles):
        """
        Publish particles
        """
        self.particle_array = Particles()
        self.particle_array.header.frame_id = 'map'
        self.particle_array.header.stamp = self.get_clock().now().to_msg()
        
        for i in range(len(particles)):
            self.pose = Pose()
            self.pose.position = Point(x=particles[i][0], y=particles[i][1], z=float(0))
            q = quaternion_from_euler(0, 0, particles[i][2], axes='sxyz')
            self.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            self.particle_array.particles.append(self.pose)
            self.particle_array.weights.append(particles[i][3])
        
        self.publisher_.publish(self.particle_array)

    def landmarks_callback(self, msg):
        particles = self.generate_particles(self.max_particles)
        self.particle_publisher(particles)

        num_of_landmarks = msg.num_of_landmarks

        for i in range(num_of_landmarks):
            # landmark estimation for each particle
            for particle in particles:
                ld_x,ld_y = landmark_estimate(particle, [msg.landmark_coordinates[i].x, msg.landmark_coordinates[i].y])
                # self.get_logger().info(f'{ld_x}, {ld_y}')


        # particles = self.generate_particles(self.max_particles)
        pass

    def odom_callback(self, msg):
        pass
        # particles = self.generate_particles(self.max_particles)
        # landmark = [0,0]
        # # landmark estimation for each particle
        # for particle in particles:
        #     ld_x,ld_y = landmark_estimate(particle, landmark)
            # self.get_logger().info(f'{ld_x}, {ld_y}')



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