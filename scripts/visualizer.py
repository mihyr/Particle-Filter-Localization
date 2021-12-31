#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from std_msgs.msg import String
from particle_filter.msg import LdPose
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from std_msgs.msg import ColorRGBA
from numpy.random import randint, random
from builtin_interfaces.msg import Time, Duration

class ParticleFilterVisualizer(Node):

    def __init__(self):
        super().__init__('visualizer')

        qos = QoSProfile(depth=10)

        # Initialize publisher
        self.landmarks_marker_pub = self.create_publisher(Marker, 'landmarks_marker', qos)
        self.clusters_marker_pub = self.create_publisher(MarkerArray, 'clusters_marker', qos)
        self.outliers_marker_pub = self.create_publisher(MarkerArray, 'outliers_marker', qos)
        
        # Initialise subscribers
        self.subscription = self.create_subscription(
            LdPose,
            'landmarks_pose',
            self.ld_pose_callback,
            qos_profile=qos_profile_sensor_data)

        # self.subscription  # prevent unused variable warning
        self.get_logger().info("Particle Filter Visualizer node has been initialised.")   

    def cone_marker_publisher(self,xc,yc,R,tag):
        self.marker = Marker()
        self.marker.header.frame_id = "base_scan"
        # self.marker.header.stamp = rclpy.get_clock.now()
        self.marker.id = tag
        self.marker.type = Marker.CYLINDER
        self.marker.action = Marker.ADD
        self.marker.scale = Vector3(x=0.15,y=0.15,z=0.4)
        self.marker.color = ColorRGBA(r=random(1)[0],g=random(1)[0],b=random(1)[0],a=1.0)
        self.marker.pose= Pose(position = Point(x=float(xc),y=float(yc),z=float(0)), orientation = Quaternion(x=float(0),y=float(0),z=float(0),w=float(1)))
        self.marker.lifetime = Duration(sec = 2,nanosec=0)
        self.landmarks_marker_pub.publish(self.marker)
        # print(f'cone marker published for tag {tag}')

    def cluster_marker_publisher(self,clusters,landmark_ids):
        self.marker_array = MarkerArray()
        markerid = 0
        # print(f'clusters: {clusters}')
        # for i in range(len(clusters)):
        #     print(f'cluster {clusters[i].x}')
        for i in range(len(clusters)):
            self.marker = Marker()
            self.marker.header.frame_id = "base_scan"
            self.marker.header.stamp = self.get_clock().now().to_msg()
            # self.marker.id = int(clusters[i].z)
            self.marker.id = markerid
            self.marker.type = Marker.SPHERE
            self.marker.action = Marker.ADD
            self.marker.scale = Vector3(x=0.02,y=0.02,z=0.02)
            # self.marker.color = ColorRGBA(r=random(1)[0],g=random(1)[0],b=random(1)[0],a=1.0)
            self.marker.color = ColorRGBA(r=1.0,g=0.0,b=0.0,a=1.0)
            self.marker.pose.position = clusters[i]
            
            self.marker.pose.position.z = 0.05
            # self.marker.pose= Pose(position = clusters[i], orientation = Quaternion(x=float(0),y=float(0),z=float(0),w=float(1)))
            self.marker.lifetime = Duration(sec = 1,nanosec=0)
            self.marker_array.markers.append(self.marker)
            markerid += 1
            # print(f'cluster marker published for tag {landmark_ids[i]}')
        # print(self.marker_array)
        # print('=======')
        self.clusters_marker_pub.publish(self.marker_array)

    def outlier_marker_publisher(self,outliers):
        self.marker_array = MarkerArray()
        markerid = 0

        for i in range(len(outliers)):
            self.marker = Marker()
            self.marker.header.frame_id = "base_scan"
            self.marker.id = markerid
            self.marker.type = Marker.SPHERE
            self.marker.action = Marker.ADD
            self.marker.scale = Vector3(x=0.02,y=0.02,z=0.02)
            self.marker.color = ColorRGBA(r=random(1)[0],g=random(1)[0],b=random(1)[0],a=1.0)
            self.marker.color = ColorRGBA(r=0.0,g=1.0,b=0.0,a=1.0)
            # self.marker.pose= Pose(position = Point(x=float(0),y=float(0),z=float(0)), orientation = Quaternion(x=float(0),y=float(0),z=float(0),w=float(1)))
            self.marker.pose.position = outliers[i]
            self.marker.pose.position.z = 0.05
            self.marker.lifetime = Duration(sec = 1,nanosec=0)
            self.marker_array.markers.append(self.marker)
            markerid += 1
            # print(f'outlier marker published for tag {outliers[i]}')
        
        self.outliers_marker_pub.publish(self.marker_array)

    def ld_pose_callback(self,msg):
        num_of_landmarks = msg.num_of_landmarks

        for i in range(num_of_landmarks):
            self.cone_marker_publisher(msg.landmark_coordinates[i].x,msg.landmark_coordinates[i].y,msg.landmark_coordinates[i].z,msg.landmark_ids[i])
            
        self.cluster_marker_publisher(msg.clusters,msg.landmark_ids)
        self.outlier_marker_publisher(msg.outliers)


def main(args=None):
    rclpy.init(args=args)

    particlefiltervisualizer = ParticleFilterVisualizer()

    rclpy.spin(particlefiltervisualizer)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    particlefiltervisualizer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()