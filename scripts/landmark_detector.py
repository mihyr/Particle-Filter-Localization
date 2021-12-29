#!/usr/bin/env python3
import rclpy
from rclpy import duration
from rclpy.node import Node
from builtin_interfaces.msg import Time, Duration
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker    
from geometry_msgs.msg import Point, Pose, Point, Quaternion, Vector3
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from std_msgs.msg import Float64
import numpy as np
from numpy.random import randint, random
from std_msgs.msg import ColorRGBA
from particle_filter.helper import polar2cartesian, scan2cartesian, circlefit, cartesian2polar, clustering, cartesian_clustering

class ScanSubscriber(Node):

    def __init__(self):
        super().__init__('landmark_detector')

        self.init_scan_state = False 
        qos = QoSProfile(depth=10)

        # Initialize publisher
        self.landmarks_pub = self.create_publisher(Point, 'landmarks_pose', qos)
        self.landmarks_marker_pub = self.create_publisher(Marker, 'landmarks_marker', 10)
        

        # Initialise subscribers

        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile=qos_profile_sensor_data)

        # self.subscription  # prevent unused variable warning
        self.get_logger().info("Turtlebot3 obstacle detection node has been initialised.")   
    
    def landmark_publisher(self,x,y,R):
        self.landmarks_pub.publish(Point(x=float(x), y=float(y), z=float(R)))
    
    def cone_marker_publisher(self,xc,yc,R,tag):
        self.marker = Marker()
        self.marker.header.frame_id = "base_scan"
        # self.marker.header.stamp = rclpy.get_clock.now()
        self.marker.id = tag
        self.marker.type = Marker.CYLINDER
        self.marker.action = Marker.ADD
        self.marker.scale = Vector3(x=0.15,y=0.15,z=0.4)
        self.marker.color = ColorRGBA(r=random(1)[0],g=random(1)[0],b=random(1)[0],a=1.0)
        self.marker.pose= Pose(position = Point(x=-float(xc),y=-float(yc),z=float(0)), orientation = Quaternion(x=float(0),y=float(0),z=float(0),w=float(1)))
        self.marker.lifetime = Duration(sec = 1,nanosec=0)
        self.landmarks_marker_pub.publish(self.marker)
        print("cone marker published")

    def scan_callback(self, msg):
        # get range data
        ranges = list(msg.ranges)

        # convert range data to cartesian coordinates
        cartesian_coordinates = scan2cartesian(ranges,increment= 0.01745329238474369)

        # cluster the cartesian coordinates
        clusters, outliers = cartesian_clustering(cartesian_coordinates)

        # for i in range(len(clusters)):
        #     print(len(clusters[i]), clusters[i])
        #     print('-----')

        # fit circles to the clusters
        tag = 0
        for cluster in clusters:
            try:
                xc, yc, R = circlefit(cluster)
                print(xc, yc, R)
                # publish landmark
                self.landmark_publisher(xc, yc,R)

                if 0.065 < R < 0.08: #ideal 0.070744334 m
                    self.cone_marker_publisher(-xc,-yc,R,tag)
                    tag = tag + 1

            except np.linalg.LinAlgError as err:
                if 'Singular matrix' in str(err):
                    continue
                else:
                    raise
        # self.init_scan_state = True


def main(args=None):
    rclpy.init(args=args)

    landmark_detector = ScanSubscriber()

    rclpy.spin(landmark_detector)

    landmark_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
