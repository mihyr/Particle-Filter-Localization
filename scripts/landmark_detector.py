#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
 
from geometry_msgs.msg import Point, Pose, Point, Quaternion, Vector3
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from std_msgs.msg import Float64
import numpy as np
from particle_filter.landmark_helper import polar2cartesian, scan2cartesian, circlefit, cartesian2polar, clustering, cartesian_clustering
from particle_filter.msg import LdPose

class LandmarkPublisher(Node):

    def __init__(self):
        super().__init__('landmark_detector')

        self.init_scan_state = False 
        qos = QoSProfile(depth=10)

        # Initialize publisher
        self.landmarks_pub = self.create_publisher(LdPose, 'landmarks_pose', qos)
        
        # Initialise subscribers
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile=qos_profile_sensor_data)

        # self.subscription  # prevent unused variable warning
        self.get_logger().info("Particle Filter Obstacle detection node has been initialised.")   

     
    def landmark_publisher(self,total_landmarks,landmark_ids,landmark_coordinates,clusters,outliers):
        self.ldpose = LdPose()
        # self.ldpose.header = 
        self.ldpose.num_of_landmarks = total_landmarks
        self.ldpose.landmark_ids = landmark_ids
        self.ldpose.landmark_coordinates = landmark_coordinates
        # self.ldpose.clusters = [Point(x=float(1),y=float(2),z =float(3)),Point(x=float(4),y=float(5),z =float(6))]
        self.ldpose.clusters = clusters
        self.ldpose.outliers = outliers
        self.landmarks_pub.publish(self.ldpose)

    def scan_callback(self, msg):
        # get range data
        # ranges = [float(item) for item in msg.ranges]
        ranges = msg.ranges
        
        # convert range data to cartesian coordinates
        cartesian_coordinates = scan2cartesian(ranges,increment= 0.01745329238474369)

        # cluster the cartesian coordinates
        clusters, outliers = cartesian_clustering(cartesian_coordinates)

        # for i in range(len(clusters)):
        #     print(len(clusters[i]), clusters[i])
        #     print('-----')

        
        tag = 0
        total_landmarks = 0
        landmark_ids = []
        landmark_coordinates = []
        clusterid = []
        outlierid = []

        # fit circles to the clusters
        for cluster in clusters:
            try:
                xc, yc, R = circlefit(cluster)
                # print(type(xc), type(yc), type(R))
                
                if 0.065 < R < 0.08: #ideal 0.070744334 m

                    # self.cone_marker_publisher(-xc,-yc,R,tag)
                    total_landmarks = total_landmarks + 1
                    tag = tag + 1
                    landmark_ids.append(tag)
                    landmark_coordinates.append(Point(x=xc,y=yc,z =R))
                    
                    for xcor,ycor in cluster:
                        clusterid.append(Point(x=float(xcor),y=float(ycor),z =float(tag)))

            except np.linalg.LinAlgError as err:
                if 'Singular matrix' in str(err):
                    continue
                else:
                    raise
        
        for outlier in outliers:
            for xcor,ycor in outlier:
                outlierid.append(Point(x=float(xcor),y=float(ycor),z =float(-1)))
        
        self.landmark_publisher(total_landmarks,landmark_ids,landmark_coordinates,clusterid,outlierid)
        # self.init_scan_state = True


def main(args=None):
    rclpy.init(args=args)

    landmark_detector = LandmarkPublisher()

    rclpy.spin(landmark_detector)

    landmark_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
