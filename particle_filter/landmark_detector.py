import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time, Duration
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker    
from geometry_msgs.msg import Point, Pose, Point, Quaternion, Vector3
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from std_msgs.msg import Float64
import math
import numpy as np
from example_interfaces.msg import Float64MultiArray
from numpy.random import randint, random
from rclpy.duration import Duration

class ScanSubscriber(Node):

    def __init__(self):
        super().__init__('landmark_detector')

        self.init_scan_state = False 
        qos = QoSProfile(depth=10)

        # Initialize publisher
        self.landmarks_pub = self.create_publisher(Point, 'landmarks_pose', qos)
        self.landmarks_marker_pub = self.create_publisher(Marker, 'landmarks_marker', 10)
        self.marker = Marker()
        

        # Initialise subscribers

        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile=qos_profile_sensor_data)

        # self.subscription  # prevent unused variable warning
        self.get_logger().info("Turtlebot3 obstacle detection node has been initialised.")

    # Helper functions
    def polar2cartesian(self,r, theta):
        x = r * math.cos(theta)
        y = r * math.sin(theta)
        return x, y

    def circlefit(self,positions):
        # split into x and y coordinates
        x = [elem[0] for elem in positions]
        y = [elem[1] for elem in positions]

        lx=len(x)
        x = np.array(x)
        y = np.array(y)
        xx = np.square(x)
        yy = np.square(y)
        xy = np.multiply(x,y)
        
        xxyy = np.add(xx,yy)
        sx = np.sum(x)
        sy = np.sum(y)
        sxx = np.sum(xx)
        syy = np.sum(yy)
        sxy = np.sum(xy)

        # inverting the matrix a=[sx sy lx;sxy syy sy;sxx sxy sx]\[sxx+syy;sum(xxyy.*y);sum(xxyy.*x)];
        a = np.array([[sx,sy,lx],[sxy,syy,sy],[sxx,sxy,sx]])   
        b = np.array([[sxx+syy], [np.sum(np.multiply(xxyy,y))], [np.sum(np.multiply(xxyy,x))]])
        c = np.linalg.solve(a,b)
        xc = 0.5*c[0]
        yc = 0.5*c[1]
        R = np.sqrt(xc**2+yc**2 + c[2])
        # print(xc, yc, R)
        
        return xc, yc, R

    def cartesian2polar(self,x,y):
        radius = math.sqrt(math.pow(x, 2) + math.pow(y, 2))
        radian = math.atan2(y, x)
        degree = math.degrees(radian)
        # print(radius, radian, degree)
        return radius, radian, degree


    def clustering(self,scan):
        '''
        find difference between adjacent points in list, if less than 0.01, create a new cluster, else discard
        '''
        clusters = []
        cluster = []

        for i in range(len(scan)):
            if i == len(scan) - 1:
                break

            if abs(scan[i+1] - scan[i]) < 0.1:
                cluster.append(scan[i])
            else:
                cluster.append(scan[i])
                clusters.append(cluster)
                cluster = []
        
        refined_clusters = []
        outliers = []
        for elem in clusters:
            # print(len(elem), elem)
            if len(elem) > 12:
                refined_clusters.append(elem)
            else:
                outliers.append(elem)

        return refined_clusters, outliers

    def cartesian_clustering(self,coordinates):
        '''
        find eucledian distance between adjacent coordinates, if less than distance, create a new cluster, else discard
        '''
        clusters = []
        cluster = []

        for i in range(len(coordinates)):
            if i == len(coordinates) - 1:
                break
            eucledian_distance = math.sqrt(math.pow(coordinates[i+1][0] - coordinates[i][0], 2) + math.pow(coordinates[i+1][1] - coordinates[i][1], 2))
            # print(i,eucledian_distance)
            if math.sqrt(math.pow(coordinates[i+1][0] - coordinates[i][0], 2) + math.pow(coordinates[i+1][1] - coordinates[i][1], 2)) < 0.01:
                cluster.append(coordinates[i])
            else:
                cluster.append(coordinates[i])
                clusters.append(cluster)
                cluster = []
        
        refined_clusters = []
        outliers = []
        for elem in clusters:
            # print(len(elem), elem)
            if len(elem) > 12:
                refined_clusters.append(elem)
            else:
                outliers.append(elem)

        return refined_clusters, outliers

    def scan2cartesian(self,lidardata):
        positions = []
        lidar_bins = [np.radians(x) for x in range(len(lidardata))]
        increment = 0.01745329238474369
        for degree in range(len(lidardata)):
            distance  = lidardata[degree]
            # radians = lidar_bins[degree]
            radians = degree*increment

            x, y = self.polar2cartesian(distance, radians)
            positions.append([x,y])
        return positions
    

   
    
    def landmark_publisher(self,x,y):
        self.point = Point()
        self.point.x = float(x)
        self.point.y = float(y)
        self.landmarks_pub.publish(self.point)
    
    def cone_marker_publisher(self,xc,yc,R,tag):
        self.marker = Marker()
        # self.marker.lifetime = Duration()
        self.marker.header.frame_id = "base_scan"
        # self.marker.header.stamp = rclpy.get_clock.now()
        self.marker.id = tag
        self.marker.type = Marker.CYLINDER
        self.marker.action = Marker.ADD
        self.marker.scale.x = 0.15
        self.marker.scale.y = 0.15
        self.marker.scale.z = 0.4
        self.marker.color.a = 1.0
        self.marker.color.r = random(1)[0]
        self.marker.color.g = random(1)[0]
        self.marker.color.b = random(1)[0]
        self.marker.pose.orientation.w = 1.0
        self.marker.pose.position.x = -float(xc)
        self.marker.pose.position.y = -float(yc)
        self.marker.pose.position.z = 0.0
        # self.marker.lifetime = self.Duration(0.5)
        self.landmarks_marker_pub.publish(self.marker)
        print("cone marker published")

    def scan_callback(self, msg):
        # get range data
        ranges = list(msg.ranges)

        # convert range data to cartesian coordinates
        cartesian_coordinates = self.scan2cartesian(ranges)

        # cluster the cartesian coordinates
        clusters, outliers = self.cartesian_clustering(cartesian_coordinates)


        # for i in range(len(clusters)):
        #     print(len(clusters[i]), clusters[i])
        #     print('-----')


        # fit circles to the clusters
        tag = 0
        for cluster in clusters:
            try:
                xc, yc, R = self.circlefit(cluster)
                # print(xc, yc, R)
                # convert to float
                xc = float(xc)
                yc = float(yc)
                R = float(R)
                print(xc, yc, R)
                # print(type(xc))
                # publish landmark
                self.landmark_publisher(xc, yc)

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
