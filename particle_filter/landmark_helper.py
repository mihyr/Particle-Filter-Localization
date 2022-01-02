#!/usr/bin/env python3
# Landmark detection helper functions
import numpy as np
import math

def polar2cartesian(r, theta):
    x = r * math.cos(theta)
    y = r * math.sin(theta)
    return x, y

def circlefit(positions):
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
    
    return float(xc), float(yc), float(R)

def cartesian2polar(x,y):
    radius = math.sqrt(math.pow(x, 2) + math.pow(y, 2))
    radian = math.atan2(y, x)
    degree = math.degrees(radian)
    # print(radius, radian, degree)
    return radius, radian, degree


def clustering(scan):
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

def cartesian_clustering(coordinates):
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
        if coordinates[i] != [0,0]:
            if eucledian_distance < 0.01:
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
        elif 0 < len(elem) < 12:
            outliers.append(elem)
        else:
            pass

    return refined_clusters, outliers

def scan2cartesian(lidardata, increment):
    positions = []
    # lidar_bins = [np.radians(x) for x in range(len(lidardata))]
    
    for degree in range(len(lidardata)):
        distance  = lidardata[degree]
        # radians = lidar_bins[degree]
        radians = degree*increment
        x, y = polar2cartesian(distance, radians)
        positions.append([-x,-y])
    return positions