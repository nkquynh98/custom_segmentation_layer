#!/usr/bin/env python

import sensor_msgs.point_cloud2 as pc2
import rospy
from sensor_msgs.msg import PointCloud2, LaserScan
import laser_geometry.laser_geometry as lg
import math

def scan_cb(msg):
    # convert the message of type LaserScan to a PointCloud2
    pc2_msg = lp.projectLaser(msg)

    # now we can do something with the PointCloud2 for example:
    # publish it
    pc_pub.publish(pc2_msg)
    
    # convert it to a generator of the individual points
    point_generator = pc2.read_points(pc2_msg)
    

    # we can access the point list with an index, each element is a namedtuple
    # we can access the elements by name, the generator does not yield namedtuples!
    # if we convert it to a list and back this possibility is lost


if __name__ == '__main__' :
    rospy.init_node("laserscan_to_pointcloud")

    lp = lg.LaserProjection()

    pc_pub = rospy.Publisher("filtered_cloud", PointCloud2, queue_size=1)

    rospy.Subscriber("/front/scan", LaserScan, scan_cb, queue_size=1)
    rospy.spin()