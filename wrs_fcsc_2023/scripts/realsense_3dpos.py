#!/usr/bin/env python3
import rospy
import pyrealsense2 as rs
import cv2
import numpy as np
from wrs_fcsc_2023.msg import real_pub

# calculate 3D point from pixel
def deproject_pixel_to_point(pixel_x, pixel_y, depth):
    depth_point = []
    
    # calculation (using param from /camera/depth/camera_info)
    x = (pixel_x - 321.7812805175781) / 380.3281555175781
    y = (pixel_y - 238.0182342529297) / 380.3281555175781

    # in the original code, they use BROWN_CONRADY, but we have to use plum_bob
    # however, the distortion coefficients is same, we write the same code
    r2 = x * x + y * y
    f = 1 + 0*r2 + 0*r2*r2 + 0*r2*r2*r2
    ux = x*f + 2*0*x*y + 0*(r2 + 2*x*x)
    uy = y*f + 2*0*x*y + 0*(r2 + 2*y*y)
    x = ux
    y = uy

    # output 
    depth_point.append(depth * x )  # cm -> m
    depth_point.append(depth * y )
    depth_point.append(depth)
    return [depth_point[0], depth_point[1], depth_point[2]]

def callback(msg):
    depth_point = deproject_pixel_to_point(msg.pixel_x, msg.pixel_y, msg.depth)
    print('--------------------------------')
    print('x = ', depth_point[0])
    print('y = ', depth_point[1])
    print('d = ', depth_point[2])


def subscriber():
    rospy.init_node('realsense_3dpos', anonymous=True)
    rospy.Subscriber('real_pub', real_pub, callback)
    rospy.spin()

if __name__ == '__main__':
    rospy.loginfo('0')
    subscriber()
