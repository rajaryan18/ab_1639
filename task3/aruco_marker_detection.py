#!/usr/bin/env python3
import cv2 as cv
import numpy as np
import roslib
import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import tf2_ros
import geometry_msgs.msg
import tf_conversions

cv_image = None
cf_u = 0
cf_v = 0
cf_d = 0


def depth_callback(msg):
    global cf_d, cf_u, cf_v
    # create OpenCV depth image using defalut passthrough encoding
    try:
        depth_image = CvBridge().imgmsg_to_cv2(msg, desired_encoding='passthrough')
    except CvBridgeError as e:
        print(e)

    # using green ball (u, v) position, find depth value of Crazyflie point and divide by 1000
    # to change millimeters into meters (for Kinect sensors only)
    cf_d = depth_image[cf_v, cf_u]/1.0


def callback(data):
    # Initializing variables
    global cv_image, cf_d
    focal_length = 554.387
    center_x = 320.5
    center_y = 240.5
    aruco_dimension = 0.1
    try:
        bridge = CvBridge()
        frame = bridge.imgmsg_to_cv2(data, "bgr8")
        img_hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

        mask1 = cv.inRange(img_hsv, (0,90,50), (5,255,255))
        mask2 = cv.inRange(img_hsv, (175,90,50), (180,255,255))

        mask = cv.bitwise_or(mask1, mask2)


        (contours,_) = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        for c in contours:
            area = cv.contourArea(c)
            if(area>200):
                x, y, w, h = cv.boundingRect(c)
                frame = cv.rectangle(frame, (x, y-10), (x+w, y+h+10), (255, 0, 0), 1)
                
                pixel_width = h
                cX = (x + x+w) // 2
                cY = (y + y+h) // 2
                frame = cv.circle(frame, (cX, cY), 1, (255, 255, 255), 1)

                # obtain depth for each ArUco marker
                distance = (focal_length*aruco_dimension)/pixel_width
                rospy.Subscriber("/camera/depth/image_raw2", Image, depth_callback)
                # transforming pixel coordinates to world coordinates
                world_x = ((cX - center_x)/focal_length)*cf_d
                world_y = ((cY - center_y)/focal_length)*cf_d
                world_z = cf_d

                # broadcasting TF for each aruco marker
                br = tf2_ros.TransformBroadcaster()
                t = geometry_msgs.msg.TransformStamped()
                t.header.stamp = rospy.Time.now()
                t.header.frame_id = "camera_rgb_frame2"
                t.child_frame_id = "Ob"+str(x)

                # putting world coordinates coordinates as viewed for sjcam frame
                t.transform.translation.x = world_z
                t.transform.translation.y = -world_x
                t.transform.translation.z = world_y
                # not extracting any orientation thus orientation is (0, 0, 0)
                q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
                t.transform.rotation.x = q[0]
                t.transform.rotation.y = q[1]
                t.transform.rotation.z = q[2]
                t.transform.rotation.w = q[3]

                br.sendTransform(t)
        cv.imshow("frame", frame)
        cv.waitKey(1)
    except CvBridgeError as e:
        print(e)


def main(args):
    rospy.init_node('aruco_tf', anonymous=True)
    #subscribing to /ebot/camera1/image_raw topic which is the image frame of sjcam camera
    image_sub = rospy.Subscriber("/camera/color/image_raw2", Image, callback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
