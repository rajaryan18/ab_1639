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
depth_image = None
tracker = cv.legacy.TrackerMOSSE_create()
multiTracker = cv.legacy.MultiTracker_create()



def depth_callback(msg):
    global depth_image
    # create OpenCV depth image using defalut passthrough encoding
    try:
        depth_image = CvBridge().imgmsg_to_cv2(msg, desired_encoding='passthrough')
    except CvBridgeError as e:
        pass


def getDepth(x, y):
    global depth_image
    return depth_image[x, y]


def callback(data):
    # Initializing variables
    global cv_image, tracker, multiTracker
    focal_length = 554.387
    center_x = 320.5
    center_y = 240.5
    cf_d = 0
    try:
        bridge = CvBridge()
        frame = bridge.imgmsg_to_cv2(data, "bgr8")
        img_hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

        mask1 = cv.inRange(img_hsv, (0,70,150), (10,255,255))
        mask2 = cv.inRange(img_hsv, (170,70,150), (180,255,255))

        mask = cv.bitwise_or(mask1, mask2)

        (contours,_) = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        for c in contours:
            area = cv.contourArea(c)
            if(area>100):
                x, y, w, h = cv.boundingRect(c)
                #calculating the centre of the frame
                # to avoid repeating the same tomato, we check the distance between this centre and centres of other tomatoes
                multiTracker.add(tracker, frame, (x, y, w, h))
                success, boxes = multiTracker.update(frame)
                for j, newbox in enumerate(boxes):
                    p1 = (int(newbox[0]), int(newbox[1]))
                    p2 = (int(newbox[0] + newbox[2]), int(newbox[1] + newbox[3]))
                    cv.rectangle(frame, p1, p2, (255, 255, 255), 1, 1)
                    cv.circle(frame, (int(newbox[0] + (newbox[2]//2)), int(newbox[1] + (newbox[3]//2))), 1, (255, 255, 255), 2)
                    #cv.putText(frame, ("Ob"+str(j)), (int(newbox[0] + (newbox[2]//2)), int(newbox[1] - 0.5 + (newbox[3]//2))), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                    cX = int(newbox[0] + (newbox[2]//2))
                    cY = int(newbox[1] + (newbox[3]//2))
                    if cX > 470:
                        cX = 470
                    # transforming pixel coordinates to world coordinates
                    cf_d = getDepth(cX, cY)
                    world_x = ((cX - center_x)/focal_length)*cf_d
                    world_y = ((cY - center_y)/focal_length)*cf_d
                    world_z = cf_d

                # broadcasting TF for each aruco marker
                    br = tf2_ros.TransformBroadcaster()
                    t = geometry_msgs.msg.TransformStamped()
                    t.header.stamp = rospy.Time.now()
                    t.header.frame_id = "camera_rgb_frame2"
                    t.child_frame_id = str(j)

                    if not (np.isnan(world_x) or np.isnan(world_y) or np.isnan(world_z)):
                        # putting world coordinates coordinates as viewed for rgb_frame2 cam
                        t.transform.translation.x = world_x
                        t.transform.translation.y = world_y
                        t.transform.translation.z = world_z
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
        pass

def main(args):
    rospy.init_node('aruco_tf', anonymous=True)
    #subscribing to /ebot/camera1/image_raw topic which is the image frame of sjcam camera
    rospy.Subscriber("/camera/color/image_raw2", Image, callback, queue_size = 10)
    rospy.Subscriber("/camera/depth/image_raw2", Image, depth_callback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
