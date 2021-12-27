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
import math


class EuclideanDistTracker:
    def __init__(self):
        # Store the center positions of the objects
        self.center_points = {}
        # Keep the count of the IDs
        # each time a new object id detected, the count will increase by one
        self.id_count = -1


    def update(self, objects_rect):
        # Objects boxes and ids
        objects_bbs_ids = []

        # Get center point of new object
        for rect in objects_rect:
            x, y, w, h = rect
            cx = (x + x + w) // 2
            cy = (y + y + h) // 2

            # Find out if that object was detected already
            same_object_detected = False
            for i, pt in self.center_points.items():
                dist = math.hypot(cx - pt[0], cy - pt[1])

                if dist < 16:
                    self.center_points[i] = (cx, cy)
                    objects_bbs_ids.append([x, y, w, h, i])
                    same_object_detected = True
                    break

            # New object is detected we assign the ID to that object
            if same_object_detected is False:
                self.center_points[self.id_count] = (cx, cy)
                objects_bbs_ids.append([x, y, w, h, self.id_count])
                self.id_count += 1

        # Clean the dictionary by center points to remove IDS not used anymore
        new_center_points = {}
        for obj_bb_id in objects_bbs_ids:
            _, _, _, _, object_id = obj_bb_id
            center = self.center_points[object_id]
            new_center_points[object_id] = center

        # Update dictionary with IDs not used removed
        self.center_points = new_center_points.copy()
        return objects_bbs_ids


cv_image = None
depth_image = None
tracker = EuclideanDistTracker()


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
    global cv_image, tracker
    focal_length = 554.387
    center_x = 320.5
    center_y = 240.5
    cf_d = 0
    try:
        bridge = CvBridge()
        frame = bridge.imgmsg_to_cv2(data, "bgr8")
        #resizing the rgb image to match the depth image
        img_hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

        mask1 = cv.inRange(img_hsv, (0,70,150), (10,255,255))
        mask2 = cv.inRange(img_hsv, (170,70,150), (180,255,255))

        mask = cv.bitwise_or(mask1, mask2)

        (contours,_) = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        detections = []
        for c in contours:
            area = cv.contourArea(c)
            if(area>100):
                x, y, w, h = cv.boundingRect(c)
                #calculating the centre of the frame
                cX = int(x) + (w//2)
                cY = int(y) + (h//2)
                cv.rectangle(frame, (x, y), (x+w, y+h), (255, 255, 255), 2)
                detections.append([x, y, w, h])
                boxes_ids = tracker.update(detections)
                if len(boxes_ids) == 1:
                    boxes_ids[0][4] = 0
                if len(boxes_ids) > 1:
                    if boxes_ids[0][0] < boxes_ids[1][0]:
                        boxes_ids[0][4] = 1
                        boxes_ids[1][4] = 0
                    else:
                        boxes_ids[1][4] = 1
                        boxes_ids[0][4] = 0

                for box_id in boxes_ids:
                    x, y, w, h, i = box_id
                    cv.putText(frame, "Ob"+str(i), (x, y-5), cv.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 1)
                    cv.rectangle(frame, (x, y), (x+w, y+h), (255, 255, 255), 2)
                    # transforming pixel coordinates to world coordinates
                    if cX > 470:
                        cX = 470
                    if cY > 630:
                        cY = 630
                    cf_d = getDepth(cX, cY)*3.0
                    world_x = ((cX - center_x)/focal_length)*cf_d
                    world_y = ((cY - center_y)/focal_length)*cf_d
                    world_z = cf_d
                    #print("+++++++++++++++++++++++++++++++")
                    #print(world_x)
                    #print(world_y)
                    #print(world_z)
                    #print("+++++++++++++++++++++++++++++++")

                    # broadcasting TF for each aruco marker
                    br = tf2_ros.TransformBroadcaster()
                    t = geometry_msgs.msg.TransformStamped()
                    t.header.stamp = rospy.Time.now()
                    t.header.frame_id = "camera_rgb_frame2"
                    t.child_frame_id = "Ob" + str(i)

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
                        #print("++++++++++++++++++++++++++++++++++++++++++")
                        #print(t)
                        #print("++++++++++++++++++++++++++++++++++++++++++")
                        br.sendTransform(t)
        cv.imshow("frame", frame)
        cv.waitKey(1)

    except CvBridgeError as e:
        pass

def main(args):
    rospy.init_node('aruco_tf', anonymous=True)
    rospy.Subscriber("/camera/color/image_raw2", Image, callback)
    rospy.Subscriber("/camera/depth/image_raw2", Image, depth_callback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)