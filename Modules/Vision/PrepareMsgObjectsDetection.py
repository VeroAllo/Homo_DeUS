#!/usr/bin/env python3

import rospy

from homodeus_msgs.msg import ObjectDetection, ObjectsDetection
from homodeus_library.homodeus_precomp import *

from cv_bridge import CvBridge
import numpy as np


# A ajouter apres le init_node dans detect.py
self.__bridge = CvBridge()
self.__depth_image = None
self.__objects_detection_pub = rospy.Publisher('/Homodeus/Perception/Detect', ObjectsDetection, timeout=None)

# A ajouter apres l'autre wait_for_message
image : Image = rospy.wait_for_message('xtion/depth_registered/image_raw', Image, timeout=None)
self.__depth_image = self.__bridge.imgmsg_to_cv2(image, "passthrough")

# A ajouter apres la for de detect boxes
msgHBBA: ObjectsDetection = obj.__prepareMsgObjectDetection(self.__depth_image, labels, labels, labels)
self.__objects_detection_pub.publish(msgHBBA)


class PrepareMsgObjectsDetection():
  def __init__(self):
    __camera_P = [522.1910329546544, 0.0, 320.5, -0.0, 0.0, 522.1910329546544, 240.5, 0.0, 0.0, 0.0, 1.0, 0.0]
    __camera_P = np.reshape(__camera_P, (3,4))
    self.__camera_Pinv = np.linalg.pinv(__camera_P)
    self.__tf_listener = tf.TransformListener()


  def __ownProjectPixelTo3dRay(self, ctr_pt:list, disparity: float) -> Vector3:
    point_2d = point_2d = np.array([[ctr_pt[0], ctr_pt[1], 1]])
    (x, y, z,_) = self.__camera_Pinv @ (point_2d * disparity).T
    return Vector3(x[0], y[0], disparity)


  def __prepareMsgObjectsDetection(self, __depth_image: np.ndarray, labels: list, start_pts: list, end_pts: list) -> ObjectsDetection:
    objectsDetected: ObjectsDetection = ObjectsDetection()
    objectsDetected.header.seq       = 0
    objectsDetected.header.stamp     = rospy.Time(0)
    objectsDetected.header.frame_id  = 'objects list'

    for idx, label in enumerate(labels):
        objectDetection: ObjectDetection = ObjectDetection()
        objectDetection = self.__prepareMsgObjectDetection(__depth_image, label, start_pts[idx], end_pts[idx])
        objectsDetected.objects.append(objectDetection)

    return objectsDetected


  def __prepareMsgObjectDetection(self, __depth_image: np.ndarray, what_is: str, start_pt: list, end_pt: list) -> ObjectDetection:
    up_left_pt: Point     = Point( start_pt[0] if start_pt[0] < end_pt[0] else end_pt[0], 
                                   start_pt[1] if start_pt[1] < end_pt[1] else end_pt[1], 0 )
    down_right_pt: Point  = Point( start_pt[0] if start_pt[0] > end_pt[0] else end_pt[0], 
                                   start_pt[1] if start_pt[1] > end_pt[1] else end_pt[1], 0 )
    width: int            = abs( start_pt[0] - end_pt[0] )
    height: int           = abs( start_pt[1] - end_pt[1] )
    padding_width: int    = int(width * 0.025)
    padding_height: int   = int(height * 0.025)

    # Trouver la zone de l'image de profondeur associée a la zone de l'image rgb
    face_depth_view   = __depth_image[( up_left_pt.y + padding_height ): ( down_right_pt.y - padding_height ),
                                      ( up_left_pt.x + padding_width ): ( down_right_pt.x - padding_width ) ]
    object_dist: float  = np.nanmean(face_depth_view)

    # Trouver les coordonnées 3D a partir des coordonnées 2D pixels
    center_pt: list = [0,0]
    center_pt[0]  = int(up_left_pt.x + width / 2)
    center_pt[1]  = int(up_left_pt.y + height / 2)
    point_3d      = self.__ownProjectPixelTo3dRay(center_pt, object_dist)
    # print('point_3d\n', point_3d)
    angle : float = atan2( point_3d.x, point_3d.z )

    euler2quat    = tf.transformations.quaternion_from_euler( 0, 0, -angle )
    q: Quaternion = Quaternion( euler2quat[0], euler2quat[1], euler2quat[2], euler2quat[3] )

    frame_from  = "/head_1_link"
    frame_to    = "/map"
    pose = PoseStamped()
    pose.pose.position.x  =  point_3d.z
    pose.pose.position.y  = -point_3d.x
    pose.pose.position.z  = -point_3d.y
    pose.pose.orientation = q
    pose.header.stamp     = rospy.Time(0)
    pose.header.frame_id  = frame_from
    map_pose              = self.__tf_listener.transformPose(frame_to, pose)

    # Initialize homodeus_msgs::ObjectDetection to send on topic
    objectDetected: ObjectDetection = ObjectDetection()

    # Prepare msg header
    objectDetected.header.seq       = 0
    objectDetected.header.stamp     = rospy.Time(0)
    objectDetected.header.frame_id  = what_is

    # Prepare msg box
    objectDetected.box.x            = up_left_pt.x
    objectDetected.box.y            = up_left_pt.y
    objectDetected.box.width        = width   
    objectDetected.box.height       = height  

    # Données géospatiales de l'objet détecté selon pose du robot
    objectDetected.distance         = object_dist
    objectDetected.angle            = angle
    objectDetected.pose             = map_pose.pose
    # print('distance', object_dist)
    # print('angle', angle)
    # print('pose\n', map_pose.pose)

    # return msg objectDetection
    return objectDetected