#!/usr/bin/env python3

import rospy

from homodeus_msgs.msg import BoundingBoxes, BoundingBox, ObjectDetection

from homodeus_library.homodeus_precomp import *
from base_navigation.NavSelector import NavSelector

from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel
import numpy as np


def f(a) -> None: #As we see, we can get events from the NavSelector in the controller
  print(f"Voici ce qu'on recoit : {convGoalStatus(a)}")


class ApproachClient():
  __CAMERA_INFO_TOPIC: str = '/xtion/rgb/camera_info'

  def __init__(self):
    initRosNode("approche_client")

    self.__bridge   = CvBridge()
    self.__depth_image = None

    # https://docs.ros.org/en/api/image_geometry/html/python/
    self.__model    = PinholeCameraModel()
    self.__cam_info = rospy.wait_for_message(self.__CAMERA_INFO_TOPIC, CameraInfo, timeout=10)
    # rospy.loginfo(self.__cam_info)
    self.__model.fromCameraInfo(self.__cam_info)

    __camera_P = [522.1910329546544, 0.0, 320.5, -0.0, 0.0, 522.1910329546544, 240.5, 0.0, 0.0, 0.0, 1.0, 0.0]
    __camera_P = np.reshape(__camera_P, (3,4))
    self.__camera_Pinv = np.linalg.pinv(__camera_P)

    self.__navigator = NavSelector()
    self.__navigator.ConnectCallBack(f)
    self.__approach_dist = 1.375
    self.__tolerance = 0.15

    self.__tf_listener = tf.TransformListener()

    # JT, Utiliser ce subscriber si vous voulez utiliser les rosbags & pseudo_facedetection
    # self.__camera_sub = rospy.Subscriber('/proc_output_depth_image', Float32MultiArray, self.__pseduo_camera_callback, queue_size=5)
    self.__camera_sub = rospy.Subscriber('xtion/depth_registered/image_raw', Image, self.__camera_callback, queue_size=5)

    # JT, Utiliser ce subscriber si vous voulez utiliser les rosbags & pseudo_facedetection
    # self.__face_sub =rospy.Subscriber('proc_output_face_positions', BoundingBoxes, self.__face_callback, queue_size=5)
    self.__face_sub =rospy.Subscriber('/homodeus/perception/detection', ObjectDetection, self.__face_callback, queue_size=5)

    rospy.on_shutdown(self.__close_connection_node)
    rospy.loginfo("approach client initialized")


  def __ownProjectPixelTo3dRay(self, ctr_pt:list, disparity: float) -> Vector3:
    point_2d = point_2d = np.array([[ctr_pt[0], ctr_pt[1], 1]])
    (x, y, z,_) = self.__camera_Pinv @ (point_2d * disparity).T
    return Vector3(x[0], y[0], disparity)


  def __prepareMsgObjectDetection(self, what_is: str, start_pt: list, end_pt: list) -> ObjectDetection:
    up_left_pt: Point     = Point( start_pt[0] if start_pt[0] < end_pt[0] else end_pt[0], 
                                   start_pt[1] if start_pt[1] < end_pt[1] else end_pt[1], 0 )
    down_right_pt: Point  = Point( start_pt[0] if start_pt[0] > end_pt[0] else end_pt[0], 
                                   start_pt[1] if start_pt[1] > end_pt[1] else end_pt[1], 0 )
    width: int            = abs( start_pt[0] - end_pt[0] )
    height: int           = abs( start_pt[1] - end_pt[1] )
    padding_width: int    = int(width * 0.025)
    padding_height: int   = int(height * 0.025)

    # Trouver la zone de l'image de profondeur associée a la zone de l'image rgb
    face_depth_view   = self.__depth_image[( up_left_pt.y + padding_height ): ( down_right_pt.y - padding_height ),
                                           ( up_left_pt.x + padding_width ): ( down_right_pt.x - padding_width ) ]
    object_dist: float  = np.nanmean(face_depth_view)

    # Trouver les coordonnées 3D a partir des coordonnées 2D pixels
    center_pt: list = [0,0]
    center_pt[0]  = int(up_left_pt.x + width / 2)
    center_pt[1]  = int(up_left_pt.y + height / 2)
    point_3d      = self.__model.projectPixelTo3dRay(center_pt)
    print('self.__model.projectPixelTo3dRay(center_pt).z', point_3d[2])
    point_3d      = Point(point_3d[0] * object_dist, point_3d[1] * object_dist, point_3d[2] * object_dist)
    # TODO puisque __model.projectPixelTo3dRay ne semble pas donne la bonne valeur (un z != 1 !?)
    point_3d      = self.__ownProjectPixelTo3dRay(center_pt, object_dist)
    print('point_3d\n', point_3d)
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
    print('distance', object_dist)
    print('angle', angle)
    print('pose\n', map_pose.pose)

    # return msg objectDetection
    return objectDetected


  # def __face_callback(self, detections : BoundingBoxes) -> None:
  def __face_callback(self, object_detected : ObjectDetection) -> None:
    rospy.loginfo("approach client face CB")

    # if self.__depth_image is not None:
    if object_detected.header.frame_id == "person":
      face: BoundingBox   = object_detected.box
      height: float       = face.height
      width: float        = face.width
      min_y, max_y        = face.y, face.y + height
      min_x, max_x        = face.x, face.x + width

      object_dist   = object_detected.distance
      # TODO Debug self.__prepareMsgObjectDetection
      # self.__prepareMsgObjectDetection('person', start_pt=[min_x, min_y], end_pt=[max_x, max_y])

      if object_dist > (1+self.__tolerance)*self.__approach_dist and 0 < object_dist:
        dist_to_approach = object_dist - self.__approach_dist

        center_pt: list = [0,0]
        center_pt[0]  = min_x + width / 2
        center_pt[1]  = min_y + height / 2
        point_3d      = self.__model.projectPixelTo3dRay(center_pt)
        point_3d      = Point(point_3d[0] * dist_to_approach, point_3d[1] * dist_to_approach, point_3d[2] * dist_to_approach)
        # TODO puisque __model.projectPixelTo3dRay ne semble pas donne la bonne valeur (un z != 1 !?)
        point_3d      = self.__ownProjectPixelTo3dRay(center_pt, dist_to_approach)

        approach_point : Vector3 = Vector3(dist_to_approach, -point_3d.x, -point_3d.y) # l'axe des x (profondeur) reste // au plan xy de /map
        angle : float = atan2( point_3d.x, point_3d.z )
        # print('approach_point', approach_point.x, approach_point.y, approach_point.z)
        # print('angle', angle)
        euler2quat = tf.transformations.quaternion_from_euler(0,0,-angle)
        q : Quaternion = Quaternion(euler2quat[0],euler2quat[1],euler2quat[2],euler2quat[3])

        # frame_from = "/xtion_rgb_optical_frame"     # l'axe des z (profondeur) change d'orientation en fct de la tete du robot
        frame_from = "/head_1_link"                 # l'axe des x (profondeur) reste // au plan xy de /map

        pose = PoseStamped()
        pose.pose.position    = approach_point
        pose.pose.orientation = q
        pose.header.stamp     = rospy.Time(0)
        pose.header.frame_id  = frame_from
        map_pose              = self.__tf_listener.transformPose("/map", pose)
        # print(map_pose)
        print(quarternion2euler(map_pose.pose.orientation).z)

        print('GetState()', self.__navigator.GetState())
        if self.__navigator.GetState() != GoalStatus.PENDING and \
           self.__navigator.GetState() != GoalStatus.ACTIVE:
          rospy.loginfo("approaching detected client")

          center_face_in_img = quarternion2euler(map_pose.pose.orientation).z

          res = 'O' # input('Oui/Non?')
          if res == 'O':
            self.__navigator.AddGoal(map_pose.pose.position.x, map_pose.pose.position.y, 0, center_face_in_img,  "Approche")
            self.__navigator.SendGoal()
            # rospy.loginfo("result is:" + str(result))


  # Vrai fct «callback» pour l'image de profondeur
  def __camera_callback(self, image : Image) -> None:
    self.__height = image.height
    self.__width = image.width
    self.__depth_image = self.__bridge.imgmsg_to_cv2(image, "passthrough")


  # Fct «callback» utilisee pour les rosbags & PseudoDepthImage
  def __pseduo_camera_callback(self, image : Float32MultiArray) -> None:
    self.__height = image.layout.dim[0].size
    self.__width = image.layout.dim[1].size
    self.__depth_image = np.reshape(image.data, (self.__height, self.__width))


  def __close_connection_node(self) -> None:
    # self.__camera_sub.unregister()
    self.__face_sub.unregister()

    rospy.loginfo("approach client shutdown")
