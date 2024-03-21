#!/usr/bin/env python3

import rospy

from homodeus_msgs.msg import FacePositions, BoundingBox, ObjectDetection
from image_geometry import StereoCameraModel

from homodeus_library.homodeus_precomp import *
from base_navigation.NavSelector import NavSelector

import numpy as np


def f(a) -> None: #As we see, we can get events from the NavSelector in the controller
  print(f"Voici ce qu'on recoit : {convGoalStatus(a)}")


class ApproachClient():
  def __init__(self):
    initRosNode("approche_client")

    self.__bridge = CvBridge()

    # https://docs.ros.org/en/api/image_geometry/html/python/
    self.__cameraModel = StereoCameraModel()  # TODO, utiliser cet objet a la place de tout faire tf 2D -> 3D a la main ...
    # TODO, Recuperer info sur topic (Actuellement, coder a la dure. A NE PAS FAIRE)
    # info camera intriseque : xtion/depth_registered/camera_info == /xtion/rgb/camera_info
    self.__camera_P = [522.1910329546544, 0.0, 320.5, -0.0, 0.0, 522.1910329546544, 240.5, 0.0, 0.0, 0.0, 1.0, 0.0]
    self.__camera_P = np.reshape(self.__camera_P, (3,4))
    self.__camera_Pinv = np.linalg.pinv(self.__camera_P)
    self.__depth_image = None

    self.__navigator = NavSelector()
    self.__navigator.ConnectCallBack(f)
    self.__approach_dist = 1.375
    self.__tolerance = 0.15

    self.__tf_listener = tf.TransformListener()

    # JT, Utiliser ce subscriber si vous voulez utiliser les rosbags & pseudo_facedetection
    # self.__camera_sub = rospy.Subscriber('/proc_output_depth_image', Float32MultiArray, self.__camera_callback, queue_size=5)
    # self.__camera_sub = rospy.Subscriber('xtion/depth_registered/image_raw', Image, self.__camera_callback, queue_size=5)

    # JT, Utiliser ce subscriber si vous voulez utiliser les rosbags & pseudo_facedetection
    # self.__face_sub =rospy.Subscriber('proc_output_face_positions', FacePositions, self.__face_callback, queue_size=5)
    self.__face_sub =rospy.Subscriber('/homodeus/perception/detection', ObjectDetection, self.__face_callback, queue_size=5)

    rospy.on_shutdown(self.__close_connection_node)
    rospy.loginfo("approach client initialized")


  def __sendObjectDetection(self, what_is: str, start_pt: np.ndarray, end_pt: np.ndarray) -> None:
    min_x             = start_pt.x if start_pt.x < end_pt.x else end_pt.x
    max_x             = start_pt.x if start_pt.x > end_pt.x else end_pt.x
    min_y             = start_pt.y if start_pt.y < end_pt.y else end_pt.y
    max_y             = start_pt.y if start_pt.y > end_pt.y else end_pt.y
    width             = max_x - min_x
    height            = max_y - min_y

    # Trouver la zone de l'image de profondeur associée a la zone de l'image rgb
    face_depth_view   = self.__depth_image[min_y: max_y, min_x: max_x]
    # TODO Ajouter une marge interne
    object_dist: float  = np.nanmean(face_depth_view)
    px = min_x + width / 2       # central point x (pixel)
    py = min_y + height / 2      # central point y (pixel)
    point_2d = np.array([[px, py, 1]])

    # Initialize homodeus_msgs::ObjectDetection to send on topic
    # TODO Revoir le calcul et d'où vient self.__camera_Pinv
    # https://docs.ros.org/en/api/image_geometry/html/python/
    # Commentaire 
    # On initialise l’objet avec le message provenant du « camera_info » de la caméra, puis on peut utiliser des fonctions très pratiques comme « projectPixelTo3dRay », qui donne un vecteur unitaire qui lui ensuite peut être multiplié par le « d » que vous trouvez dans la matrice de profondeur. C’est qu’en fait la carte de profondeur est déjà en « z », donc on peut sauter une étape. Le détail est que ce n’est pas toujours clair dans quelles unités les données sont, mais l’avantage de passer par PinholeCameraModel est qu’il se base entièrement sur le format utilisé par camera_info et ne devrait pas vous donnez de soucis.
    objectDetected: ObjectDetection = ObjectDetection()
    (offset_x, offset_y,_,_) = self.__camera_Pinv @ (point_2d * object_dist).T
    approach_point : Vector3 = Vector3(object_dist, -offset_x[0], -offset_y[0])
    angle : float = atan2(-offset_x[0], object_dist)
    euler2quat = tf.transformations.quaternion_from_euler( 0, 0, angle )
    q : Quaternion = Quaternion(euler2quat[0],euler2quat[1],euler2quat[2],euler2quat[3])

    frame_from  = "/head_1_link"
    frame_to    = "/map"
    pose = PoseStamped()
    pose.pose.position    = approach_point
    pose.pose.orientation = q
    pose.header.stamp     = rospy.Time(0)
    pose.header.frame_id  = frame_from
    map_pose              = self.__tf_listener.transformPose(frame_to, pose)

    # Prepare msg header
    objectDetected.header.seq       = 0
    objectDetected.header.stamp     = rospy.Time(0)
    objectDetected.header.frame_id  = what_is

    # Prepare msg box
    objectDetected.box.x            = min_x   # start_pt.x if start_pt.x < end_pt.x else end_pt.x
    objectDetected.box.y            = min_y   # start_pt.y if start_pt.y < end_pt.y else end_pt.y
    objectDetected.box.width        = width   # abs(start_pt.x - end_pt.x)
    objectDetected.box.height       = height  # abs(start_pt.y > end_pt.y)

    # Données géospatiales de l'objet détecté selon pose du robot
    objectDetected.distance         = object_dist
    objectDetected.angle            = angle
    objectDetected.pose             = map_pose.pose

    # send to the topic
    self.__object_detected_pub.publish(objectDetected)


  # def __face_callback(self, detections : FacePositions) -> None:
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

      print("if", object_dist, '>', (1+self.__tolerance)*self.__approach_dist, "and 0 <", object_dist)
      if object_dist > (1+self.__tolerance)*self.__approach_dist and 0 < object_dist:
        dist_to_approach = object_dist - self.__approach_dist

        px = min_x + width / 2       # central point x (pixel)
        py = min_y + height / 2      # central point y (pixel)
        point_2d = np.array([[px, py, 1]])
        print('point_2d', point_2d)

        (offset_x, offset_y,_,_) = self.__camera_Pinv @ (point_2d * dist_to_approach).T
        print('poind_3d', offset_x[0], offset_y[0], dist_to_approach)

        approach_point : Vector3 = Vector3(dist_to_approach, -offset_x[0], -offset_y[0]) # l'axe des x (profondeur) reste // au plan xy de /map
        print('approach_point', approach_point.x, approach_point.y, approach_point.z)
        angle : float = atan2(-offset_x[0], dist_to_approach)
        print('angle', angle)
        euler2quat = tf.transformations.quaternion_from_euler(0,0,angle)
        q : Quaternion = Quaternion(euler2quat[0],euler2quat[1],euler2quat[2],euler2quat[3])

        # frame_from = "/xtion_rgb_optical_frame"     # l'axe des z (profondeur) change d'orientation en fct de la tete du robot
        frame_from = "/head_1_link"                 # l'axe des x (profondeur) reste // au plan xy de /map

        pose = PoseStamped()
        pose.pose.position    = approach_point
        pose.pose.orientation = q
        pose.header.stamp     = rospy.Time(0)
        pose.header.frame_id  = frame_from
        map_pose              = self.__tf_listener.transformPose("/map", pose)
        print(map_pose)
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
  def __camera_callback(self, image : Float32MultiArray) -> None:
    self.__height = image.layout.dim[0].size
    self.__width = image.layout.dim[1].size
    self.__depth_image = np.reshape(image.data, (self.__height, self.__width))


  def __close_connection_node(self) -> None:
    # self.__camera_sub.unregister()
    self.__face_sub.unregister()

    rospy.loginfo("approach client shutdown")
