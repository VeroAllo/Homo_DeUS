#!/usr/bin/env python

# https://wiki.ros.org/ROS/Tutorials/CreatingPackage
# https://docs.ros.org/en/melodic/api/catkin/html/howto/format2/installing_python.html
# https://docs.ros.org/en/diamondback/api/rospy/html/rospy.client-module.html

import rospy
from rospy import Publisher, Rate, Subscriber
from geometry_msgs.msg import Point, Pose , PoseWithCovarianceStamped, Quaternion, Vector3
from homodeus_msgs.msg import RobotPoseStamped
from nav_msgs.msg import Odometry

from math import atan2, pi, sqrt
from numpy import around, mean, square


# https://github.com/introlab/hbba_lite/blob/main/README.md
# https://wiki.ros.org/amcl

# http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html
# topic : /mobile_base_controller/odom (nav_msgs/Odometry) 

# ################################################################
# Noeud Perception : AMCL (Estimation de la pose du robot + Sa confiance)
#   Possiblite de 'remap' le noeud
#   Entrees (capteur) [topic]
#     - pose initiale (texte) [/initialpose]
#     - numerisation env. 2D (lidar) (/scan_raw)
#     - grille de la carte (image) [/map]
#   Sorties (perception) [topic]
#     - estimation de la pose () [/amcl_pose/pose/pose]
#     ou calcul de la pose () [/mobile_base_controller/odom/pose]
#     - sa confiance () [/amcl_pose/pose/covariance]


# TODO
#   Si perdu, atteindre que la covariance redevient sous le seuil d'acceptation
#   avant de retransmettre la pose du robot avec le decalage AMCL.


def quarternion2euler(q: Quaternion) -> Vector3:
  euler : Vector3 = Vector3()

  # roll (x-axis rotation)
  sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
  cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
  euler.x = atan2(sinr_cosp, cosr_cosp)

  # pitch (y-axis rotation)
  sinp = sqrt(1 + 2 * (q.w * q.y - q.x * q.z))
  cosp = sqrt(1 - 2 * (q.w * q.y - q.x * q.z))
  euler.y = 2 * atan2(sinp, cosp) - pi / 2

  # yaw (z-axis rotation)
  siny_cosp = 2 * (q.w * q.z + q.x * q.y)
  cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
  euler.z = atan2(siny_cosp, cosy_cosp)

  return euler


class RobotPose():
  def __init__(self) -> None:
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'robot_pose' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('robot_pose', anonymous=True)

    self.__amcl_pose: Vector3     = None
    self.__odom_pose: Vector3     = None
    self.__initial_pose_call: bool= True
    self.__offset_pose: Vector3   = Vector3()


    # Subscriber
    self.__amcl_pose_sub: Subscriber = Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.__amcl_pose_subscriber)
    self.__odom_pose_sub: Subscriber = Subscriber("/mobile_base_controller/odom", Odometry, self.__odom_pose_subscriber)
    self.__pose_initial_sub: Subscriber = Subscriber("/initialpose", PoseWithCovarianceStamped, self.__initial_pose_subscriber)

    # Publisher
    self.__rate: Rate = Rate(10) # 10hz
    self.__robot_pose_pub: Publisher = Publisher("/homodeus/perception/robot_pose", RobotPoseStamped, queue_size=1)

    rospy.on_shutdown(self.__node_shutdown)
    rospy.loginfo("Robot Pose initialized")
    self.__robot_pose_publisher()


  def __amcl_pose_subscriber(self, pose_wcs: PoseWithCovarianceStamped) -> None:
    pose:       Pose      = pose_wcs.pose.pose
    position:   Point     = pose.position
    covar_mat:  float[36] = pose_wcs.pose.covariance
    # mat:        float[36] =  around(covar_mat, 4)

    # mean_mat:   float[36] = mean(covar_mat)                 # > 0.0025 -> lost, < 0.0005 -> A+
    self.__msqrt: float   = mean(square(covar_mat))   # > 0.00005 -> lost, < 0.000001 -> A+
    # max_value:    float     = abs(max(covar_mat, key=abs))
    # mat_sum:      float     = sum([i for i in covar_mat])
    
    yaw   = quarternion2euler(pose.orientation).z
    if yaw < 0.0 :
      yaw = 2*pi+yaw
    self.__amcl_pose = Vector3(position.x, position.y, yaw)

    rospy.loginfo(rospy.get_caller_id() + " AMCL - x: %.4f, y: %.4f, yaw: %.4f; mean: %.4f", position.x, position.y, yaw, self.__msqrt)


  def __odom_pose_subscriber(self, odom: Odometry) -> None:
    pose:       Pose      = odom.pose.pose
    position:   Point     = pose.position
    covar_mat:  float[36] = odom.pose.covariance
    # mat:        float[36] =  around(covar_mat, 4)

    # mean_mat:   float[36] = mean(covar_mat)                 # > 0.0025 -> lost, < 0.0005 -> A+
    msqrt:      float[36] = mean(square(covar_mat))         # > 0.00005 -> lost, < 0.000001 -> A+
    # max_value:    float     = abs(max(covar_mat, key=abs))
    # mat_sum:      float     = sum([i for i in covar_mat])
    
    yaw   = quarternion2euler(pose.orientation).z
    if yaw < 0.0 :
      yaw = 2*pi+yaw
    self.__odom_pose = Vector3(position.x, position.y, yaw)
    # self.__robot_pose_pub.publish(self.__odom_pose)

    rospy.loginfo(rospy.get_caller_id() + " ODOM - x: %.4f, y: %.4f, yaw: %.4f; mean: %.4f", position.x, position.y, yaw, msqrt)


  def __initial_pose_subscriber(self, PoseWithCovarianceStamped) -> None:
    self.__initial_pose_call = True


  def __robot_pose_publisher(self) -> None:
    pose: RobotPoseStamped= RobotPoseStamped()
    ramp_amcl: int        = 1

    while not rospy.is_shutdown():
      if self.__amcl_pose != None:
        if self.__initial_pose_call:
          self.__initial_pose_call = False
          self.__offset_pose = self.__amcl_pose

        if self.__msqrt > 0.0005:
          ramp_amcl = 0
        elif self.__msqrt < 0.000001 and ramp_amcl == 0:
          ramp_amcl = 1
          self.__initial_pose_call = True

        angle = self.__offset_pose.z + self.__odom_pose.z
        if angle > 2*pi: angle -= 2*pi

        pose.header.stamp = rospy.Time.now()
        pose.lost = self.__msqrt > 0.0005
        pose.pose = Vector3(self.__offset_pose.x - self.__odom_pose.x, 
                            self.__offset_pose.y - self.__odom_pose.y, 
                            angle)
        self.__robot_pose_pub.publish(pose)
      self.__rate.sleep()


  def __node_shutdown(self) -> None:
    self.__amcl_pose_sub.unregister()
    self.__odom_pose_sub.unregister()
    self.__robot_pose_pub.unregister()

    rospy.loginfo("Perception Robot Pose - Shutdown")


def main() -> None:
  robot_pose = RobotPose()
  rospy.spin()
 

if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException as ROSie:
    rospy.loginfo("Perception Robot Pose", ROSie)

