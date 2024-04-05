#!/usr/bin/env python3


import rospy
from rospy import Publisher, Subscriber

from homodeus_msgs.msg import Int8Stamped, Float32Stamped, RobotPoseStamped
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Header

from math import pi


class PID:
  """
  This is a class for PID controller.
 
  Attributes:
    __Kp (float): proportional gain.
    __Ki (float): integral gain.
    __Kd (float): derivate gain.
    __target (float): Desired target.
    __sum_error (float): Accumulation error for integral.
    __last_timestamp (float): Last timestamp for compute command (cmd) | secs.
    __last_error (float): Previous error compute.

  Sources:
    https://github.com/AlexCampanozzi/HomoDeUS/blob/d160ae677a2b0a792f981e30bc0b687817164044/homodeus_common/common_library/src/HomoDeUS_common_py/HomoDeUS_common_py.py#L251
    https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller
  """

  def __init__(self, Kp=1.0, Ki=1.0, Kd=1.0):
    """
    The constructor for PID class.

    Parameters:
      Kp (float): proportional gain.
      Ki (float): integral gain.
      Kd (float): derivate gain.
    """
    self.__Kp: float            = Kp
    self.__Ki: float            = Ki
    self.__Kd: float            = Kd

    self.__target: float        = None
    self.__sum_error: float     = 0.
    self.__last_timestamp: float= rospy.get_time()
    self.__last_error: float    = 0.


  def get_next_command(self, measured_value: float, epsilon: float = 1e-20) -> float:
    """
    The function to compute the next command.

    Parameters:
      measured_value (float): The measured value from control system.
      epsilon (float): Avoid a division by zero for derivate term.

    Returns:
      cmd (float): The next command to send control at the control system.
    """
    if self.__target == None:
      return None

    error: float = measured_value - self.__target

    # get loop interval time
    now = rospy.get_time()
    dT = now - self.__last_timestamp
    self.__last_timestamp = now

    # Proportial term
    P = self.__Kp * error

    # Integration term
    self.__sum_error += error * dT
    I = self.__Ki * self.__sum_error

    # Derivative term
    d_error = (error - self.__last_error) / (dT + epsilon)
    D = self.__Kd * d_error

    cmd = P + I + D
    return cmd


  def set_target(self, target: float, reset:bool = True) -> None:
    """
    The function set the target and reset .

    Parameters:
      target (float): The target from control system.
      reset (bool): Reset the variables internes of control system.
    """
    self.__target = target
    if reset:
      self.__sum_error: float     = 0.
      self.__last_timestamp: float= rospy.get_time()
      self.__last_error: float    = 0.


  def set_gains(self, kp: float = None, ki: float = None, kd: float = None) -> None:
    """
    The function set the PID's gains and reset the target.

    Parameters:
      kp (float): proportional gain.
      ki (float): integral gain.
      kd (float): derivate gain.
    """
    if kp != None:
      self.__Kp = kp
    if ki != None:
      self.__Ki = ki
    if kd != None:
      self.__Kd = ki

    self.set_target(None, True)


# # Access the Class docstring using help()
# help(PID)
# # Access the method docstring using help()
# help(PID.get_next_command)


class BaseRotate():
  """
  This is a class to rotate the robot base in node ROS.
 
  Attributes:
    __header (Header): header' msg from HBBA.
    __target (float): rotating distance to reach (rad).
    __dist (float): current rotation distance (rad).
    __pid (PID): PID controller.
    __sens_rotate (int): the direction of robot's rotation.
    __last_pose_z (float): last know robot orientation (rad).
    __robot_pose_sub (float): subscriber on __ROBOT_POSE_TOPIC.
    __base_rotate_cancel_sub (float): subscriber on __BASE_ROTATE_CANCEL_TOPIC (listen cancel goal from HBBA).
    __base_rotate_goal_sub (float): subscriber on __BASE_ROTATE_GOAL_TOPIC (listen the goal from HBBA).
    __twist_pub (float): publisher on __TWIST_TOPIC (send command motor).
    __base_rotate_result_pub (float): publisher on __BASE_ROTATE_RESULT_TOPIC (send goal result).

  Sources:

  """
  __BASE_ROTATE_CANCEL_TOPIC    = "/Homodeus/Behaviour/BaseRotate/Cancel"
  __BASE_ROTATE_GOAL_TOPIC      = "/Homodeus/Behaviour/BaseRotate/Request"
  __BASE_ROTATE_RESULT_TOPIC    = "/Homodeus/Behaviour/BaseRotate/Response"
  __BASE_ROTATE_STATUS_TOPIC    = "/Homodeus/Behaviour/BaseRotate/Status"
  
  __ROBOT_POSE_TOPIC            = "/Homodeus/Perception/RobotPose"
  __TWIST_TOPIC                 = "nav_vel"


  def __init__(self, node_name: str = 'base_rotate') -> None:
    """
    The constructor for BaseRotate class.

    Parameters:
      node_name (str): Node name.
    """
    # Initialise the node
    rospy.init_node(node_name, anonymous=True)

    # Initialise parameters
    self.__header: Header     = None
    self.__target: float      = None

    self.__dist: float        = 0.
    self.__pid: PID           = PID(0.5,0,0)

    self.__sens_rotate: int   = 1
    self.__last_pose_z: float = 0.

    # Subscriber
    self.__robot_pose_sub: Subscriber = Subscriber(self.__ROBOT_POSE_TOPIC, RobotPoseStamped, self.__robot_pose_subscriber)
    self.__base_rotate_cancel_sub: Subscriber = Subscriber(self.__BASE_ROTATE_CANCEL_TOPIC, Header, self.__base_rotate_cancel_subscriber)
    self.__base_rotate_goal_sub: Subscriber = Subscriber(self.__BASE_ROTATE_GOAL_TOPIC, Float32Stamped, self.__base_rotate_goal_subscriber)

    # Publisher
    self.__twist_pub: Publisher = Publisher(self.__TWIST_TOPIC, Twist, queue_size=1)
    self.__base_rotate_result_pub: Publisher = Publisher(self.__BASE_ROTATE_RESULT_TOPIC, Int8Stamped, queue_size=1)

    rospy.on_shutdown(self.__close_connection_node)
    rospy.loginfo("Behavior Base Rotate initialized")


  def __send_result(self, value: int) -> None:
    """
    The function to send result to HBBA.

    Parameters:
      value (int): Result to send (1: ok).
    """
    result: Int8Stamped = Int8Stamped()
    result.header       = self.__header
    result.header.stamp = rospy.Time.now()
    result.value        = value
    self.__base_rotate_result_pub.publish(result)


  def __base_rotate_cancel_subscriber(self, cancel: Header) -> None:
    """
    The function listening cancel goal from HBBA

    Parameters:
      value (int): Result to send (1: ok).
    """
    rospy.loginfo("Behavior Base Rotate - Cancel Target")
    
    # cancel current command motor
    self.__twist_pub.publish(Twist())

    # reinitialise the target
    self.__dist = 0.
    self.__target = None
    self.__pid.set_target(self.__target)

    # send to HBBA that goal is canccel
    self.__header = cancel.header
    self.__send_result(1)


  def __base_rotate_goal_subscriber(self, target: Float32Stamped) -> None:
    """
    The function listening set goal from HBBA

    Parameters:
      target (Float32Stamped): target msg from HBBA.
    """
    rospy.loginfo("Behavior Base Rotate - Get Target")

    # get header from HBBA msg
    self.__header = target.header

    # initialise target from HBBA msg
    self.__dist = 0.
    self.__target = target.value
    self.__pid.set_target(self.__target)


  def __robot_pose_subscriber(self, robot_pose: RobotPoseStamped) -> None:
    """
    The function listening the robot pose

    Parameters:
      pose (RobotPoseStamped):  header with stamp,
                                lost if value is one,
                                pose (x, y, z:XY plan's angle);
    """
    pose: Vector3 = robot_pose.pose
    if self.__target == None:
      self.__last_pose_z  = pose.z
      self.__sens_rotate  = 1
      return

    delta_z = abs(pose.z - self.__last_pose_z)
    if delta_z > pi: delta_z = 2*pi - delta_z
    self.__dist += self.__sens_rotate * delta_z

    # send msg to HBBA if reach target
    if abs(self.__target - self.__dist) < 0.005:
      rospy.loginfo("Behavior Base Rotate - Reach Target")

      self.__dist = 0.
      self.__target = None
      self.__pid.set_target(self.__target)

      self.__send_result(1)
      self.__twist_pub.publish(Twist())

    # request the angular speed and limit speed to ±0.5 rad/s
    angular_speed : float = self.__pid.get_next_command(self.__dist)
    if angular_speed != None:
      rospy.loginfo("Behavior Base Rotate - Move To Target")

      if angular_speed > 0.5: angular_speed = 0.5
      elif angular_speed < -0.5: angular_speed = -0.5

      twist : Twist = Twist()
      twist.angular.z = angular_speed
      self.__twist_pub.publish(twist)

      self.__sens_rotate = -1 if angular_speed > 0. else 1
      self.__last_pose_z = pose.z


  def __close_connection_node(self) -> None:
    """
    The function close all subscribers & publisher before close node
    """
    self.__robot_pose_sub.unregister()
    self.__base_rotate_cancel_sub.unregister()
    self.__base_rotate_goal_sub.unregister()

    self.__twist_pub.unregister()
    self.__base_rotate_result_pub.unregister()

    rospy.loginfo("Behavior Base Rotate - Shutdown")


# # Access the Class docstring using help()
# help(BaseRotate)