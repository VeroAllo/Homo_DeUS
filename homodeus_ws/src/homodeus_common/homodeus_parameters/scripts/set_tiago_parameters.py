#!/usr/bin/env python


import rospy
from rospy import Publisher, Rate, Subscriber
from pal_control_msgs.msg import ActuatorCurrentLimit


def set_gripper_current_limit(current_limit: float):
  TOPIC: str = "/gripper_current_limit_controller/command"

  gripper_current_limit_pub: Publisher = Publisher(TOPIC, ActuatorCurrentLimit, queue_size=1)

  gripper_current_limit_ctrl: ActuatorCurrentLimit = ActuatorCurrentLimit()
  gripper_current_limit_ctrl.actuator_names = ['gripper_motor']
  gripper_current_limit_ctrl.current_limits = [current_limit]

  gripper_current_limit_pub.publish(gripper_current_limit_ctrl)

  # gripper_current_limit_pub.unregister()


def main() -> None:
  rospy.init_node("tiago_parameters", anonymous=False)
  rospy.loginfo("TIAGo parameters initialized")
  current_limit = 0.2
  rate: Rate = Rate(10)
  i = 0
  rospy.loginfo("TIAGo parameters published")
  while not rospy.is_shutdown():
    set_gripper_current_limit(current_limit)
    rate.sleep()
    i += 1
    if i > 10 :
      exit()

  rospy.loginfo("TIAGo parameters finished")


if __name__ == '__main__':
  try:
    main()
    rospy.spin()
  except rospy.ROSInterruptException as ROSie:
    rospy.loginfo("Node TIAGo parameters", ROSie)
